#!/usr/bin/env python3
"""Stream a ROS image topic as raw video frames for a host-side ffmpeg.

The recorder is deliberately independent of Gazebo's GUI camera.  It reads
the selected sensor topic, so the resulting video is the robot POV rather
than a fixed world-view window.
"""

import argparse
import signal
import sys
import threading
import time

import rospy
from sensor_msgs.msg import Image


ENCODINGS = {
    "rgb8": ("rgb24", 3),
    "bgr8": ("bgr24", 3),
    "rgba8": ("rgba", 4),
    "bgra8": ("bgra", 4),
}


def packed_frame(message):
    encoding = message.encoding.lower()
    if encoding not in ENCODINGS:
        raise RuntimeError(
            "unsupported image encoding %r; expected one of %s"
            % (message.encoding, ", ".join(sorted(ENCODINGS)))
        )
    pixel_format, channels = ENCODINGS[encoding]
    row_bytes = int(message.width) * channels
    if int(message.step) < row_bytes:
        raise RuntimeError(
            "image step %d is smaller than packed row size %d"
            % (message.step, row_bytes)
        )
    payload = bytes(message.data)
    expected = int(message.step) * int(message.height)
    if len(payload) < expected:
        raise RuntimeError(
            "image payload is short: got %d bytes, expected at least %d"
            % (len(payload), expected)
        )
    if int(message.step) == row_bytes:
        return payload[: row_bytes * int(message.height)], pixel_format
    return b"".join(
        payload[row * int(message.step) : row * int(message.step) + row_bytes]
        for row in range(int(message.height))
    ), pixel_format


class ImageStreamer:
    def __init__(self, topic, fps, emit_frame_times=False, repeat_latest=False):
        self.topic = topic
        self.period = 1.0 / max(float(fps), 0.1)
        self.emit_frame_times = emit_frame_times
        self.repeat_latest = repeat_latest
        self.ready = threading.Event()
        self.stop = threading.Event()
        self.spec = None
        self.last_emit = 0.0
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.frame_index = 0
        self.error = None

    def _emit_frame(self, frame):
        try:
            sys.stdout.buffer.write(frame)
            sys.stdout.buffer.flush()
        except BrokenPipeError:
            self.stop.set()
            rospy.signal_shutdown("ffmpeg pipe closed")
            return False
        if self.emit_frame_times:
            print(
                "frame timestamp: index=%d epoch=%.6f"
                % (self.frame_index, time.time()),
                file=sys.stderr,
                flush=True,
            )
        self.frame_index += 1
        return True

    def get_latest_frame(self):
        with self.frame_lock:
            return self.latest_frame

    def callback(self, message):
        if self.stop.is_set():
            return
        try:
            frame, pixel_format = packed_frame(message)
        except Exception as error:
            self.error = str(error)
            self.stop.set()
            rospy.signal_shutdown(self.error)
            return

        current_spec = (
            int(message.width),
            int(message.height),
            pixel_format,
        )
        first_spec = self.spec is None
        if first_spec:
            self.spec = current_spec
        elif self.spec != current_spec:
            self.error = "image format changed from %s to %s" % (
                self.spec,
                current_spec,
            )
            self.stop.set()
            rospy.signal_shutdown(self.error)
            return

        if self.repeat_latest:
            with self.frame_lock:
                self.latest_frame = frame
            if first_spec:
                self.ready.set()
                print(
                    "stream ready: topic=%s width=%d height=%d pixel_format=%s mode=repeat_latest"
                    % (self.topic, *current_spec),
                    file=sys.stderr,
                    flush=True,
                )
            return

        if first_spec:
            self.ready.set()
            print(
                "stream ready: topic=%s width=%d height=%d pixel_format=%s"
                % (self.topic, *current_spec),
                file=sys.stderr,
                flush=True,
            )

        now = time.monotonic()
        if now - self.last_emit < self.period:
            return
        self.last_emit = now
        self._emit_frame(frame)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--wait-timeout", type=float, default=30.0)
    parser.add_argument(
        "--emit-frame-times",
        action="store_true",
        help="write one epoch wall-clock line per emitted frame to stderr",
    )
    parser.add_argument(
        "--repeat-latest",
        action="store_true",
        help="emit at a fixed rate and hold the latest ROS frame between updates",
    )
    args = parser.parse_args()

    rospy.init_node("robot_pov_image_stream", anonymous=True, disable_signals=True)
    streamer = ImageStreamer(
        args.topic, args.fps, args.emit_frame_times, args.repeat_latest
    )
    rospy.Subscriber(
        args.topic,
        Image,
        streamer.callback,
        queue_size=2,
        buff_size=8 * 1024 * 1024,
    )

    def stop_handler(signum, _frame):
        streamer.stop.set()
        rospy.signal_shutdown("signal %d" % signum)

    signal.signal(signal.SIGTERM, stop_handler)
    signal.signal(signal.SIGINT, stop_handler)

    deadline = time.monotonic() + max(args.wait_timeout, 0.1)
    while not streamer.ready.is_set() and not streamer.stop.is_set():
        if time.monotonic() >= deadline:
            streamer.error = "no image received on %s within %.1fs" % (
                args.topic,
                args.wait_timeout,
            )
            streamer.stop.set()
            rospy.signal_shutdown(streamer.error)
            break
        time.sleep(0.05)

    if not streamer.ready.is_set():
        print("robot POV stream FAILED: %s" % (streamer.error or "stopped"), file=sys.stderr)
        return 2

    if args.repeat_latest:
        next_emit = time.monotonic()
        while not streamer.stop.is_set() and not rospy.is_shutdown():
            now = time.monotonic()
            if now < next_emit:
                time.sleep(min(0.02, next_emit - now))
                continue
            frame = streamer.get_latest_frame()
            if frame is not None and not streamer._emit_frame(frame):
                break
            next_emit += streamer.period
            if next_emit <= now:
                next_emit = now + streamer.period
    else:
        while not streamer.stop.is_set() and not rospy.is_shutdown():
            time.sleep(0.2)
    return 0 if streamer.error in (None, "ffmpeg pipe closed") else 2


if __name__ == "__main__":
    raise SystemExit(main())
