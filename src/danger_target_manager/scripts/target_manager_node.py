#!/usr/bin/env python3
"""ROS adapter providing timestamp frame buffering around TargetManagerCore."""
from __future__ import annotations

import math
import time
from typing import Dict, List

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from danger_target_manager.msg import ConfirmedDanger, DangerObservation
from danger_target_manager.core import Observation, TargetManagerConfig, TargetManagerCore


class TargetManagerNode:
    """Own ROS I/O, leaving all target decisions in the pure core."""

    def __init__(self) -> None:
        values = {name: rospy.get_param("~" + name, default) for name, default in {
            "expected_frame": "world", "window_size": 7, "min_valid_count": 5,
            "candidate_radius": 0.20, "association_radius": 0.40, "history_dedup_radius": 0.40,
            "max_consecutive_misses": 3, "observation_timeout": 1.0,
            "update_reported_position": True, "reported_position_alpha": 0.10}.items()}
        try:
            self.core = TargetManagerCore(TargetManagerConfig(**values))
        except ValueError as error:
            rospy.logfatal("Invalid target-manager parameter: %s", error)
            raise
        self.flush_delay = float(rospy.get_param("~frame_flush_delay", 0.05))
        self.diagnostic_period = float(rospy.get_param("~diagnostic_period", 1.0))
        if self.flush_delay <= 0 or self.diagnostic_period <= 0:
            raise ValueError("frame_flush_delay and diagnostic_period must be > 0")
        self.input_topic = rospy.get_param("~input_topic", "/danger_observation")
        self.output_topic = rospy.get_param("~output_topic", "/confirmed_danger")
        self.reset_service_name = rospy.get_param("~reset_service", "/target_manager/reset")
        self._frames: Dict[float, List[Observation]] = {}
        self._arrivals: Dict[float, float] = {}
        self.publisher = rospy.Publisher(self.output_topic, ConfirmedDanger, queue_size=20)
        self.subscriber = rospy.Subscriber(self.input_topic, DangerObservation, self._on_observation, queue_size=100)
        self.reset_service = rospy.Service(self.reset_service_name, Trigger, self._on_reset)
        self.timer = rospy.Timer(rospy.Duration(min(self.flush_delay, 0.05)), self._on_timer)
        self.last_diag = 0.0
        rospy.loginfo("target manager ready: %s -> %s, reset=%s", self.input_topic, self.output_topic, self.reset_service_name)

    def _on_observation(self, message: DangerObservation) -> None:
        stamp = message.header.stamp.to_sec()
        if not math.isfinite(stamp) or stamp <= 0:
            rospy.logwarn_throttle(2.0, "Rejecting observation with invalid timestamp")
            return
        diagnostics = self.core.get_diagnostics()
        last = diagnostics["last_processed_stamp"]
        if last is not None and stamp < float(last):
            rospy.logwarn_throttle(2.0, "Rejecting time-reversed observation stamp %.9f", stamp)
            return
        if message.header.frame_id != self.core.config.expected_frame:
            rospy.logwarn_throttle(2.0, "Rejecting observation from frame '%s'", message.header.frame_id)
            return
        position = (message.center.x, message.center.y, message.center.z)
        spatial_valid = bool(message.valid) and all(math.isfinite(value) for value in position)
        if message.valid and not spatial_valid:
            rospy.logwarn_throttle(2.0, "Rejecting non-finite observation coordinates")
        # A valid=false message intentionally remains in the frame as a miss signal.
        observation = Observation(position if spatial_valid else None, spatial_valid, message.header.frame_id, message.detector_confidence)
        for old_stamp in sorted(key for key in self._frames if key < stamp):
            self._flush(old_stamp)
        self._frames.setdefault(stamp, []).append(observation)
        self._arrivals.setdefault(stamp, time.monotonic())

    def _flush(self, stamp: float) -> None:
        observations = self._frames.pop(stamp, None)
        self._arrivals.pop(stamp, None)
        if observations is None:
            return
        for target in self.core.process_frame(stamp, observations):
            message = ConfirmedDanger()
            message.header.stamp = rospy.Time.from_sec(target.stamp)
            message.header.frame_id = self.core.config.expected_frame
            message.track_id = target.track_id
            message.position.x, message.position.y, message.position.z = target.position
            message.confidence = target.confidence
            self.publisher.publish(message)
            rospy.loginfo("confirmed target id=%d position=(%.3f, %.3f, %.3f)", target.track_id, *target.position)

    def _on_timer(self, _event: rospy.timer.TimerEvent) -> None:
        now_mono = time.monotonic()
        for stamp in sorted(key for key, arrival in self._arrivals.items() if now_mono - arrival >= self.flush_delay):
            self._flush(stamp)
        self.core.expire(rospy.Time.now().to_sec())
        now = rospy.get_time()
        if now - self.last_diag >= self.diagnostic_period:
            self.last_diag = now
            diag = self.core.get_diagnostics()
            rospy.loginfo("diagnostic active=%d history=%d tracks=%s", diag["active_count"], diag["history_count"], diag["tracks"])

    def _on_reset(self, _request: Trigger) -> TriggerResponse:
        self._frames.clear(); self._arrivals.clear(); self.core.reset()
        rospy.loginfo("target manager reset")
        return TriggerResponse(success=True, message="Target manager reset: candidates, history, frames, timestamps, and track IDs cleared.")


def main() -> None:
    """Start the target manager ROS node."""
    rospy.init_node("target_manager")
    TargetManagerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
