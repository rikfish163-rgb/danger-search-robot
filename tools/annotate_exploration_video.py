#!/usr/bin/env python3
"""Bind a robot-POV video to the three-floor mission log.

The recorder writes video frames at a fixed output rate while the mission log
contains elapsed wall-clock values.  When the stream log contains per-frame
epoch wall-clock values and the mission log records its epoch origin, event
positions are matched to the nearest encoded frame.  Older recordings without
that sidecar use an explicitly marked linear approximation instead.

Outputs:
  * ``*_annotated.mp4`` with the event labels burned into the video;
  * ``*_timeline.json`` with machine-readable event timestamps;
  * ``*_timeline.srt`` with the subtitle/event track used for the overlay.
"""

from __future__ import annotations

import argparse
import ast
import bisect
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path


MISSION_LINE = re.compile(
    r"^\[wall (?P<wall>[0-9]+(?:\.[0-9]+)?)\]"
    r"\[ros (?P<ros>[0-9]+(?:\.[0-9]+)?)\] (?P<message>.*)$"
)
FRAME_LINE = re.compile(
    r"^frame timestamp: index=(?P<index>[0-9]+)"
    r" epoch=(?P<epoch>[0-9]+(?:\.[0-9]+)?)$"
)


def fail(message: str) -> "NoReturn":
    raise SystemExit("annotate exploration video FAILED: " + message)


def ffprobe_video(video: Path) -> dict:
    command = [
        "ffprobe",
        "-v",
        "error",
        "-show_entries",
        "format=duration:stream=avg_frame_rate,nb_frames,width,height,codec_name",
        "-of",
        "json",
        str(video),
    ]
    try:
        raw = subprocess.check_output(command, text=True)
    except (OSError, subprocess.CalledProcessError) as error:
        fail("ffprobe could not inspect %s: %s" % (video, error))
    try:
        data = json.loads(raw)
        duration = float(data["format"]["duration"])
        stream = data.get("streams", [{}])[0]
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        fail("invalid ffprobe output: %s" % error)
    frame_rate = parse_frame_rate(stream.get("avg_frame_rate", "10/1"))
    return {
        "duration_seconds": duration,
        "frame_rate": frame_rate,
        "codec_name": stream.get("codec_name"),
        "width": stream.get("width"),
        "height": stream.get("height"),
        "nb_frames": stream.get("nb_frames"),
    }


def parse_frame_rate(value: str) -> float:
    try:
        numerator, denominator = value.split("/", 1)
        result = float(numerator) / float(denominator)
        return result if result > 0 else 10.0
    except (AttributeError, ValueError, ZeroDivisionError):
        return 10.0


def parse_position(value: str):
    try:
        parsed = ast.literal_eval(value)
    except (SyntaxError, ValueError):
        return None
    if isinstance(parsed, (list, tuple)) and len(parsed) >= 3:
        try:
            return [float(parsed[0]), float(parsed[1]), float(parsed[2])]
        except (TypeError, ValueError):
            return None
    return None


def floor_name(floor: int) -> str:
    return "第%d层" % (floor + 1)


def floor_name_en(floor: int) -> str:
    return "FLOOR %d" % (floor + 1)


def add_event(events, event_id, event_type, wall, ros, label_zh, label, **extra):
    event = {
        "id": event_id,
        "type": event_type,
        "mission_wall_seconds": wall,
        "mission_ros_seconds": ros,
        "label_zh": label_zh,
        "label": label,
    }
    event.update(extra)
    events.append(event)


def parse_mission_log(log_path: Path):
    events = []
    red_candidates = 0
    red_confirmations = 0
    mission_origin_epoch = None

    try:
        lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as error:
        fail("cannot read mission log %s: %s" % (log_path, error))

    for line in lines:
        match = MISSION_LINE.match(line)
        if not match:
            continue
        wall = float(match.group("wall"))
        ros = float(match.group("ros"))
        message = match.group("message")

        origin_match = re.match(r"MISSION_WALL_ORIGIN_EPOCH=([0-9.]+)$", message)
        if origin_match:
            mission_origin_epoch = float(origin_match.group(1))
            continue

        if message == "FULL THREE-FLOOR RUN START":
            add_event(
                events,
                "mission_start",
                "mission_start",
                wall,
                ros,
                "开始三层完整探索",
                "THREE-FLOOR MISSION START",
            )
            continue

        floor_match = re.match(r"FLOOR (\d+) START", message)
        if floor_match:
            floor = int(floor_match.group(1))
            is_first_floor = floor == 0
            add_event(
                events,
                "floor_%d_start" % floor,
                "floor_start",
                wall,
                ros,
                (
                    "%s开始搜索" % floor_name(floor)
                    if is_first_floor
                    else "到达%s，开始搜索" % floor_name(floor)
                ),
                (
                    "%s SEARCH START" % floor_name_en(floor)
                    if is_first_floor
                    else "%s REACHED; SEARCH START" % floor_name_en(floor)
                ),
                floor=floor,
                overlay=not is_first_floor,
            )
            continue

        candidate_match = re.match(r"VALID danger observation:\s*(.*)$", message)
        if candidate_match:
            red_candidates += 1
            add_event(
                events,
                "red_ball_candidate_%d" % red_candidates,
                "red_ball_candidate",
                wall,
                ros,
                "发现红球候选 #%d" % red_candidates,
                "RED BALL CANDIDATE #%d" % red_candidates,
                sequence=red_candidates,
                position=parse_position(candidate_match.group(1)),
                overlay=False,
            )
            continue

        confirmed_match = re.match(r"CONFIRMED danger observation:\s*(.*)$", message)
        if confirmed_match:
            red_confirmations += 1
            add_event(
                events,
                "red_ball_confirmed_%d" % red_confirmations,
                "red_ball_confirmed",
                wall,
                ros,
                "确认红球 #%d" % red_confirmations,
                "RED BALL #%d CONFIRMED" % red_confirmations,
                sequence=red_confirmations,
                position=parse_position(confirmed_match.group(1)),
            )
            continue

        complete_match = re.match(r"FLOOR (\d+) COMPLETE", message)
        if complete_match:
            floor = int(complete_match.group(1))
            add_event(
                events,
                "floor_%d_complete" % floor,
                "floor_complete",
                wall,
                ros,
                "%s搜索完成" % floor_name(floor),
                "%s SEARCH COMPLETE" % floor_name_en(floor),
                floor=floor,
                overlay=False,
            )
            continue

        transition_match = re.match(
            r"TRANSITION (\d+) -> (\d+): moving to elevator lobby", message
        )
        if transition_match:
            from_floor = int(transition_match.group(1))
            to_floor = int(transition_match.group(2))
            direction = "上楼" if to_floor > from_floor else "下楼"
            direction_en = "UP" if to_floor > from_floor else "DOWN"
            add_event(
                events,
                "elevator_%d_to_%d_start" % (from_floor, to_floor),
                "elevator_start",
                wall,
                ros,
                "%s搜索完成，电梯%s：%s → %s" % (
                    floor_name(from_floor),
                    direction,
                    floor_name(from_floor),
                    floor_name(to_floor),
                ),
                "%s DONE; ELEVATOR %s: %d -> %d" % (
                    floor_name_en(from_floor),
                    direction_en,
                    from_floor + 1,
                    to_floor + 1,
                ),
                from_floor=from_floor,
                to_floor=to_floor,
                direction=direction,
                overlay=True,
            )
            continue

        move_match = re.match(r"move service accepted=True current_floor=(\d+)", message)
        if move_match:
            to_floor = int(move_match.group(1))
            previous = next(
                (
                    event
                    for event in reversed(events)
                    if event["type"] == "elevator_start"
                    and event["to_floor"] == to_floor
                    and event["mission_wall_seconds"] <= wall
                ),
                None,
            )
            from_floor = previous["from_floor"] if previous else None
            direction = previous["direction"] if previous else "到达"
            add_event(
                events,
                "elevator_%s_arrived_%d"
                % ("from_%d" % from_floor if from_floor is not None else "", to_floor),
                "elevator_arrived",
                wall,
                ros,
                "电梯到达%s" % floor_name(to_floor),
                "ELEVATOR ARRIVED %s" % floor_name_en(to_floor),
                from_floor=from_floor,
                to_floor=to_floor,
                direction=direction,
                overlay=True,
            )
            continue

        reached_match = re.match(r"TRANSITION (\d+) -> (\d+) COMPLETE", message)
        if reached_match:
            from_floor = int(reached_match.group(1))
            to_floor = int(reached_match.group(2))
            add_event(
                events,
                "floor_%d_reached" % to_floor,
                "floor_reached",
                wall,
                ros,
                "已到达%s" % floor_name(to_floor),
                "%s REACHED" % floor_name_en(to_floor),
                from_floor=from_floor,
                to_floor=to_floor,
                overlay=False,
            )
            continue

        if message == "FINAL RETURN: descending to floor 0":
            add_event(
                events,
                "final_return_start",
                "final_return",
                wall,
                ros,
                "三层搜索完成，开始返回一层",
                "FLOOR 3 COMPLETE; FINAL RETURN TO FLOOR 1",
                overlay=False,
            )
            continue

        if message.startswith("FINAL POSE="):
            add_event(
                events,
                "returned_to_start",
                "returned_to_start",
                wall,
                ros,
                "返回起点并完成定位",
                "RETURNED TO START",
                overlay=False,
            )
            continue

        if message == "FULL THREE-FLOOR RUN COMPLETE":
            add_event(
                events,
                "mission_complete",
                "mission_complete",
                wall,
                ros,
                "三层完整探索完成",
                "RETURNED TO START; THREE-FLOOR MISSION COMPLETE",
            )

    if not events:
        fail("no timestamped mission events found in %s" % log_path)
    if not any(event["type"] == "mission_start" for event in events):
        fail("mission start event is missing from %s" % log_path)
    if not any(event["type"] == "mission_complete" for event in events):
        fail("mission complete event is missing from %s" % log_path)
    return events, mission_origin_epoch


def parse_frame_log(frame_log: Path):
    frames = []
    try:
        lines = frame_log.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as error:
        fail("cannot read frame log %s: %s" % (frame_log, error))
    for line in lines:
        match = FRAME_LINE.match(line.strip())
        if match:
            frames.append((float(match.group("epoch")), int(match.group("index"))))
    frames.sort()
    return frames


def map_events(events, video_info, frame_log: Path | None, mission_origin_epoch):
    mission_start = next(
        event["mission_wall_seconds"]
        for event in events
        if event["type"] == "mission_start"
    )
    mission_end = next(
        event["mission_wall_seconds"]
        for event in reversed(events)
        if event["type"] == "mission_complete"
    )
    if mission_end <= mission_start:
        fail("mission wall-clock range is not increasing")

    frames = parse_frame_log(frame_log) if frame_log else []
    frame_walls = [item[0] for item in frames]
    if frames and mission_origin_epoch is not None:
        mapping = {
            "kind": "nearest_encoded_frame_epoch",
            "approximate": False,
            "note": "event epoch matched to the nearest emitted frame; encoded frame time is frame_index / fps",
            "mission_wall_start": mission_start,
            "mission_wall_end": mission_end,
            "mission_wall_origin_epoch": mission_origin_epoch,
            "frame_log": str(frame_log),
        }
    else:
        mapping = {
            "kind": "linear_from_mission_wall",
            "approximate": True,
            "note": (
                "recording has no epoch-aligned per-frame sidecar; timestamps are "
                "an approximate linear alignment"
            ),
            "mission_wall_start": mission_start,
            "mission_wall_end": mission_end,
        }

    duration = video_info["duration_seconds"]
    fps = video_info["frame_rate"]
    for event in events:
        wall = event["mission_wall_seconds"]
        if frames and mission_origin_epoch is not None:
            event_epoch = mission_origin_epoch + wall
            position = bisect.bisect_left(frame_walls, event_epoch)
            candidates = []
            if position < len(frames):
                candidates.append(frames[position])
            if position > 0:
                candidates.append(frames[position - 1])
            _, frame_index = min(
                candidates, key=lambda item: abs(item[0] - event_epoch)
            )
            video_seconds = frame_index / fps
            event["matched_frame_index"] = frame_index
            event["event_epoch"] = round(event_epoch, 6)
        else:
            ratio = (wall - mission_start) / (mission_end - mission_start)
            video_seconds = ratio * duration
        video_seconds = max(0.0, min(duration, video_seconds))
        event["video_seconds"] = round(video_seconds, 3)
        event["subtitle_video_seconds"] = round(
            max(0.0, duration - 4.0)
            if video_seconds >= duration - 0.1
            else video_seconds,
            3,
        )
        event["video_timestamp"] = srt_timestamp(event["video_seconds"])
    mapping["video_duration_seconds"] = duration
    mapping["video_frame_rate"] = fps
    return mapping


def srt_timestamp(seconds: float) -> str:
    milliseconds = max(0, int(round(seconds * 1000)))
    hours, remainder = divmod(milliseconds, 3_600_000)
    minutes, remainder = divmod(remainder, 60_000)
    seconds_value, millis = divmod(remainder, 1000)
    return "%02d:%02d:%02d,%03d" % (hours, minutes, seconds_value, millis)


def subtitle_duration(event_type: str) -> float:
    if event_type in {"red_ball_candidate", "red_ball_confirmed"}:
        return 6.0
    if event_type in {"elevator_start", "elevator_arrived", "final_return"}:
        return 5.0
    return 4.0


def write_srt(events, path: Path, duration: float):
    lines = []
    overlay_events = [event for event in events if event.get("overlay", True)]
    for index, event in enumerate(overlay_events, start=1):
        # Keep the exact event timestamp in JSON, but leave enough screen
        # time for a completion label logged at the final encoded frame.
        start = event.get("subtitle_video_seconds", event["video_seconds"])
        end = min(duration, start + subtitle_duration(event["type"]))
        if index < len(overlay_events):
            next_start = overlay_events[index].get(
                "subtitle_video_seconds", overlay_events[index]["video_seconds"]
            )
            if next_start > start:
                end = min(end, next_start - 0.05)
        if end <= start:
            end = min(duration, start + 0.5)
        text = "%s\n%s" % (event["label_zh"], event["label"])
        lines.extend(
            [
                str(index),
                "%s --> %s" % (srt_timestamp(start), srt_timestamp(end)),
                text,
                "",
            ]
        )
    try:
        path.write_text("\n".join(lines), encoding="utf-8")
    except OSError as error:
        fail("cannot write subtitle file %s: %s" % (path, error))


def render_video(video: Path, subtitles: Path, output: Path):
    if shutil.which("ffmpeg") is None:
        fail("ffmpeg is unavailable")
    filter_graph = (
        "subtitles=%s:force_style="
        "'FontName=Noto Sans CJK SC,FontSize=20,PrimaryColour=&H00FFFFFF,"
        "OutlineColour=&H00000000,BorderStyle=3,BackColour=&H80000000,"
        "MarginV=34'"
    ) % subtitles
    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "warning",
        "-y",
        "-i",
        str(video),
        "-vf",
        filter_graph,
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "fast",
        "-crf",
        "20",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output),
    ]
    print("rendering annotated video: %s" % output, file=sys.stderr)
    try:
        subprocess.run(command, check=True)
    except (OSError, subprocess.CalledProcessError) as error:
        fail("ffmpeg subtitle rendering failed: %s" % error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", required=True, type=Path)
    parser.add_argument("--log", required=True, type=Path, dest="mission_log")
    parser.add_argument(
        "--frame-log",
        type=Path,
        help="optional stream stderr containing frame timestamp lines",
    )
    parser.add_argument("--output", type=Path, help="annotated MP4 path")
    parser.add_argument("--timeline", type=Path, help="timeline JSON path")
    parser.add_argument("--subtitles", type=Path, help="SRT path")
    parser.add_argument(
        "--no-render",
        action="store_true",
        help="write JSON/SRT only and skip ffmpeg rendering",
    )
    args = parser.parse_args()

    if not args.video.is_file():
        fail("video does not exist: %s" % args.video)
    if not args.mission_log.is_file():
        fail("mission log does not exist: %s" % args.mission_log)
    if args.frame_log and not args.frame_log.is_file():
        fail("frame log does not exist: %s" % args.frame_log)

    video = args.video.resolve()
    output = (args.output or video.with_name(video.stem + "_annotated.mp4")).resolve()
    timeline = (args.timeline or video.with_name(video.stem + "_timeline.json")).resolve()
    subtitles = (args.subtitles or video.with_name(video.stem + "_timeline.srt")).resolve()
    video_info = ffprobe_video(video)
    events, mission_origin_epoch = parse_mission_log(args.mission_log.resolve())
    mapping = map_events(
        events,
        video_info,
        args.frame_log.resolve() if args.frame_log else None,
        mission_origin_epoch,
    )

    document = {
        "schema": "three_floor_exploration_timeline_v1",
        "video": str(video),
        "annotated_video": str(output),
        "mission_log": str(args.mission_log.resolve()),
        "video_info": video_info,
        "mapping": mapping,
        "events": events,
        "summary": {
            "red_ball_candidates": sum(
                event["type"] == "red_ball_candidate" for event in events
            ),
            "red_balls_confirmed": sum(
                event["type"] == "red_ball_confirmed" for event in events
            ),
            "floors_started": sum(event["type"] == "floor_start" for event in events),
            "elevator_movements": sum(
                event["type"] == "elevator_start" for event in events
            ),
        },
    }
    try:
        timeline.write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    except OSError as error:
        fail("cannot write timeline %s: %s" % (timeline, error))
    write_srt(events, subtitles, video_info["duration_seconds"])
    if not args.no_render:
        render_video(video, subtitles, output)

    print("timeline JSON: %s" % timeline)
    print("timeline SRT: %s" % subtitles)
    if not args.no_render:
        print("annotated video: %s" % output)
    print(
        "events=%d red_ball_confirmed=%d approximate=%s"
        % (
            len(events),
            document["summary"]["red_balls_confirmed"],
            mapping["approximate"],
        )
    )


if __name__ == "__main__":
    main()
