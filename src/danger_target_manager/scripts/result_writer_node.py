#!/usr/bin/env python3
"""Write confirmed danger positions to a JSON result file.

The node accepts only ConfirmedDanger messages expressed in the configured
frame.  Result export uses ``world`` by default.  An alternative frame may be
used only in an isolated diagnostic run with an explicit output path.
"""

import json
import math
import os
import tempfile
import threading
from pathlib import Path
from typing import Dict, Optional, Tuple

import rospy
import rospkg
from std_srvs.srv import Trigger, TriggerResponse

from danger_target_manager.msg import ConfirmedDanger


Position = Tuple[float, float, float]


class ResultWriterNode:
    """Collect confirmed targets and atomically maintain the result file."""

    def __init__(self) -> None:
        self._lock = threading.RLock()

        self._confirmed_topic = rospy.get_param(
            "~confirmed_topic", "/confirmed_danger"
        )
        self._expected_frame = self._normalize_frame(
            rospy.get_param("~expected_frame", "world")
        )
        self._allow_non_world_test_output = bool(
            rospy.get_param("~allow_non_world_test_output", False)
        )
        self._team_scene_info = str(
            rospy.get_param("~team_scene_info", "")
        ).strip()
        self._default_result_file = str(
            rospy.get_param("~default_result_file", "")
        ).strip()
        self._runtime_result_file = str(
            rospy.get_param("~runtime_result_file", "")
        ).strip()
        self._workspace_root_param = str(
            rospy.get_param("~workspace_root", "")
        ).strip()
        self._spatial_dedup_radius = float(
            rospy.get_param("~spatial_dedup_radius", 0.40)
        )
        self._fixed_exploration_time = float(
            rospy.get_param("~fixed_exploration_time", -1.0)
        )

        self._validate_parameters()
        self._workspace_root = self._find_workspace_root()
        self._result_path = self._resolve_result_path()
        self._runtime_result_path = self._resolve_runtime_result_path()
        self._protect_default_path_during_non_world_test()

        # track_id -> (position, confidence).  A dictionary makes repeated
        # updates for the same confirmed track idempotent.
        self._targets: Dict[int, Tuple[Position, float]] = {}
        self._start_time: Optional[rospy.Time] = None
        self._end_time: Optional[rospy.Time] = None

        self._subscriber = rospy.Subscriber(
            self._confirmed_topic,
            ConfirmedDanger,
            self._confirmed_callback,
            queue_size=20,
        )
        self._reset_service = rospy.Service(
            "~reset", Trigger, self._handle_reset
        )
        self._start_service = rospy.Service(
            "~start", Trigger, self._handle_start
        )
        self._finalize_service = rospy.Service(
            "~finalize", Trigger, self._handle_finalize
        )

        # Create a valid empty document on the native runtime filesystem.
        # Public NTFS output is published by the bounded post-run publisher.
        self._write_result_file()

        rospy.loginfo(
            "Result writer ready: topic=%s expected_frame=%s runtime=%s public=%s",
            self._confirmed_topic,
            self._expected_frame,
            self._runtime_result_path,
            self._result_path,
        )

    @staticmethod
    def _normalize_frame(frame: str) -> str:
        """ROS frame ids are compared without an optional leading slash."""
        return str(frame).strip().lstrip("/")

    def _validate_parameters(self) -> None:
        if not self._expected_frame:
            raise ValueError("~expected_frame must not be empty")
        if (
            self._expected_frame != "world"
            and not self._allow_non_world_test_output
        ):
            raise ValueError(
                "World-frame output is required by the default result mode. "
                "For an isolated diagnostic run, set "
                "~allow_non_world_test_output:=true and provide an "
                "absolute ~default_result_file outside the default results path."
            )
        if not math.isfinite(self._spatial_dedup_radius):
            raise ValueError("~spatial_dedup_radius must be finite")
        if self._spatial_dedup_radius < 0.0:
            raise ValueError("~spatial_dedup_radius must be non-negative")
        if not math.isfinite(self._fixed_exploration_time):
            raise ValueError("~fixed_exploration_time must be finite")

    def _find_workspace_root(self) -> Path:
        """Resolve the workspace root in source, devel, or install layouts."""
        if self._workspace_root_param:
            configured = Path(os.path.expandvars(
                os.path.expanduser(self._workspace_root_param)
            )).resolve()
            if not configured.is_dir():
                raise FileNotFoundError(
                    "Configured workspace root does not exist: "
                    + str(configured)
                )
            return configured

        environment_root = os.environ.get("SIMENV_ROOT", "").strip()
        if environment_root:
            configured = Path(os.path.expandvars(
                os.path.expanduser(environment_root)
            )).resolve()
            if not configured.is_dir():
                raise FileNotFoundError(
                    "SIMENV_ROOT does not exist: " + str(configured)
                )
            return configured

        package_path = Path(
            rospkg.RosPack().get_path("danger_target_manager")
        ).resolve()

        # Source layout: <workspace>/src/danger_target_manager.
        if package_path.parent.name == "src":
            return package_path.parent.parent

        # Installed/devel layout: <workspace>/{install,devel}/share/package.
        prefix = package_path.parent.parent
        if (
            package_path.parent.name == "share"
            and prefix.name in ("install", "devel")
        ):
            return prefix.parent

        raise RuntimeError(
            "Cannot infer catkin workspace from package path: "
            + str(package_path)
            + ". Set ~workspace_root or SIMENV_ROOT explicitly."
        )

    def _candidate_scene_info_path(self) -> Optional[Path]:
        if self._team_scene_info:
            return Path(os.path.expandvars(
                os.path.expanduser(self._team_scene_info)
            )).resolve()

        environment_path = os.environ.get("TEAM_SCENE_INFO", "").strip()
        if environment_path:
            return Path(os.path.expandvars(
                os.path.expanduser(environment_path)
            )).resolve()

        default_path = (
            self._workspace_root
            / "generated_building"
            / "team_scene_info.json"
        )
        return default_path if default_path.is_file() else None

    def _result_path_from_scene_info(self, scene_path: Path) -> Path:
        if not scene_path.is_file():
            raise FileNotFoundError(
                "team_scene_info.json does not exist: " + str(scene_path)
            )

        with scene_path.open("r", encoding="utf-8") as stream:
            document = json.load(stream)

        try:
            configured_path = document["allowed_interfaces"]["result_file"]
        except (KeyError, TypeError) as error:
            raise ValueError(
                "team_scene_info.json is missing "
                "allowed_interfaces.result_file"
            ) from error

        if not isinstance(configured_path, str) or not configured_path.strip():
            raise ValueError(
                "allowed_interfaces.result_file must be a non-empty string"
            )

        path = Path(os.path.expandvars(
            os.path.expanduser(configured_path.strip())
        ))
        if not path.is_absolute():
            path = self._workspace_root / path
        return path.resolve()

    def _resolve_result_path(self) -> Path:
        # Alternative-frame diagnostics always use an explicitly supplied path;
        # scene metadata is ignored in this mode.
        if self._expected_frame != "world":
            path = Path(os.path.expandvars(
                os.path.expanduser(self._default_result_file)
            ))
            return path.resolve()

        scene_path = self._candidate_scene_info_path()
        if scene_path is not None:
            return self._result_path_from_scene_info(scene_path)

        if self._default_result_file:
            path = Path(os.path.expandvars(
                os.path.expanduser(self._default_result_file)
            ))
            if not path.is_absolute():
                path = self._workspace_root / path
            return path.resolve()

        return (
            self._workspace_root / "results" / "detected_danger.json"
        ).resolve()

    def _resolve_runtime_result_path(self) -> Path:
        configured = self._runtime_result_file or os.environ.get(
            "ROS1_RUNTIME_RESULT_FILE", ""
        ).strip()
        if not configured:
            runtime_root = os.environ.get(
                "ROS1_RUNTIME_ROOT", "/tmp/ros1_runtime"
            ).strip()
            configured = str(Path(runtime_root) / "detected_danger.json")

        path = Path(os.path.expandvars(os.path.expanduser(configured)))
        if not path.is_absolute():
            raise ValueError("~runtime_result_file must be an absolute path")
        return path.resolve()

    def _protect_default_path_during_non_world_test(self) -> None:
        if self._expected_frame == "world":
            return

        if not self._default_result_file:
            raise ValueError(
                "A non-world bag test requires an explicit "
                "~default_result_file"
            )

        requested = Path(os.path.expandvars(
            os.path.expanduser(self._default_result_file)
        ))
        if not requested.is_absolute():
            raise ValueError(
                "The non-world test output path must be absolute"
            )

        default_world_result = (
            self._workspace_root / "results" / "detected_danger.json"
        ).resolve()
        if self._result_path == default_world_result:
            raise ValueError(
                "An alternative-frame diagnostic must not overwrite the "
                "default world-frame result"
            )

    @staticmethod
    def _position_from_message(msg: ConfirmedDanger) -> Optional[Position]:
        position = (
            float(msg.position.x),
            float(msg.position.y),
            float(msg.position.z),
        )
        return position if all(math.isfinite(v) for v in position) else None

    @staticmethod
    def _distance(a: Position, b: Position) -> float:
        return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))

    def _matching_track(self, position: Position) -> Optional[int]:
        if self._spatial_dedup_radius <= 0.0:
            return None
        for track_id, (stored_position, _) in self._targets.items():
            if self._distance(position, stored_position) <= self._spatial_dedup_radius:
                return track_id
        return None

    def _confirmed_callback(self, msg: ConfirmedDanger) -> None:
        frame = self._normalize_frame(msg.header.frame_id)
        if frame != self._expected_frame:
            rospy.logerr_throttle(
                2.0,
                "Rejecting ConfirmedDanger in frame '%s'; expected '%s'",
                frame,
                self._expected_frame,
            )
            return

        if msg.header.stamp == rospy.Time(0):
            rospy.logwarn_throttle(
                2.0, "Rejecting ConfirmedDanger with a zero timestamp"
            )
            return

        position = self._position_from_message(msg)
        confidence = float(msg.confidence)
        if position is None or not math.isfinite(confidence):
            rospy.logwarn_throttle(
                2.0, "Rejecting non-finite ConfirmedDanger"
            )
            return

        track_id = int(msg.track_id)
        with self._lock:
            destination_id = track_id
            if destination_id not in self._targets:
                matched_id = self._matching_track(position)
                if matched_id is not None:
                    destination_id = matched_id

            previous = self._targets.get(destination_id)
            # Prefer the highest-confidence estimate for a spatially duplicated
            # track.  An update of the same track id remains idempotent.
            if previous is None or confidence >= previous[1] or destination_id == track_id:
                self._targets[destination_id] = (position, confidence)

            self._write_result_file_locked()

    def _elapsed_seconds_locked(self) -> float:
        if self._fixed_exploration_time >= 0.0:
            return self._fixed_exploration_time
        if self._start_time is None:
            return 0.0
        stop_time = self._end_time or rospy.Time.now()
        return max(0.0, (stop_time - self._start_time).to_sec())

    def _payload_locked(self) -> dict:
        ordered_targets = [
            self._targets[track_id][0]
            for track_id in sorted(self._targets)
        ]
        return {
            "exploration_time": self._elapsed_seconds_locked(),
            "detected_danger_sources": [
                {"position": [x, y, z]}
                for x, y, z in ordered_targets
            ],
        }

    @staticmethod
    def _write_json_atomic(path: Path, payload: dict) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=path.name + ".",
            suffix=".tmp",
            dir=str(path.parent),
            text=True,
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump(payload, stream, ensure_ascii=False, indent=2)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, path)
        except Exception:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise

    def _write_result_file_locked(self, publish_public: bool = False) -> None:
        payload = self._payload_locked()
        self._write_json_atomic(self._runtime_result_path, payload)
        if publish_public and self._result_path != self._runtime_result_path:
            self._write_json_atomic(self._result_path, payload)

    def _write_result_file(self, publish_public: bool = False) -> None:
        with self._lock:
            self._write_result_file_locked(publish_public=publish_public)

    def _handle_reset(self, _request) -> TriggerResponse:
        with self._lock:
            self._targets.clear()
            self._start_time = None
            self._end_time = None
            self._write_result_file_locked()
        return TriggerResponse(success=True, message="result writer reset")

    def _handle_start(self, _request) -> TriggerResponse:
        now = rospy.Time.now()
        if now == rospy.Time(0):
            return TriggerResponse(
                success=False,
                message="ROS time is zero; wait for /clock before starting",
            )
        with self._lock:
            self._start_time = now
            self._end_time = None
            self._write_result_file_locked()
        return TriggerResponse(success=True, message="exploration timer started")

    def _handle_finalize(self, _request) -> TriggerResponse:
        with self._lock:
            if (
                self._fixed_exploration_time < 0.0
                and self._start_time is None
            ):
                return TriggerResponse(
                    success=False,
                    message="start service has not been called",
                )
            self._end_time = rospy.Time.now()
            self._write_result_file_locked()
            count = len(self._targets)
        return TriggerResponse(
            success=True,
            message="finalized {} confirmed target(s)".format(count),
        )


def main() -> None:
    rospy.init_node("danger_result_writer")
    try:
        ResultWriterNode()
    except Exception as error:
        rospy.logfatal("Cannot start result writer: %s", error)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
