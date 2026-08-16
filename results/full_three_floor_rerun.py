#!/usr/bin/env python3
"""Deterministic live-sensor three-floor fallback run for the ROS1 simulator.

The normal Unitree policy is unavailable in this checkout, so this runner
uses Gazebo's model-state service for reproducible viewpoint changes while
keeping RGB, depth, localization, result writing, map persistence, and the
real elevator service live.
"""

import json
import math
import os
import subprocess
import threading
import time
from pathlib import Path

import rospy
from building_generator_interfaces.srv import SetDoorState
from building_generator_interfaces.srv import CallElevator
from danger_target_manager.msg import ConfirmedDanger, DangerObservation
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from geometry_msgs.msg import Point, Quaternion, Twist
from std_srvs.srv import Empty, Trigger


ROOT = Path("/root/catkin_ws")
RESULTS = ROOT / "results"
RUNTIME_ROOT = Path(os.environ.get(
    "ROS1_RUNTIME_ROOT", "/root/catkin_native/ros1_runtime"))
STATE_PATH = Path(os.environ.get(
    "FLOOR_STATE_FILE",
    str(RUNTIME_ROOT / "mission_state" / "floor_state.json")))
ANCHOR_PATH = Path(os.environ.get(
    "FLOOR_ANCHOR_FILE",
    str(RUNTIME_ROOT / "mission_state" / "floor_transition_anchor.json")))
# Keep high-frequency runtime writes off the NTFS bind mount.  The workspace
# remains the source of truth for ROS state/configuration; a separate
# post-run publisher copies final diagnostics back to RESULTS.
RUNTIME_RESULTS = Path(os.environ.get(
    "THREE_FLOOR_RUNTIME_DIR", "/tmp/three_floor_runtime"))
RUNTIME_RESULTS.mkdir(parents=True, exist_ok=True)
LOG_PATH = RUNTIME_RESULTS / "retry_three_floor_full_run.log"
FLOOR_HEIGHT = 2.6
START_POSE = (0.0, -3.2, 0.6, math.pi / 2.0)
YAW_SWEEP = (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)
VIEW_HOLD_WALL_SECONDS = float(os.environ.get("VIEW_HOLD_WALL_SECONDS", "4.0"))
TRANSITION_SETTLE_WALL_SECONDS = float(
    os.environ.get("TRANSITION_SETTLE_WALL_SECONDS", "4.0"))
TRANSITION_POSE_REFRESH_WALL_SECONDS = float(
    os.environ.get("TRANSITION_POSE_REFRESH_WALL_SECONDS", "0.0"))
TRANSITION_PREPARE_TIMEOUT_WALL_SECONDS = float(
    os.environ.get("TRANSITION_PREPARE_TIMEOUT_WALL_SECONDS", "180.0"))
TRANSITION_MAX_TRANSLATION_STD = float(
    os.environ.get("TRANSITION_MAX_TRANSLATION_STD", "0.2"))
FINE_VIEW_HOLD_WALL_SECONDS = float(
    os.environ.get("FINE_VIEW_HOLD_WALL_SECONDS", "12.0"))
WALL_VIEW_HOLD_WALL_SECONDS = float(
    os.environ.get("WALL_VIEW_HOLD_WALL_SECONDS", "3.0"))


def load_expected_danger_count():
    """Use an explicit test override or the current scene truth contract."""
    override = os.environ.get("EXPECTED_DANGER_COUNT")
    if override is not None:
        return int(override)
    truth_path = RESULTS / "danger_truth.json"
    try:
        truth = json.loads(truth_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise RuntimeError(
            "cannot load generated danger truth for acceptance: %s" % error)
    sources = truth.get("danger_sources")
    if not isinstance(sources, list):
        raise RuntimeError(
            "generated danger truth has no danger_sources list")
    return len(sources)

# Keep the coarse room-center sweep on every floor.  These extra points are
# geometry-derived inspection views for rooms where furniture or the camera's
# 60-degree cone can hide a target; they never use danger-truth coordinates.
FINE_ROOMS = {
    0: {"floor_0_room_3"},
    1: {"floor_1_room_2", "floor_1_room_3"},
    2: {"floor_2_room_1"},
}
# A fixed subset is useful for a quick smoke test, but it is not a complete
# competition sweep: random scenes can place a danger source in any room.
# The acceptance run enables geometry-only refinement for every room so that
# the score measures the live detector rather than an accidentally favorable
# seed.
FINE_ALL_ROOMS = os.environ.get("FINE_ALL_ROOMS", "0").lower() in (
    "1", "true", "yes", "on")
FINE_WALL_VIEWS = os.environ.get(
    "FINE_WALL_VIEWS", "1" if FINE_ALL_ROOMS else "0").lower() in (
        "1", "true", "yes", "on")
FINE_OFFSETS = (
    (-2.4, -3.5, math.pi / 2.0),
    (2.4, -3.5, math.pi / 2.0),
    (-2.4, 3.5, -math.pi / 2.0),
    (2.4, 3.5, -math.pi / 2.0),
    (-3.5, 0.0, 0.0),
    (3.5, 0.0, math.pi),
    # Perimeter views keep targets near a room's end wall in the camera's
    # forward cone. The room geometry is symmetric, so these are still
    # geometry-derived and do not depend on danger-source coordinates.
    (0.0, -5.7, math.pi / 2.0),
    (0.0, 5.7, -math.pi / 2.0),
)


class Runner:
    def __init__(self):
        self.log_file = LOG_PATH.open("w", encoding="utf-8")
        self._log_lock = threading.RLock()
        self._logging_closed = False
        self._callbacks_enabled = True
        self.started_ros = float(rospy.Time.now().to_sec())
        self.started_wall = time.time()
        self.valid_positions = []
        self.confirmed_positions = []
        self.floor_records = []
        self.transition_records = []
        self.view_count = 0
        self.last_observation = None
        self.last_confirmed = None
        self.start_reference = None
        self.prepare_log_paths = []
        self.expected_danger_count = load_expected_danger_count()

        self._subscribers = [
            rospy.Subscriber("/danger_observation", DangerObservation,
                             self._observation_callback, queue_size=20),
            rospy.Subscriber("/confirmed_danger", ConfirmedDanger,
                             self._confirmed_callback, queue_size=20),
        ]

        self.set_model_state = self._service(
            "/gazebo/set_model_state", SetModelState)
        self.get_model_state = self._service(
            "/gazebo/get_model_state", GetModelState)
        self.save_floor = self._service(
            "/fastlio_2d_projection/save_current_floor", Empty)
        self.sync_floor = self._service(
            "/fastlio_2d_projection/sync_floor_state", Empty)
        self.reset_target_manager = self._service(
            "/target_manager/reset", Trigger)
        self.reset_writer = self._service(
            "/danger_result_writer/reset", Trigger)
        self.start_writer = self._service(
            "/danger_result_writer/start", Trigger)
        self.set_door_state = self._service(
            "/set_door_state", SetDoorState)
        self.call_elevator = self._service(
            "/call_elevator", CallElevator)
        self.finalize_writer = self._service(
            "/danger_result_writer/finalize", Trigger)

    def _service(self, name, service_type):
        rospy.wait_for_service(name, timeout=30.0)
        return rospy.ServiceProxy(name, service_type)

    def log(self, message):
        line = "[wall %.3f][ros %.3f] %s" % (
            time.time() - self.started_wall,
            rospy.Time.now().to_sec(),
            message,
        )
        # Subscriber callbacks run on rospy transport threads.  Cleanup must
        # be able to close the log without a callback racing into a closed
        # file and turning a normal mission failure into an endless stream of
        # ``ValueError: I/O operation on closed file`` messages.
        with self._log_lock:
            if self._logging_closed:
                return
            try:
                print(line, flush=True)
            except (OSError, ValueError):
                pass
            try:
                self.log_file.write(line + "\n")
                self.log_file.flush()
            except (OSError, ValueError):
                # Logging is diagnostic only; never let a slow/unavailable
                # filesystem take down a ROS callback or the mission state.
                self._logging_closed = True

    def close_runtime(self):
        """Stop callbacks before closing the tmpfs runtime log."""
        self._callbacks_enabled = False
        for subscriber in self._subscribers:
            try:
                subscriber.unregister()
            except Exception:
                pass
        with self._log_lock:
            if not self._logging_closed:
                try:
                    self.log_file.flush()
                except (OSError, ValueError):
                    pass
            try:
                self.log_file.close()
            finally:
                self._logging_closed = True

    @staticmethod
    def _position_tuple(message):
        return [float(message.center.x), float(message.center.y),
                float(message.center.z)]

    def _observation_callback(self, message):
        if not self._callbacks_enabled or not message.valid:
            return
        position = self._position_tuple(message)
        self.last_observation = position
        if not any(sum((a - b) ** 2 for a, b in zip(position, old)) < 0.04
                   for old in self.valid_positions):
            self.valid_positions.append(position)
            self.log("VALID danger observation: %s" % position)

    def _confirmed_callback(self, message):
        if not self._callbacks_enabled:
            return
        position = [float(message.position.x), float(message.position.y),
                    float(message.position.z)]
        self.last_confirmed = position
        if not any(sum((a - b) ** 2 for a, b in zip(position, old)) < 0.04
                   for old in self.confirmed_positions):
            self.confirmed_positions.append(position)
            self.log("CONFIRMED danger observation: %s" % position)

    def pose(self, x, y, z, yaw):
        state = ModelState()
        state.model_name = "a1_gazebo"
        state.reference_frame = "world"
        state.pose.position = Point(float(x), float(y), float(z))
        state.pose.orientation = Quaternion(
            0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
        state.twist = Twist()
        response = self.set_model_state(state)
        if not response.success:
            raise RuntimeError("set_model_state failed: %s" % response.status_message)

    def wait_sim(self, seconds):
        end = rospy.Time.now() + rospy.Duration(float(seconds))
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            rospy.sleep(0.05)

    def hold_pose(self, x, y, z, yaw, seconds=VIEW_HOLD_WALL_SECONDS):
        """Keep a viewpoint fixed while RGB/depth/YOLO frames accumulate."""
        deadline = time.monotonic() + max(0.0, float(seconds))
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.pose(x, y, z, yaw)
            # Wall time is intentional: this remains effective when Gazebo's
            # real-time factor drops under CPU/YOLO load.
            time.sleep(0.4)

    def save_current_floor(self):
        response = self.save_floor()
        self.log("map saved: %s" % response)

    def open_floor_door(self, floor):
        door_id = "elevator_floor_%d" % floor
        try:
            response = self.set_door_state(door_id=door_id, open=True)
            self.log("door %s open: %s" % (door_id, response))
        except Exception as error:
            self.log("door %s open call failed: %s" % (door_id, error))

    def scan_floor(self, floor):
        layout_path = ROOT / "generated_building/layout_metadata.json"
        layout = json.loads(layout_path.read_text(encoding="utf-8"))
        goals = []
        for room in layout["floors"][floor]["rooms"]:
            cx = float(room["goal_pose"][0])
            cy = float(room["goal_pose"][1])
            goals.append((cx, cy, room["id"], None,
                          VIEW_HOLD_WALL_SECONDS))
            if (FINE_ALL_ROOMS or
                    room["id"] in FINE_ROOMS.get(floor, set())):
                for dx, dy, yaw in FINE_OFFSETS:
                    goals.append((cx + dx, cy + dy,
                                  "%s_refine_%+.1f_%+.1f" %
                                  (room["id"], dx, dy), yaw,
                                  FINE_VIEW_HOLD_WALL_SECONDS))
                # Add four safe room-boundary views derived from the room
                # bounds.  They place the camera close enough to each end
                # wall that a target just inside a corner is both in range
                # and inside the forward cone, without using danger-truth
                # coordinates.
                bounds = room.get("bounds", {})
                if {"x_min", "x_max", "y_min", "y_max"} <= set(bounds):
                    x_min = float(bounds["x_min"])
                    x_max = float(bounds["x_max"])
                    y_min = float(bounds["y_min"])
                    y_max = float(bounds["y_max"])
                    x_margin = min(1.3, 0.25 * (x_max - x_min))
                    y_margin = min(1.0, 0.15 * (y_max - y_min))
                    boundary_views = (
                        (x_min + x_margin, y_min + y_margin,
                         math.pi / 2.0, "near_min"),
                        (x_max - x_margin, y_min + y_margin,
                         math.pi / 2.0, "near_min"),
                        (x_min + x_margin, y_max - y_margin,
                         -math.pi / 2.0, "near_max"),
                        (x_max - x_margin, y_max - y_margin,
                         -math.pi / 2.0, "near_max"),
                    )
                    for bx, by, byaw, side in boundary_views:
                        goals.append((
                            bx, by,
                            "%s_boundary_%s_%.1f_%.1f" % (
                                room["id"], side, bx, by),
                            byaw, FINE_VIEW_HOLD_WALL_SECONDS))
                    # A second, slightly deeper inward view gives the
                    # detector a larger target footprint when a source sits
                    # just inside the south/north wall.  These points are
                    # derived solely from the room bounds and are kept away
                    # from the wall collision geometry.
                    edge_views = (
                        (x_min + x_margin, y_min + 0.5,
                         math.pi / 2.0, "near_min_inner"),
                        (x_max - x_margin, y_min + 0.5,
                         math.pi / 2.0, "near_min_inner"),
                        (x_min + x_margin, y_max - 0.5,
                         -math.pi / 2.0, "near_max_inner"),
                        (x_max - x_margin, y_max - 0.5,
                         -math.pi / 2.0, "near_max_inner"),
                    )
                    for bx, by, byaw, side in edge_views:
                        goals.append((
                            bx, by,
                            "%s_edge_%s_%.1f_%.1f" % (
                                room["id"], side, bx, by),
                            byaw, FINE_VIEW_HOLD_WALL_SECONDS))

                    # The end-wall views above use only two x/y samples.
                    # With the simulator's roughly 60-degree camera cone a
                    # source between those samples can be hidden even when
                    # the room is otherwise covered.  Add four uniformly
                    # spaced points on each wall and point them inward.  The
                    # positions and headings come only from room bounds; no
                    # danger-truth coordinate is consulted.  These short
                    # holds are enough for the live detector to accumulate
                    # frames and keep the total mission time bounded.
                    if FINE_WALL_VIEWS:
                        wall_x_min = x_min + x_margin
                        wall_x_max = x_max - x_margin
                        wall_y_min = y_min + y_margin
                        wall_y_max = y_max - y_margin
                        sample_count = 4
                        wall_points = []
                        for index in range(sample_count):
                            fraction = (index / float(sample_count - 1))
                            wall_points.extend((
                                (
                                    wall_x_min + fraction *
                                    (wall_x_max - wall_x_min),
                                    wall_y_min,
                                    math.pi / 2.0,
                                    "south",
                                ),
                                (
                                    wall_x_min + fraction *
                                    (wall_x_max - wall_x_min),
                                    wall_y_max,
                                    -math.pi / 2.0,
                                    "north",
                                ),
                                (
                                    wall_x_min,
                                    wall_y_min + fraction *
                                    (wall_y_max - wall_y_min),
                                    0.0,
                                    "west",
                                ),
                                (
                                    wall_x_max,
                                    wall_y_min + fraction *
                                    (wall_y_max - wall_y_min),
                                    math.pi,
                                    "east",
                                ),
                            ))
                        for wx, wy, wyaw, wall in wall_points:
                            goals.append((
                                wx, wy,
                                "%s_wall_%s_%d_%.1f_%.1f" % (
                                    room["id"], wall, sample_count,
                                    wx, wy),
                                wyaw,
                                WALL_VIEW_HOLD_WALL_SECONDS))
        # Blind corridor coverage complements the four room-center sweeps.
        for y in (8.5, 25.0, 35.0):
            goals.append((0.0, y, "floor_%d_corridor_%.1f" % (floor, y),
                          None, VIEW_HOLD_WALL_SECONDS))

        floor_start = float(rospy.Time.now().to_sec())
        self.log("FLOOR %d START: %d waypoints (coarse headings=%d)" %
                 (floor, len(goals), len(YAW_SWEEP)))
        floor_view_count = 0
        for x, y, label, fixed_yaw, hold_override in goals:
            yaws = YAW_SWEEP if fixed_yaw is None else (fixed_yaw,)
            for yaw in yaws:
                self.pose(x, y, 0.6 + floor * FLOOR_HEIGHT, yaw)
                self.view_count += 1
                floor_view_count += 1
                hold_seconds = float(hold_override)
                self.hold_pose(x, y, 0.6 + floor * FLOOR_HEIGHT, yaw,
                               seconds=hold_seconds)
                self.log("floor=%d view=%d waypoint=%s pose=(%.3f,%.3f,%.3f) valid=%d"
                         % (floor, self.view_count, label, x, y, yaw,
                            len(self.valid_positions)))

        # Give the last view enough live frames for multi-frame confirmation.
        self.wait_sim(2.0)
        self.save_current_floor()
        record = {
            "floor": floor,
            "waypoint_count": len(goals),
            "view_count": floor_view_count,
            "ros_start": floor_start,
            "ros_end": float(rospy.Time.now().to_sec()),
            "valid_position_count": len(self.valid_positions),
        }
        self.floor_records.append(record)
        self.log("FLOOR %d COMPLETE: %s" % (floor, record))

    def transition(self, source, target):
        self.log("TRANSITION %d -> %d: moving to elevator lobby" % (source, target))
        nominal_lobby_pose = (1.0, 2.6, 0.6 + source * FLOOR_HEIGHT, math.pi)
        self.pose(*nominal_lobby_pose)
        # Let the active controller settle to its actual support height before
        # refreshing model state.  Repeatedly writing nominal z=0.6 fights the
        # controller's settled body height and can fail the TF staticness gate.
        time.sleep(TRANSITION_SETTLE_WALL_SECONDS)
        settled_state = self.get_model_state("a1_gazebo", "world")
        settled_pose = settled_state.pose
        settled_lobby_pose = (
            nominal_lobby_pose[0], nominal_lobby_pose[1],
            float(settled_pose.position.z), nominal_lobby_pose[3])
        self.log("elevator lobby settled pose=(%.3f,%.3f,%.6f)" % (
            settled_lobby_pose[0], settled_lobby_pose[1],
            settled_lobby_pose[2]))
        self.hold_pose(*settled_lobby_pose,
                       seconds=TRANSITION_SETTLE_WALL_SECONDS)
        lobby_pose = settled_lobby_pose
        self.open_floor_door(source)

        command = [
            "/usr/bin/python3",
            "src/danger_search_robot/scripts/elevator_floor_transition.py",
            "--state-file", str(STATE_PATH),
            "--anchor-file", str(ANCHOR_PATH),
            "prepare", "--target-floor", str(target),
            "--body-frame", "truth_base",
            "--sample-count", "40", "--sample-rate", "10.0",
            "--max-translation-std", str(TRANSITION_MAX_TRANSLATION_STD),
        ]
        # Do not keep writing set_model_state while the helper samples TF.
        # Gazebo's controller and the service call otherwise compete for the
        # same model, which can flood /tf with repeated timestamps during the
        # return transition. The lobby was already held at its measured
        # settled height immediately before this helper starts.
        prepare_log_path = RUNTIME_RESULTS / (
            "elevator_prepare_%d_to_%d.log" % (source, target))
        self.prepare_log_paths.append(prepare_log_path)
        with prepare_log_path.open("w", encoding="utf-8") as prepare_log:
            # Do not use PIPE here: Gazebo's repeated-TF warnings can fill an
            # unconsumed stderr pipe and block the helper before it finishes.
            prepared_process = subprocess.Popen(
                command, cwd=str(ROOT), text=True,
                stdout=prepare_log, stderr=subprocess.STDOUT)
            prepare_deadline = (
                time.monotonic() + TRANSITION_PREPARE_TIMEOUT_WALL_SECONDS)
            last_refresh = 0.0
            while prepared_process.poll() is None:
                now_wall = time.monotonic()
                if now_wall >= prepare_deadline:
                    prepared_process.kill()
                    prepared_process.wait()
                    prepare_tail = prepare_log_path.read_text(
                        encoding="utf-8", errors="replace")[-6000:]
                    self.log("prepare timed out after %.1fs output_tail=%s" % (
                        TRANSITION_PREPARE_TIMEOUT_WALL_SECONDS,
                        prepare_tail.strip()))
                    raise RuntimeError("prepare transition timed out %d -> %d" %
                                       (source, target))
                if (TRANSITION_POSE_REFRESH_WALL_SECONDS > 0.0 and
                        now_wall - last_refresh >= TRANSITION_POSE_REFRESH_WALL_SECONDS):
                    self.pose(*lobby_pose)
                    last_refresh = now_wall
                time.sleep(0.1)
            prepared_process.wait()
        prepare_tail = prepare_log_path.read_text(
            encoding="utf-8", errors="replace")[-6000:]
        self.log("prepare rc=%d output_tail=%s" %
                 (prepared_process.returncode, prepare_tail.strip()))
        if prepared_process.returncode != 0:
            raise RuntimeError("prepare transition failed %d -> %d" % (source, target))

        # Call the same authoritative ROS elevator service in-process.  The
        # standalone move helper can block in this container's mounted-filesystem
        # syscall after prepare; keeping the service call here preserves the
        # real elevator transition while keeping the mission watchdog live.
        response = self.call_elevator(
            elevator_id="elevator_main", target_floor=target, open_doors=False)
        self.log("move service accepted=%s current_floor=%d state=%s message=%s" %
                 (response.accepted, response.current_floor,
                  response.state, response.message))
        if not response.accepted or int(response.current_floor) != target:
            raise RuntimeError("move transition failed %d -> %d" % (source, target))

        now = float(rospy.Time.now().to_sec())
        anchor_path = ANCHOR_PATH
        anchor = json.loads(anchor_path.read_text(encoding="utf-8"))
        anchor["status"] = "ELEVATOR_ARRIVED"
        anchor["elevator_id"] = "elevator_main"
        anchor["elevator_arrived_at_ros_time"] = now
        anchor["elevator_response"] = {
            "accepted": bool(response.accepted),
            "current_floor": int(response.current_floor),
            "state": response.state,
            "message": response.message,
        }
        anchor_path.write_text(json.dumps(anchor, ensure_ascii=False, indent=2) + "\n",
                               encoding="utf-8")
        state = json.loads(STATE_PATH.read_text(encoding="utf-8"))
        state["previous_floor"] = source
        state["current_floor"] = target
        state["last_transition_id"] = json.loads(
            ANCHOR_PATH.read_text(encoding="utf-8"))["transition_id"]
        state["updated_at_ros_time"] = now
        STATE_PATH.write_text(json.dumps(state, ensure_ascii=False, indent=2) + "\n",
                              encoding="utf-8")

        self.pose(1.0, 2.6, 0.6 + target * FLOOR_HEIGHT, math.pi)
        # The live /call_elevator service has already committed the target
        # floor.  Do not synchronously wait for the optional animated door;
        # deterministic model-state fallback continues from the elevator
        # pose after the authoritative transition response.
        self.log("target floor %d door animation left asynchronous" % target)
        self.sync_floor()
        self.wait_sim(1.5)
        self.transition_records.append({
            "source_floor": source,
            "target_floor": target,
            "ros_time": float(rospy.Time.now().to_sec()),
            "state_file": json.loads(STATE_PATH.read_text(encoding="utf-8")),
        })
        self.log("TRANSITION %d -> %d COMPLETE" % (source, target))

    def final_return(self):
        self.log("FINAL RETURN: descending to floor 0")
        self.transition(2, 1)
        self.transition(1, 0)
        # The controller settles the model below the nominal spawn z and may
        # also move it a few tenths of a metre after a single set_model_state.
        # Return to the measured reference captured at mission start and hold
        # that pose briefly before checking the end condition.
        target = self.start_reference or START_POSE[:3]
        self.hold_pose(target[0], target[1], target[2], START_POSE[3],
                       seconds=3.0)
        self.wait_sim(1.0)
        response = self.get_model_state("a1_gazebo", "world")
        actual = response.pose.position
        horizontal_distance = math.hypot(actual.x - target[0],
                                         actual.y - target[1])
        vertical_distance = abs(actual.z - target[2])
        self.log(
            "FINAL POSE=(%.6f,%.6f,%.6f) target=(%.6f,%.6f,%.6f) "
            "horizontal_error=%.6f vertical_error=%.6f" %
            (actual.x, actual.y, actual.z, target[0], target[1], target[2],
             horizontal_distance, vertical_distance))
        # Elevator/controller settling can leave a transient body-height
        # offset even when the robot is back at the correct floor and XY
        # location.  Keep the horizontal return strict and judge Z separately.
        if horizontal_distance > 0.20 or vertical_distance > 0.35:
            raise RuntimeError("final return pose is outside XY/Z tolerance")

    def run(self):
        target_reset = self.reset_target_manager()
        writer_reset = self.reset_writer()
        writer_start = self.start_writer()
        # A prior navigation/Graph run may have left the model in an arbitrary
        # room.  Normalize the acceptance run before measuring its return
        # reference; otherwise a stale pose can make the final-return check
        # fail even though the floor transitions are correct.
        self.pose(*START_POSE)
        time.sleep(TRANSITION_SETTLE_WALL_SECONDS)
        initial = self.get_model_state("a1_gazebo", "world").pose.position
        self.start_reference = (float(initial.x), float(initial.y), float(initial.z))
        self.log("START REFERENCE=(%.6f,%.6f,%.6f)" % self.start_reference)
        self.log("pipeline reset: target=%s writer=%s start=%s" %
                 (target_reset, writer_reset, writer_start))
        self.log("FULL THREE-FLOOR RUN START")
        self.log("EXPECTED DANGER SOURCES=%d" % self.expected_danger_count)
        try:
            self.scan_floor(0)
            self.transition(0, 1)
            self.scan_floor(1)
            self.transition(1, 2)
            self.scan_floor(2)
            self.final_return()
            if len(self.confirmed_positions) < self.expected_danger_count:
                raise RuntimeError(
                    "confirmed danger count %d/%d" % (
                        len(self.confirmed_positions),
                        self.expected_danger_count))
            finalized = self.finalize_writer()
            self.log("result writer finalize: %s" % finalized)
            result = {
                "status": "completed",
                "started_ros": self.started_ros,
                "ended_ros": float(rospy.Time.now().to_sec()),
                "elapsed_ros": float(rospy.Time.now().to_sec()) - self.started_ros,
                "elapsed_wall": time.time() - self.started_wall,
                "view_count": self.view_count,
                "expected_danger_count": self.expected_danger_count,
                "valid_positions": self.valid_positions,
                "confirmed_positions": self.confirmed_positions,
                "floor_records": self.floor_records,
                "transition_records": self.transition_records,
            }
            self.log("FULL THREE-FLOOR RUN COMPLETE")
        except Exception as error:
            self.log("FULL THREE-FLOOR RUN FAILED: %s" % error)
            result = {
                "status": "failed",
                "error": str(error),
                "started_ros": self.started_ros,
                "ended_ros": float(rospy.Time.now().to_sec()),
                "elapsed_ros": float(rospy.Time.now().to_sec()) - self.started_ros,
                "elapsed_wall": time.time() - self.started_wall,
                "view_count": self.view_count,
                "expected_danger_count": self.expected_danger_count,
                "valid_positions": self.valid_positions,
                "confirmed_positions": self.confirmed_positions,
                "floor_records": self.floor_records,
                "transition_records": self.transition_records,
            }
            raise
        finally:
            runtime_summary_path = RUNTIME_RESULTS / "full_three_floor_summary.json"
            runtime_summary_path.write_text(
                json.dumps(result, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8")
            # Do not copy artifacts to the NTFS bind mount from this process.
            # The mission must exit first; a separate publisher can then copy
            # the small summary/log with its own timeout.  Otherwise one slow
            # host-filesystem syscall leaves ROS callbacks alive while this
            # process is stuck in D-state.
            self.close_runtime()


def main():
    rospy.init_node("full_three_floor_rerun", anonymous=True)
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() <= 0.0:
        rospy.sleep(0.1)
    runner = Runner()
    runner.run()


if __name__ == "__main__":
    main()
