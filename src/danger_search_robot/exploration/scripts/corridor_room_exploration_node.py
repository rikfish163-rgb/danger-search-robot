#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import actionlib
import cv2
import numpy as np
import rospy
import tf2_ros

from actionlib_msgs.msg import GoalStatus
from danger_target_manager.msg import ConfirmedDanger
from geometry_msgs.msg import Point, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class AxisEstimate:
    x: float
    y: float
    yaw: float
    score: float
    forward_score: float
    reverse_score: float
    offset: float
    wall_ratio: float
    balance_error: float
    unknown_ratio: float


@dataclass
class Candidate:
    x: float
    y: float
    yaw: float
    kind: str
    score: float
    progress: float
    lateral: float
    zone_id: str = ""


class CorridorRoomExplorer:
    """
    Automatic corridor-first, room-second explorer.

    The corridor axis is estimated from the occupancy grid. It does not use
    the robot's startup yaw and does not require RViz clicks.

    State machine:
      AUTO_AXIS_ESTIMATION
        -> CORRIDOR_ADVANCE
        -> ROOM_SWEEP
        -> FINISHED
    """

    def __init__(self) -> None:
        rospy.init_node("corridor_room_explorer")

        # ROS interfaces
        self.map_topic = rospy.get_param("~map_topic", "/map_confirmed")
        self.base_frame = rospy.get_param("~base_frame", "body")
        self.action_name = rospy.get_param("~move_base_action", "/move_base")
        self.dry_run = bool(rospy.get_param("~dry_run", True))
        self.global_frame = rospy.get_param("~global_frame", "world")
        self.require_mapping_health = bool(
            rospy.get_param("~require_mapping_health", True)
        )
        self.mapping_health_topic = rospy.get_param(
            "~mapping_health_topic", "/mapping_healthy"
        )
        self.mapping_health_timeout = float(
            rospy.get_param("~mapping_health_timeout", 2.5)
        )
        self.map_max_age = float(
            rospy.get_param("~map_max_age", 1.5)
        )
        self.map_future_tolerance = float(
            rospy.get_param("~map_future_tolerance", 0.25)
        )

        # A real mission must not finish on an empty frontier list. These
        # gates are deliberately configurable because the same node is used
        # for smoke tests and for the four-room competition floor.
        self.required_room_zones = int(
            rospy.get_param("~required_room_zones", 4)
        )
        self.require_danger_confirmation = bool(
            rospy.get_param("~require_danger_confirmation", True)
        )
        self.minimum_confirmed_dangers = int(
            rospy.get_param("~minimum_confirmed_dangers", 1)
        )
        self.danger_topic = rospy.get_param(
            "~danger_topic", "/confirmed_danger"
        )

        # Map interpretation
        self.occupied_threshold = int(
            rospy.get_param("~occupied_threshold", 50)
        )
        self.min_goal_clearance = float(
            rospy.get_param("~min_goal_clearance", 0.38)
        )

        # Automatic corridor-axis estimation
        self.axis_angle_step_deg = float(
            rospy.get_param("~axis_angle_step_deg", 3.0)
        )
        self.axis_refine_step_deg = float(
            rospy.get_param("~axis_refine_step_deg", 0.75)
        )
        self.axis_refine_window_deg = float(
            rospy.get_param("~axis_refine_window_deg", 4.0)
        )
        self.axis_max_center_offset = float(
            rospy.get_param("~axis_max_center_offset", 1.50)
        )
        self.axis_center_offset_step = float(
            rospy.get_param("~axis_center_offset_step", 0.15)
        )
        self.axis_sample_length = float(
            rospy.get_param("~axis_sample_length", 9.0)
        )
        self.axis_sample_step = float(
            rospy.get_param("~axis_sample_step", 0.25)
        )
        self.axis_center_band_half_width = float(
            rospy.get_param("~axis_center_band_half_width", 0.35)
        )
        self.axis_center_band_step = float(
            rospy.get_param("~axis_center_band_step", 0.20)
        )
        self.axis_wall_search_min = float(
            rospy.get_param("~axis_wall_search_min", 0.65)
        )
        self.axis_wall_search_max = float(
            rospy.get_param("~axis_wall_search_max", 2.40)
        )
        self.axis_wall_search_step = float(
            rospy.get_param("~axis_wall_search_step", 0.10)
        )
        self.axis_wall_probe_start = float(
            rospy.get_param("~axis_wall_probe_start", 0.60)
        )
        self.axis_wall_probe_end = float(
            rospy.get_param("~axis_wall_probe_end", 5.50)
        )
        self.axis_wall_probe_step = float(
            rospy.get_param("~axis_wall_probe_step", 0.40)
        )
        self.axis_min_score = float(
            rospy.get_param("~axis_min_score", 6.0)
        )
        self.axis_min_wall_ratio = float(
            rospy.get_param("~axis_min_wall_ratio", 0.20)
        )
        self.axis_stability_required = int(
            rospy.get_param("~axis_stability_required", 5)
        )
        self.axis_stability_angle_deg = float(
            rospy.get_param("~axis_stability_angle_deg", 3.0)
        )
        self.axis_stability_center = float(
            rospy.get_param("~axis_stability_center", 0.30)
        )
        self.axis_reestimate_period = float(
            rospy.get_param("~axis_reestimate_period", 1.0)
        )

        # Axis scoring weights
        self.axis_weight_clear = float(
            rospy.get_param("~axis_weight_clear", 4.0)
        )
        self.axis_weight_unknown = float(
            rospy.get_param("~axis_weight_unknown", 5.0)
        )
        self.axis_weight_wall = float(
            rospy.get_param("~axis_weight_wall", 5.0)
        )
        self.axis_weight_continuity = float(
            rospy.get_param("~axis_weight_continuity", 3.0)
        )
        self.axis_weight_balance = float(
            rospy.get_param("~axis_weight_balance", 2.5)
        )
        self.axis_weight_blocked = float(
            rospy.get_param("~axis_weight_blocked", 7.0)
        )
        self.axis_direction_unknown_weight = float(
            rospy.get_param("~axis_direction_unknown_weight", 7.0)
        )
        self.axis_direction_clear_weight = float(
            rospy.get_param("~axis_direction_clear_weight", 2.0)
        )

        # Corridor exploration
        self.frontier_min_cells = int(
            rospy.get_param("~frontier_min_cells", 4)
        )
        self.frontier_goal_radius = float(
            rospy.get_param("~frontier_goal_radius", 1.20)
        )
        self.forward_margin = float(
            rospy.get_param("~forward_margin", 0.15)
        )
        self.backtrack_tolerance = float(
            rospy.get_param("~backtrack_tolerance", 0.25)
        )
        self.corridor_half_width = float(
            rospy.get_param("~corridor_half_width", 1.35)
        )
        self.corridor_center_weight = float(
            rospy.get_param("~corridor_center_weight", 3.5)
        )
        self.corridor_empty_cycles_to_finish = int(
            rospy.get_param("~corridor_empty_cycles_to_finish", 8)
        )
        self.minimum_corridor_progress = float(
            rospy.get_param("~minimum_corridor_progress", 3.0)
        )

        # Room sweep
        self.coverage_radius = float(
            rospy.get_param("~coverage_radius", 0.85)
        )
        self.room_lateral_min = float(
            rospy.get_param("~room_lateral_min", 1.20)
        )
        self.room_bin_length = float(
            rospy.get_param("~room_bin_length", 3.00)
        )
        self.room_progress_min = float(
            rospy.get_param("~room_progress_min", 0.60)
        )
        self.room_min_area = float(
            rospy.get_param("~room_min_area", 0.80)
        )
        self.room_empty_cycles_to_finish = int(
            rospy.get_param("~room_empty_cycles_to_finish", 5)
        )

        # Navigation
        self.min_goal_distance = float(
            rospy.get_param("~min_goal_distance", 0.55)
        )
        self.max_goal_distance = float(
            rospy.get_param("~max_goal_distance", 10.0)
        )
        self.goal_timeout = float(
            rospy.get_param("~goal_timeout", 75.0)
        )
        self.progress_timeout = float(
            rospy.get_param("~progress_timeout", 25.0)
        )
        self.progress_epsilon = float(
            rospy.get_param("~progress_epsilon", 0.20)
        )
        self.blacklist_radius = float(
            rospy.get_param("~blacklist_radius", 1.00)
        )
        self.plan_period = float(
            rospy.get_param("~plan_period", 1.0)
        )
        self.scan_pause = float(
            rospy.get_param("~scan_pause", 1.5)
        )
        self.require_make_plan = bool(
            rospy.get_param("~require_make_plan", True)
        )
        self.plan_service_name = rospy.get_param(
            "~plan_service", "/move_base/GlobalPlanner/make_plan"
        )
        self.plan_tolerance = float(
            rospy.get_param("~plan_tolerance", 0.25)
        )
        self.navigation_retry_cooldown = float(
            rospy.get_param("~navigation_retry_cooldown", 3.0)
        )
        self.action_server_timeout = float(
            rospy.get_param("~action_server_timeout", 45.0)
        )

        # TF and move_base
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.client = actionlib.SimpleActionClient(
            self.action_name,
            MoveBaseAction,
        )

        # Map state
        self.map_msg: Optional[OccupancyGrid] = None
        self.grid: Optional[np.ndarray] = None
        self.covered: Optional[np.ndarray] = None
        self.map_resolution = 0.0
        self.map_width = 0
        self.map_height = 0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_origin_yaw = 0.0
        self.last_map_received_wall: Optional[float] = None
        self.mapping_health_received = False
        self.mapping_health_ok = False
        self.mapping_health_received_wall: Optional[float] = None

        # Mission state
        self.phase = "AUTO_AXIS_ESTIMATION"
        self.mission_origin: Optional[Tuple[float, float, float]] = None
        self.axis_preview: Optional[AxisEstimate] = None
        self.axis_history: List[AxisEstimate] = []
        self.last_axis_estimate_time = rospy.Time(0)
        self.max_corridor_progress = 0.0
        self.last_mark_pose: Optional[Tuple[float, float]] = None

        self.corridor_empty_cycles = 0
        self.room_empty_cycles = 0
        self.finished = False

        # Navigation state
        self.current_goal: Optional[Candidate] = None
        self.current_goal_start = rospy.Time(0)
        self.last_goal_distance = float("inf")
        self.last_goal_progress_time = rospy.Time(0)
        self.last_selected: Optional[Candidate] = None

        # Room scan state
        self.scan_queue: List[float] = []
        self.scan_position: Optional[Tuple[float, float]] = None
        self.scan_zone_id = ""
        self.scan_wait_until = rospy.Time(0)
        self.navigation_blocked_until = rospy.Time(0)
        self.confirmed_danger_ids: Set[int] = set()

        self.completed_zones: Set[str] = set()
        self.failed_zones: Dict[str, int] = {}
        self.blacklist: List[Tuple[float, float]] = []

        # Publications
        self.status_pub = rospy.Publisher(
            "~status", String, queue_size=1, latch=True
        )
        self.axis_diagnostic_pub = rospy.Publisher(
            "~axis_diagnostic", String, queue_size=1, latch=True
        )
        self.finished_pub = rospy.Publisher(
            "~finished", Bool, queue_size=1, latch=True
        )
        self.marker_pub = rospy.Publisher(
            "~markers", MarkerArray, queue_size=1, latch=True
        )
        self.mapping_health_sub = rospy.Subscriber(
            self.mapping_health_topic,
            Bool,
            self.mapping_health_callback,
            queue_size=1,
        )
        self.danger_sub = rospy.Subscriber(
            self.danger_topic,
            ConfirmedDanger,
            self.danger_callback,
            queue_size=20,
        )

        rospy.Subscriber(
            self.map_topic,
            OccupancyGrid,
            self.map_callback,
            queue_size=1,
        )

        if not self.dry_run:
            rospy.loginfo(
                "[corridor_room_explorer] waiting for %s...",
                self.action_name,
            )
            if not self.wait_for_action_server():
                rospy.logfatal(
                    "[corridor_room_explorer] action server did not become "
                    "ready within %.1f wall seconds",
                    self.action_server_timeout,
                )
                raise rospy.ROSException("move_base action server unavailable")
            rospy.loginfo(
                "[corridor_room_explorer] move_base action available"
            )

        rospy.Timer(
            rospy.Duration(self.plan_period),
            self.timer_callback,
        )

        self.publish_status("WAITING_FOR_MAP")
        self.finished_pub.publish(Bool(data=False))
        rospy.loginfo(
            "[corridor_room_explorer] started dry_run=%s map=%s "
            "frame=%s base=%s automatic_axis=true",
            self.dry_run,
            self.map_topic,
            self.global_frame,
            self.base_frame,
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def angle_difference(a: float, b: float) -> float:
        return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

    def publish_status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))

    def wait_for_action_server(self) -> bool:
        """Wait using wall time; Gazebo time can run much faster than wall time."""
        deadline = time.monotonic() + self.action_server_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                if self.client.wait_for_server(rospy.Duration(0.5)):
                    return True
            except Exception as exc:
                rospy.logwarn_throttle(
                    3.0,
                    "[corridor_room_explorer] action handshake pending: %s",
                    str(exc),
                )
            time.sleep(0.05)
        return False

    def action_goal_in_flight(self) -> bool:
        if self.dry_run:
            return False
        return self.client.get_state() in (
            GoalStatus.PENDING,
            GoalStatus.ACTIVE,
            GoalStatus.PREEMPTING,
            GoalStatus.RECALLING,
        )

    def mapping_health_callback(self, message: Bool) -> None:
        self.mapping_health_received = True
        self.mapping_health_ok = bool(message.data)
        self.mapping_health_received_wall = time.monotonic()

    def danger_callback(self, message: ConfirmedDanger) -> None:
        # The target manager already performs temporal/spatial confirmation.
        # The explorer only needs a monotonic evidence count for its finish
        # gate; repeated publications of one track remain idempotent.
        self.confirmed_danger_ids.add(int(message.track_id))

    def mapping_health_ready(self) -> bool:
        if not self.require_mapping_health:
            return True
        if not self.mapping_health_received or not self.mapping_health_ok:
            return False
        if self.mapping_health_received_wall is None:
            return False
        return (
            time.monotonic() - self.mapping_health_received_wall
            <= self.mapping_health_timeout
        )

    def map_is_fresh(self, message: OccupancyGrid) -> bool:
        stamp = message.header.stamp
        now = rospy.Time.now()
        if stamp.is_zero() or now.is_zero():
            return False
        age = (now - stamp).to_sec()
        return (
            math.isfinite(age)
            and age <= self.map_max_age
            and age >= -self.map_future_tolerance
        )

    def map_callback(self, msg: OccupancyGrid) -> None:
        expected_frame = self.global_frame.strip().lstrip("/")
        actual_frame = msg.header.frame_id.strip().lstrip("/")
        if expected_frame and actual_frame != expected_frame:
            rospy.logwarn_throttle(
                3.0,
                "[corridor_room_explorer] rejecting map frame=%s expected=%s",
                actual_frame or "<empty>",
                expected_frame,
            )
            return
        if not self.map_is_fresh(msg):
            rospy.logwarn_throttle(
                3.0,
                "[corridor_room_explorer] rejecting stale/future map stamp",
            )
            return

        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)

        if width <= 0 or height <= 0 or resolution <= 0.0:
            return

        data = np.asarray(msg.data, dtype=np.int16)
        if data.size != width * height:
            return

        grid = data.reshape((height, width))

        q = msg.info.origin.orientation
        _, _, origin_yaw = euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

        geometry_changed = (
            self.covered is None
            or width != self.map_width
            or height != self.map_height
            or abs(resolution - self.map_resolution) > 1e-9
            or abs(msg.info.origin.position.x - self.map_origin_x) > 1e-6
            or abs(msg.info.origin.position.y - self.map_origin_y) > 1e-6
            or self.angle_difference(origin_yaw, self.map_origin_yaw) > 1e-6
        )

        self.map_msg = msg
        self.grid = grid
        self.map_width = width
        self.map_height = height
        self.map_resolution = resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_origin_yaw = origin_yaw
        self.last_map_received_wall = time.monotonic()

        if geometry_changed:
            self.covered = np.zeros(
                (height, width), dtype=np.uint8
            )
            self.last_mark_pose = None
            self.axis_history = []
            self.axis_preview = None
            self.mission_origin = None
            self.phase = "AUTO_AXIS_ESTIMATION"
            self.completed_zones.clear()
            self.failed_zones.clear()
            self.blacklist.clear()
            self.current_goal = None
            self.scan_position = None
            self.scan_queue = []
            rospy.loginfo(
                "[corridor_room_explorer] map geometry %dx%d res=%.3f",
                width,
                height,
                resolution,
            )

    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.25),
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                3.0,
                "[corridor_room_explorer] TF unavailable: %s",
                str(exc),
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        _, _, yaw = euler_from_quaternion(
            [rotation.x, rotation.y, rotation.z, rotation.w]
        )
        return translation.x, translation.y, yaw

    def world_to_grid(
        self, x: float, y: float
    ) -> Optional[Tuple[int, int]]:
        dx = x - self.map_origin_x
        dy = y - self.map_origin_y

        c = math.cos(-self.map_origin_yaw)
        s = math.sin(-self.map_origin_yaw)
        local_x = c * dx - s * dy
        local_y = s * dx + c * dy

        col = int(math.floor(local_x / self.map_resolution))
        row = int(math.floor(local_y / self.map_resolution))

        if (
            row < 0
            or row >= self.map_height
            or col < 0
            or col >= self.map_width
        ):
            return None
        return row, col

    def grid_to_world(
        self, row: int, col: int
    ) -> Tuple[float, float]:
        local_x = (col + 0.5) * self.map_resolution
        local_y = (row + 0.5) * self.map_resolution

        c = math.cos(self.map_origin_yaw)
        s = math.sin(self.map_origin_yaw)
        x = self.map_origin_x + c * local_x - s * local_y
        y = self.map_origin_y + s * local_x + c * local_y
        return x, y

    def sample_grid(self, x: float, y: float) -> int:
        if self.grid is None:
            return -2
        cell = self.world_to_grid(x, y)
        if cell is None:
            return -2
        row, col = cell
        return int(self.grid[row, col])

    def sample_clearance(
        self,
        clearance: np.ndarray,
        x: float,
        y: float,
    ) -> float:
        cell = self.world_to_grid(x, y)
        if cell is None:
            return 0.0
        row, col = cell
        return float(clearance[row, col])

    def build_clearance(self) -> Optional[np.ndarray]:
        if self.grid is None:
            return None
        occupied = (
            self.grid >= self.occupied_threshold
        ).astype(np.uint8)
        cells = cv2.distanceTransform(
            (occupied == 0).astype(np.uint8),
            cv2.DIST_L2,
            5,
        )
        return cells * self.map_resolution

    def evaluate_axis_direction(
        self,
        robot_x: float,
        robot_y: float,
        yaw: float,
        center_offset: float,
        direction_sign: float,
        clearance: np.ndarray,
    ) -> Tuple[float, Dict[str, float], float, float]:
        """
        Evaluate one directed corridor hypothesis.

        Returns:
          score, diagnostics, center_x, center_y
        """
        fx = math.cos(yaw) * direction_sign
        fy = math.sin(yaw) * direction_sign
        lx = -math.sin(yaw)
        ly = math.cos(yaw)

        center_x = robot_x + lx * center_offset
        center_y = robot_y + ly * center_offset

        s_values = np.arange(
            0.25,
            self.axis_sample_length + 1e-6,
            self.axis_sample_step,
        )
        t_values = np.arange(
            -self.axis_center_band_half_width,
            self.axis_center_band_half_width + 1e-6,
            self.axis_center_band_step,
        )

        clear_count = 0
        unknown_count = 0
        occupied_count = 0
        outside_count = 0
        consecutive_open = 0
        best_open_run = 0
        first_unknown_index = len(s_values)

        for s_index, s_value in enumerate(s_values):
            cross_open = 0
            cross_unknown = 0
            cross_blocked = 0

            for t_value in t_values:
                x = center_x + fx * s_value + lx * t_value
                y = center_y + fy * s_value + ly * t_value
                value = self.sample_grid(x, y)

                if value == -2:
                    outside_count += 1
                    cross_blocked += 1
                elif value < 0:
                    unknown_count += 1
                    cross_unknown += 1
                elif value >= self.occupied_threshold:
                    occupied_count += 1
                    cross_blocked += 1
                else:
                    local_clearance = self.sample_clearance(
                        clearance, x, y
                    )
                    if local_clearance >= 0.20:
                        clear_count += 1
                        cross_open += 1
                    else:
                        cross_blocked += 1

            cross_total = max(1, len(t_values))
            open_ratio = (
                cross_open + 0.50 * cross_unknown
            ) / cross_total

            if cross_unknown > 0 and first_unknown_index == len(s_values):
                first_unknown_index = s_index

            if open_ratio >= 0.55:
                consecutive_open += 1
                best_open_run = max(
                    best_open_run, consecutive_open
                )
            else:
                consecutive_open = 0

        wall_left_hits: List[float] = []
        wall_right_hits: List[float] = []

        wall_s_values = np.arange(
            self.axis_wall_probe_start,
            self.axis_wall_probe_end + 1e-6,
            self.axis_wall_probe_step,
        )
        wall_t_values = np.arange(
            self.axis_wall_search_min,
            self.axis_wall_search_max + 1e-6,
            self.axis_wall_search_step,
        )

        for s_value in wall_s_values:
            base_x = center_x + fx * s_value
            base_y = center_y + fy * s_value

            left_distance = None
            right_distance = None

            for t_value in wall_t_values:
                if left_distance is None:
                    left_value = self.sample_grid(
                        base_x + lx * t_value,
                        base_y + ly * t_value,
                    )
                    if left_value >= self.occupied_threshold:
                        left_distance = float(t_value)

                if right_distance is None:
                    right_value = self.sample_grid(
                        base_x - lx * t_value,
                        base_y - ly * t_value,
                    )
                    if right_value >= self.occupied_threshold:
                        right_distance = float(t_value)

                if (
                    left_distance is not None
                    and right_distance is not None
                ):
                    break

            if left_distance is not None:
                wall_left_hits.append(left_distance)
            if right_distance is not None:
                wall_right_hits.append(right_distance)

        probe_count = max(1, len(wall_s_values))
        left_ratio = len(wall_left_hits) / probe_count
        right_ratio = len(wall_right_hits) / probe_count
        both_wall_ratio = min(left_ratio, right_ratio)

        if wall_left_hits and wall_right_hits:
            left_median = float(np.median(wall_left_hits))
            right_median = float(np.median(wall_right_hits))
            balance_error = abs(left_median - right_median)
        else:
            left_median = 0.0
            right_median = 0.0
            balance_error = self.axis_wall_search_max

        total_samples = max(
            1,
            clear_count
            + unknown_count
            + occupied_count
            + outside_count,
        )
        clear_ratio = clear_count / total_samples
        unknown_ratio = unknown_count / total_samples
        blocked_ratio = (
            occupied_count + outside_count
        ) / total_samples
        continuity_ratio = best_open_run / max(1, len(s_values))

        # Unknown should exist ahead, but an entirely unknown direction is not
        # accepted unless wall/continuity evidence also supports a corridor.
        score = (
            self.axis_weight_clear * clear_ratio
            + self.axis_weight_unknown * unknown_ratio
            + self.axis_weight_wall * both_wall_ratio
            + self.axis_weight_continuity * continuity_ratio
            - self.axis_weight_balance * min(balance_error, 2.0)
            - self.axis_weight_blocked * blocked_ratio
        )

        # Prefer unknown that begins after at least some known traversable
        # corridor instead of immediately entering an unobserved void.
        known_prefix_ratio = min(
            first_unknown_index / max(1, len(s_values)),
            1.0,
        )
        if first_unknown_index == 0:
            score -= 2.0
        elif known_prefix_ratio > 0.05:
            score += min(known_prefix_ratio, 0.35)

        diagnostics = {
            "clear_ratio": clear_ratio,
            "unknown_ratio": unknown_ratio,
            "blocked_ratio": blocked_ratio,
            "continuity_ratio": continuity_ratio,
            "wall_ratio": both_wall_ratio,
            "left_wall_ratio": left_ratio,
            "right_wall_ratio": right_ratio,
            "left_median": left_median,
            "right_median": right_median,
            "balance_error": balance_error,
        }
        return score, diagnostics, center_x, center_y

    def estimate_corridor_axis(
        self,
        robot_pose: Tuple[float, float, float],
    ) -> Optional[AxisEstimate]:
        clearance = self.build_clearance()
        if clearance is None:
            return None

        robot_x, robot_y, _ = robot_pose

        def search_angles(
            angle_values: np.ndarray,
            current_best: Optional[AxisEstimate],
        ) -> Optional[AxisEstimate]:
            best = current_best
            offsets = np.arange(
                -self.axis_max_center_offset,
                self.axis_max_center_offset + 1e-6,
                self.axis_center_offset_step,
            )

            for undirected_yaw in angle_values:
                yaw = self.normalize_angle(float(undirected_yaw))

                for offset in offsets:
                    positive = self.evaluate_axis_direction(
                        robot_x,
                        robot_y,
                        yaw,
                        float(offset),
                        1.0,
                        clearance,
                    )
                    negative = self.evaluate_axis_direction(
                        robot_x,
                        robot_y,
                        yaw,
                        float(offset),
                        -1.0,
                        clearance,
                    )

                    positive_score, positive_diag, px, py = positive
                    negative_score, negative_diag, nx, ny = negative

                    # Direction choice strongly rewards unexplored depth.
                    positive_direction_score = (
                        positive_score
                        + self.axis_direction_unknown_weight
                        * positive_diag["unknown_ratio"]
                        + self.axis_direction_clear_weight
                        * positive_diag["continuity_ratio"]
                    )
                    negative_direction_score = (
                        negative_score
                        + self.axis_direction_unknown_weight
                        * negative_diag["unknown_ratio"]
                        + self.axis_direction_clear_weight
                        * negative_diag["continuity_ratio"]
                    )

                    if positive_direction_score >= negative_direction_score:
                        directed_yaw = yaw
                        direction_score = positive_direction_score
                        base_score = positive_score
                        diag = positive_diag
                        center_x, center_y = px, py
                    else:
                        directed_yaw = self.normalize_angle(yaw + math.pi)
                        direction_score = negative_direction_score
                        base_score = negative_score
                        diag = negative_diag
                        center_x, center_y = nx, ny

                    # The final score includes a small direction-separation
                    # reward. A corridor should have a clear preference for
                    # the unexplored side over the already-covered hall.
                    direction_separation = abs(
                        positive_direction_score - negative_direction_score
                    )
                    final_score = (
                        direction_score
                        + 0.20 * direction_separation
                    )

                    estimate = AxisEstimate(
                        x=center_x,
                        y=center_y,
                        yaw=directed_yaw,
                        score=final_score,
                        forward_score=direction_score,
                        reverse_score=(
                            negative_direction_score
                            if positive_direction_score
                            >= negative_direction_score
                            else positive_direction_score
                        ),
                        offset=float(offset),
                        wall_ratio=float(diag["wall_ratio"]),
                        balance_error=float(diag["balance_error"]),
                        unknown_ratio=float(diag["unknown_ratio"]),
                    )

                    if best is None or estimate.score > best.score:
                        best = estimate

            return best

        coarse_angles = np.deg2rad(
            np.arange(
                0.0,
                180.0,
                self.axis_angle_step_deg,
            )
        )
        best = search_angles(coarse_angles, None)
        if best is None:
            return None

        # Refine around the best undirected orientation.
        base_undirected = best.yaw % math.pi
        refine_offsets = np.arange(
            -self.axis_refine_window_deg,
            self.axis_refine_window_deg + 1e-6,
            self.axis_refine_step_deg,
        )
        refine_angles = np.asarray(
            [
                (base_undirected + math.radians(delta)) % math.pi
                for delta in refine_offsets
            ],
            dtype=np.float64,
        )
        best = search_angles(refine_angles, best)

        if best is None:
            return None

        if (
            best.score < self.axis_min_score
            or best.wall_ratio < self.axis_min_wall_ratio
        ):
            rospy.logwarn_throttle(
                2.0,
                "[corridor_room_explorer] axis confidence too low: "
                "score=%.2f min=%.2f wall_ratio=%.2f min=%.2f",
                best.score,
                self.axis_min_score,
                best.wall_ratio,
                self.axis_min_wall_ratio,
            )
            return None

        return best

    def update_axis_stability(
        self,
        estimate: AxisEstimate,
    ) -> bool:
        self.axis_preview = estimate

        if not self.axis_history:
            self.axis_history = [estimate]
        else:
            previous = self.axis_history[-1]
            angle_error = self.angle_difference(
                estimate.yaw, previous.yaw
            )
            center_error = math.hypot(
                estimate.x - previous.x,
                estimate.y - previous.y,
            )

            if (
                angle_error
                <= math.radians(self.axis_stability_angle_deg)
                and center_error <= self.axis_stability_center
            ):
                self.axis_history.append(estimate)
            else:
                rospy.logwarn(
                    "[corridor_room_explorer] axis estimate changed: "
                    "angle_error=%.2f deg center_error=%.2f m; "
                    "stability counter reset",
                    math.degrees(angle_error),
                    center_error,
                )
                self.axis_history = [estimate]

        if len(self.axis_history) > self.axis_stability_required:
            self.axis_history = self.axis_history[
                -self.axis_stability_required:
            ]

        count = len(self.axis_history)
        self.publish_status(
            f"AUTO_AXIS_ESTIMATION_{count}_OF_"
            f"{self.axis_stability_required}"
        )

        diagnostic = (
            f"count={count}/{self.axis_stability_required} "
            f"yaw_deg={math.degrees(estimate.yaw):.2f} "
            f"center=({estimate.x:.2f},{estimate.y:.2f}) "
            f"score={estimate.score:.2f} "
            f"wall_ratio={estimate.wall_ratio:.2f} "
            f"balance={estimate.balance_error:.2f} "
            f"unknown={estimate.unknown_ratio:.2f}"
        )
        self.axis_diagnostic_pub.publish(String(data=diagnostic))

        rospy.loginfo(
            "[corridor_room_explorer] axis estimate %d/%d "
            "yaw=%.2f deg center=(%.2f, %.2f) score=%.2f "
            "wall=%.2f balance=%.2f unknown=%.2f",
            count,
            self.axis_stability_required,
            math.degrees(estimate.yaw),
            estimate.x,
            estimate.y,
            estimate.score,
            estimate.wall_ratio,
            estimate.balance_error,
            estimate.unknown_ratio,
        )

        return count >= self.axis_stability_required

    def lock_corridor_axis(self) -> None:
        if not self.axis_history:
            return

        xs = np.asarray(
            [estimate.x for estimate in self.axis_history],
            dtype=np.float64,
        )
        ys = np.asarray(
            [estimate.y for estimate in self.axis_history],
            dtype=np.float64,
        )
        cos_values = np.asarray(
            [math.cos(estimate.yaw) for estimate in self.axis_history],
            dtype=np.float64,
        )
        sin_values = np.asarray(
            [math.sin(estimate.yaw) for estimate in self.axis_history],
            dtype=np.float64,
        )

        x = float(np.mean(xs))
        y = float(np.mean(ys))
        yaw = math.atan2(
            float(np.mean(sin_values)),
            float(np.mean(cos_values)),
        )

        self.mission_origin = (x, y, yaw)
        self.max_corridor_progress = 0.0
        self.phase = "CORRIDOR_ADVANCE"
        self.corridor_empty_cycles = 0
        self.publish_status("CORRIDOR_ADVANCE")

        rospy.loginfo(
            "[corridor_room_explorer] AUTOMATIC AXIS LOCKED "
            "center=(%.2f, %.2f) yaw=%.2f deg",
            x,
            y,
            math.degrees(yaw),
        )

    def project(self, x: float, y: float) -> Tuple[float, float]:
        if self.mission_origin is None:
            return 0.0, 0.0

        ox, oy, yaw = self.mission_origin
        dx = x - ox
        dy = y - oy

        fx = math.cos(yaw)
        fy = math.sin(yaw)
        lx = -fy
        ly = fx

        progress = fx * dx + fy * dy
        lateral = lx * dx + ly * dy
        return progress, lateral

    def mark_coverage(
        self,
        pose: Tuple[float, float, float],
    ) -> None:
        if self.covered is None:
            return

        x, y, _ = pose
        cell = self.world_to_grid(x, y)
        if cell is None:
            return

        if self.last_mark_pose is not None:
            dx = x - self.last_mark_pose[0]
            dy = y - self.last_mark_pose[1]
            if math.hypot(dx, dy) < 0.10:
                return

        row, col = cell
        radius_cells = max(
            1,
            int(round(
                self.coverage_radius / self.map_resolution
            )),
        )
        cv2.circle(
            self.covered,
            (col, row),
            radius_cells,
            1,
            thickness=-1,
        )
        self.last_mark_pose = (x, y)

    def map_arrays(self) -> Optional[Dict[str, np.ndarray]]:
        if (
            self.grid is None
            or self.covered is None
            or self.mission_origin is None
        ):
            return None

        grid = self.grid.copy()
        free = (
            (grid >= 0)
            & (grid < self.occupied_threshold)
        ).astype(np.uint8)
        occupied = (
            grid >= self.occupied_threshold
        ).astype(np.uint8)
        unknown = (grid < 0).astype(np.uint8)

        clearance_cells = cv2.distanceTransform(
            (occupied == 0).astype(np.uint8),
            cv2.DIST_L2,
            5,
        )
        clearance = clearance_cells * self.map_resolution

        safe = (
            (free > 0)
            & (clearance >= self.min_goal_clearance)
        )

        rows, cols = np.indices(grid.shape)
        local_x = (cols + 0.5) * self.map_resolution
        local_y = (rows + 0.5) * self.map_resolution

        c = math.cos(self.map_origin_yaw)
        s = math.sin(self.map_origin_yaw)
        world_x = (
            self.map_origin_x
            + c * local_x
            - s * local_y
        )
        world_y = (
            self.map_origin_y
            + s * local_x
            + c * local_y
        )

        ox, oy, heading = self.mission_origin
        dx = world_x - ox
        dy = world_y - oy

        fx = math.cos(heading)
        fy = math.sin(heading)
        lx = -fy
        ly = fx

        progress = (fx * dx + fy * dy).astype(np.float32)
        lateral = (lx * dx + ly * dy).astype(np.float32)

        return {
            "grid": grid,
            "free": free,
            "occupied": occupied,
            "unknown": unknown,
            "clearance": clearance,
            "safe": safe,
            "progress": progress,
            "lateral": lateral,
            "covered": self.covered.copy(),
        }

    def is_blacklisted(self, x: float, y: float) -> bool:
        for bx, by in self.blacklist:
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                return True
        return False

    def add_blacklist(self, x: float, y: float) -> None:
        self.blacklist.append((x, y))
        if len(self.blacklist) > 100:
            self.blacklist = self.blacklist[-100:]

    def candidate_distance_ok(
        self,
        x: float,
        y: float,
        robot_pose: Tuple[float, float, float],
    ) -> bool:
        distance = math.hypot(
            x - robot_pose[0],
            y - robot_pose[1],
        )
        return (
            self.min_goal_distance
            <= distance
            <= self.max_goal_distance
        )

    def select_corridor_candidate(
        self,
        arrays: Dict[str, np.ndarray],
        robot_pose: Tuple[float, float, float],
    ) -> Optional[Candidate]:
        unknown = arrays["unknown"]
        free = arrays["free"]
        occupied = arrays["occupied"]
        safe = arrays["safe"]
        progress = arrays["progress"]
        lateral = arrays["lateral"]
        clearance = arrays["clearance"]

        kernel = np.ones((3, 3), dtype=np.uint8)
        unknown_touch = cv2.dilate(
            unknown,
            kernel,
            iterations=1,
        )
        wall_guard = cv2.dilate(
            occupied,
            kernel,
            iterations=1,
        )

        frontier = (
            (free > 0)
            & (unknown_touch > 0)
            & (wall_guard == 0)
        ).astype(np.uint8)

        # Hard corridor-band and no-backtracking constraints.
        frontier[
            np.abs(lateral) > self.corridor_half_width
        ] = 0
        frontier[
            progress
            < (
                self.max_corridor_progress
                - self.backtrack_tolerance
            )
        ] = 0

        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            frontier,
            connectivity=8,
        )

        best: Optional[Candidate] = None
        radius_cells = max(
            1,
            int(round(
                self.frontier_goal_radius
                / self.map_resolution
            )),
        )
        local_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (
                2 * radius_cells + 1,
                2 * radius_cells + 1,
            ),
        )

        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.frontier_min_cells:
                continue

            component = (labels == label).astype(np.uint8)
            component_progress = float(
                np.max(progress[component > 0])
            )

            if (
                component_progress
                < self.max_corridor_progress + self.forward_margin
            ):
                continue

            neighborhood = cv2.dilate(
                component,
                local_kernel,
                iterations=1,
            )

            valid = (
                (neighborhood > 0)
                & safe
                & (
                    np.abs(lateral)
                    <= self.corridor_half_width
                )
                & (
                    progress
                    >= (
                        self.max_corridor_progress
                        - self.backtrack_tolerance
                    )
                )
            )

            valid_indices = np.argwhere(valid)
            if valid_indices.size == 0:
                continue

            scores = (
                10.0 * progress[valid]
                + 3.0 * clearance[valid]
                - self.corridor_center_weight
                * np.abs(lateral[valid])
            )
            order = np.argsort(scores)[::-1]

            chosen = None
            for index in order[:150]:
                row, col = valid_indices[int(index)]
                x, y = self.grid_to_world(
                    int(row), int(col)
                )

                if self.is_blacklisted(x, y):
                    continue
                if not self.candidate_distance_ok(
                    x, y, robot_pose
                ):
                    continue

                chosen = (
                    int(row),
                    int(col),
                    x,
                    y,
                    float(scores[int(index)]),
                )
                break

            if chosen is None:
                continue

            row, col, x, y, score = chosen
            candidate = Candidate(
                x=x,
                y=y,
                # Face the selected point instead of forcing the locked
                # corridor heading. This avoids an unnecessary reverse
                # segment when a frontier lies behind the robot's current
                # heading or on a short side branch.
                yaw=math.atan2(y - robot_pose[1], x - robot_pose[0]),
                kind="CORRIDOR_FRONTIER",
                score=score,
                progress=float(progress[row, col]),
                lateral=float(lateral[row, col]),
            )

            if (
                best is None
                or candidate.progress > best.progress + 0.05
                or (
                    abs(candidate.progress - best.progress) <= 0.05
                    and candidate.score > best.score
                )
            ):
                best = candidate

        if best is not None:
            return best

        # Fallback probe: deepest already-known safe point in corridor band.
        probe_mask = (
            safe
            & (
                np.abs(lateral)
                <= self.corridor_half_width
            )
            & (
                progress
                >= self.max_corridor_progress + self.forward_margin
            )
        )
        indices = np.argwhere(probe_mask)
        if indices.size == 0:
            return None

        scores = (
            12.0 * progress[probe_mask]
            + 3.0 * clearance[probe_mask]
            - self.corridor_center_weight
            * np.abs(lateral[probe_mask])
        )
        order = np.argsort(scores)[::-1]

        for index in order[:250]:
            row, col = indices[int(index)]
            x, y = self.grid_to_world(int(row), int(col))

            if self.is_blacklisted(x, y):
                continue
            if not self.candidate_distance_ok(
                x, y, robot_pose
            ):
                continue

            return Candidate(
                x=x,
                y=y,
                yaw=math.atan2(y - robot_pose[1], x - robot_pose[0]),
                kind="CORRIDOR_PROBE",
                score=float(scores[int(index)]),
                progress=float(progress[row, col]),
                lateral=float(lateral[row, col]),
            )

        return None

    def select_room_candidate(
        self,
        arrays: Dict[str, np.ndarray],
        robot_pose: Tuple[float, float, float],
    ) -> Optional[Candidate]:
        safe = arrays["safe"]
        progress = arrays["progress"]
        lateral = arrays["lateral"]
        covered = arrays["covered"]
        clearance = arrays["clearance"]

        unvisited = (
            safe
            & (covered == 0)
            & (progress >= self.room_progress_min)
            & (
                progress
                <= self.max_corridor_progress + 1.0
            )
            & (
                np.abs(lateral)
                >= self.room_lateral_min
            )
        )

        if not np.any(unvisited):
            return None

        depth_cells = cv2.distanceTransform(
            unvisited.astype(np.uint8),
            cv2.DIST_L2,
            5,
        )
        unvisited_depth = (
            depth_cells * self.map_resolution
        )

        max_bin = max(
            0,
            int(math.ceil(
                max(
                    self.max_corridor_progress,
                    self.room_progress_min,
                )
                / self.room_bin_length
            )),
        )

        candidates: List[Candidate] = []

        for bin_index in range(max_bin + 1):
            p0 = max(
                self.room_progress_min,
                bin_index * self.room_bin_length,
            )
            p1 = (bin_index + 1) * self.room_bin_length

            for side_name, side_sign in (
                ("LEFT", 1),
                ("RIGHT", -1),
            ):
                zone_id = f"{side_name}_{bin_index}"

                if zone_id in self.completed_zones:
                    continue
                if self.failed_zones.get(zone_id, 0) >= 3:
                    continue

                if side_sign > 0:
                    side_mask = (
                        lateral >= self.room_lateral_min
                    )
                else:
                    side_mask = (
                        lateral <= -self.room_lateral_min
                    )

                zone = (
                    unvisited
                    & side_mask
                    & (progress >= p0)
                    & (progress < p1)
                )

                area_m2 = (
                    float(np.count_nonzero(zone))
                    * self.map_resolution
                    * self.map_resolution
                )
                if area_m2 < self.room_min_area:
                    continue

                indices = np.argwhere(zone)
                scores = (
                    4.0 * unvisited_depth[zone]
                    + 2.0 * clearance[zone]
                    + 0.15 * progress[zone]
                )
                order = np.argsort(scores)[::-1]

                selected = None
                for index in order[:250]:
                    row, col = indices[int(index)]
                    x, y = self.grid_to_world(
                        int(row), int(col)
                    )

                    if self.is_blacklisted(x, y):
                        continue
                    if not self.candidate_distance_ok(
                        x, y, robot_pose
                    ):
                        continue

                    selected = (
                        int(row),
                        int(col),
                        x,
                        y,
                        float(scores[int(index)]),
                    )
                    break

                if selected is None:
                    continue

                row, col, x, y, score = selected
                yaw = math.atan2(
                    y - robot_pose[1],
                    x - robot_pose[0],
                )

                candidates.append(
                    Candidate(
                        x=x,
                        y=y,
                        yaw=yaw,
                        kind="ROOM",
                        score=score,
                        progress=float(progress[row, col]),
                        lateral=float(lateral[row, col]),
                        zone_id=zone_id,
                    )
                )

        if not candidates:
            return None

        # Explore far-end rooms first to avoid returning to the hall.
        candidates.sort(
            key=lambda candidate: (
                candidate.progress,
                candidate.score,
            ),
            reverse=True,
        )
        return candidates[0]

    def publish_markers(
        self,
        robot_pose: Tuple[float, float, float],
        selected: Optional[Candidate],
    ) -> None:
        markers = MarkerArray()
        now = rospy.Time.now()

        clear = Marker()
        clear.header.frame_id = self.global_frame
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        axis_source = None
        axis_locked = False

        if self.mission_origin is not None:
            axis_source = AxisEstimate(
                x=self.mission_origin[0],
                y=self.mission_origin[1],
                yaw=self.mission_origin[2],
                score=0.0,
                forward_score=0.0,
                reverse_score=0.0,
                offset=0.0,
                wall_ratio=0.0,
                balance_error=0.0,
                unknown_ratio=0.0,
            )
            axis_locked = True
        elif self.axis_preview is not None:
            axis_source = self.axis_preview

        if axis_source is not None:
            axis = Marker()
            axis.header.frame_id = self.global_frame
            axis.header.stamp = now
            axis.ns = "corridor_axis"
            axis.id = 1
            axis.type = Marker.LINE_STRIP
            axis.action = Marker.ADD
            axis.scale.x = 0.09

            if axis_locked:
                axis.color.r = 0.1
                axis.color.g = 0.8
                axis.color.b = 1.0
            else:
                axis.color.r = 1.0
                axis.color.g = 0.9
                axis.color.b = 0.1
            axis.color.a = 1.0

            backward = Point()
            backward.x = (
                axis_source.x
                - math.cos(axis_source.yaw) * 2.0
            )
            backward.y = (
                axis_source.y
                - math.sin(axis_source.yaw) * 2.0
            )
            backward.z = 0.15

            forward = Point()
            forward.x = (
                axis_source.x
                + math.cos(axis_source.yaw) * 18.0
            )
            forward.y = (
                axis_source.y
                + math.sin(axis_source.yaw) * 18.0
            )
            forward.z = 0.15

            axis.points = [backward, forward]
            markers.markers.append(axis)

        if selected is not None:
            goal = Marker()
            goal.header.frame_id = self.global_frame
            goal.header.stamp = now
            goal.ns = "selected_goal"
            goal.id = 2
            goal.type = Marker.ARROW
            goal.action = Marker.ADD
            goal.pose.position.x = selected.x
            goal.pose.position.y = selected.y
            goal.pose.position.z = 0.15

            quaternion = quaternion_from_euler(
                0.0, 0.0, selected.yaw
            )
            goal.pose.orientation.x = quaternion[0]
            goal.pose.orientation.y = quaternion[1]
            goal.pose.orientation.z = quaternion[2]
            goal.pose.orientation.w = quaternion[3]

            goal.scale.x = 0.8
            goal.scale.y = 0.18
            goal.scale.z = 0.18

            if selected.kind.startswith("CORRIDOR"):
                goal.color.r = 0.1
                goal.color.g = 1.0
                goal.color.b = 0.2
            else:
                goal.color.r = 1.0
                goal.color.g = 0.55
                goal.color.b = 0.1
            goal.color.a = 1.0
            markers.markers.append(goal)

            text = Marker()
            text.header.frame_id = self.global_frame
            text.header.stamp = now
            text.ns = "selected_goal_text"
            text.id = 3
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = selected.x
            text.pose.position.y = selected.y
            text.pose.position.z = 0.80
            text.scale.z = 0.35
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = (
                f"{self.phase} {selected.kind}\n"
                f"p={selected.progress:.2f} "
                f"lat={selected.lateral:.2f}"
            )
            markers.markers.append(text)

        self.marker_pub.publish(markers)

    def candidate_goal(self, candidate: Candidate) -> MoveBaseGoal:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = candidate.x
        goal.target_pose.pose.position.y = candidate.y

        quaternion = quaternion_from_euler(0.0, 0.0, candidate.yaw)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        return goal

    def candidate_has_plan(
        self,
        candidate: Candidate,
        robot_pose: Optional[Tuple[float, float, float]],
    ) -> bool:
        if self.dry_run or not self.require_make_plan:
            return True
        if robot_pose is None:
            self.publish_status("WAITING_FOR_NAVIGATION_TF")
            return False
        try:
            rospy.wait_for_service(
                self.plan_service_name, timeout=0.25
            )
            plan_service = rospy.ServiceProxy(
                self.plan_service_name, GetPlan
            )
            start = PoseStamped()
            start.header.frame_id = self.global_frame
            start.header.stamp = rospy.Time.now()
            start.pose.position.x = robot_pose[0]
            start.pose.position.y = robot_pose[1]
            start_quaternion = quaternion_from_euler(0.0, 0.0, robot_pose[2])
            start.pose.orientation.x = start_quaternion[0]
            start.pose.orientation.y = start_quaternion[1]
            start.pose.orientation.z = start_quaternion[2]
            start.pose.orientation.w = start_quaternion[3]
            goal = self.candidate_goal(candidate).target_pose
            response = plan_service(start, goal, self.plan_tolerance)
            if len(response.plan.poses) < 2:
                rospy.logwarn(
                    "[corridor_room_explorer] no plan for %s at (%.2f, %.2f)",
                    candidate.kind,
                    candidate.x,
                    candidate.y,
                )
                return False
            return True
        except (rospy.ROSException, rospy.ServiceException) as exc:
            self.navigation_blocked_until = (
                rospy.Time.now()
                + rospy.Duration(self.navigation_retry_cooldown)
            )
            rospy.logwarn_throttle(
                3.0,
                "[corridor_room_explorer] plan service unavailable: %s",
                str(exc),
            )
            self.publish_status("WAITING_FOR_PLAN_SERVICE")
            return False

    def send_candidate(
        self,
        candidate: Candidate,
        robot_pose: Optional[Tuple[float, float, float]] = None,
    ) -> bool:
        self.last_selected = candidate

        rospy.loginfo(
            "[corridor_room_explorer] selected phase=%s kind=%s "
            "x=%.2f y=%.2f progress=%.2f lateral=%.2f "
            "zone=%s dry_run=%s",
            self.phase,
            candidate.kind,
            candidate.x,
            candidate.y,
            candidate.progress,
            candidate.lateral,
            candidate.zone_id,
            self.dry_run,
        )

        if self.dry_run:
            self.publish_status(
                f"DRY_RUN_{self.phase}_{candidate.kind}"
            )
            return True

        if self.action_goal_in_flight():
            self.client.cancel_all_goals()
            self.publish_status("WAITING_FOR_MOVE_BASE_CANCEL")
            return False

        if rospy.Time.now() < self.navigation_blocked_until:
            self.publish_status("WAITING_FOR_NAVIGATION")
            return False
        if not self.candidate_has_plan(candidate, robot_pose):
            self.add_blacklist(candidate.x, candidate.y)
            if candidate.zone_id:
                self.failed_zones[candidate.zone_id] = (
                    self.failed_zones.get(candidate.zone_id, 0) + 1
                )
            self.publish_status("GOAL_REJECTED_NO_PLAN")
            return False

        goal = self.candidate_goal(candidate)
        self.client.send_goal(goal)

        self.current_goal = candidate
        self.current_goal_start = rospy.Time.now()
        self.last_goal_progress_time = rospy.Time.now()
        self.last_goal_distance = float("inf")
        self.publish_status(
            f"NAVIGATING_{candidate.kind}"
        )
        return True

    def start_room_scan(
        self,
        candidate: Candidate,
    ) -> None:
        self.scan_position = (candidate.x, candidate.y)
        self.scan_zone_id = candidate.zone_id
        self.scan_queue = [
            0.0,
            math.pi / 2.0,
            math.pi,
            -math.pi / 2.0,
        ]
        self.scan_wait_until = rospy.Time.now()
        self.publish_status(
            f"ROOM_SCAN_PREPARE_{candidate.zone_id}"
        )

    def send_next_scan_goal(self) -> None:
        if self.scan_position is None or not self.scan_queue:
            if self.scan_zone_id:
                self.completed_zones.add(self.scan_zone_id)
                rospy.loginfo(
                    "[corridor_room_explorer] room zone complete: %s",
                    self.scan_zone_id,
                )
            self.scan_position = None
            self.scan_zone_id = ""
            self.scan_queue = []
            self.publish_status("ROOM_SCAN_COMPLETE")
            return

        yaw = self.scan_queue.pop(0)
        candidate = Candidate(
            x=self.scan_position[0],
            y=self.scan_position[1],
            yaw=yaw,
            kind="SCAN",
            score=0.0,
            progress=0.0,
            lateral=0.0,
            zone_id=self.scan_zone_id,
        )
        if not self.send_candidate(candidate, self.get_robot_pose()):
            if self.failed_zones.get(self.scan_zone_id, 0) >= 3:
                rospy.logwarn(
                    "[corridor_room_explorer] aborting room scan after "
                    "repeated plan failures: %s",
                    self.scan_zone_id,
                )
                self.scan_position = None
                self.scan_zone_id = ""
                self.scan_queue = []
                self.publish_status("ROOM_SCAN_ABORTED_NO_PLAN")
            else:
                self.scan_queue.insert(0, yaw)

    def finish_evidence_ready(self) -> bool:
        rooms_ready = len(self.completed_zones) >= self.required_room_zones
        dangers_ready = (
            not self.require_danger_confirmation
            or len(self.confirmed_danger_ids)
            >= self.minimum_confirmed_dangers
        )
        return rooms_ready and dangers_ready

    def handle_current_goal(
        self,
        robot_pose: Tuple[float, float, float],
    ) -> None:
        if self.current_goal is None:
            return

        candidate = self.current_goal
        state = self.client.get_state()
        now = rospy.Time.now()

        if state in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            distance = math.hypot(
                candidate.x - robot_pose[0],
                candidate.y - robot_pose[1],
            )

            if (
                self.last_goal_distance - distance
                >= self.progress_epsilon
            ):
                self.last_goal_distance = distance
                self.last_goal_progress_time = now

            total_time = (
                now - self.current_goal_start
            ).to_sec()
            no_progress_time = (
                now - self.last_goal_progress_time
            ).to_sec()

            if (
                total_time > self.goal_timeout
                or no_progress_time > self.progress_timeout
            ):
                rospy.logwarn(
                    "[corridor_room_explorer] goal timeout kind=%s "
                    "distance=%.2f total=%.1f no_progress=%.1f",
                    candidate.kind,
                    distance,
                    total_time,
                    no_progress_time,
                )
                self.client.cancel_goal()
                self.add_blacklist(candidate.x, candidate.y)

                if candidate.zone_id:
                    self.failed_zones[candidate.zone_id] = (
                        self.failed_zones.get(
                            candidate.zone_id, 0
                        )
                        + 1
                    )

                self.current_goal = None
                self.publish_status("GOAL_TIMEOUT")
            return

        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo(
                "[corridor_room_explorer] goal reached kind=%s "
                "progress=%.2f zone=%s",
                candidate.kind,
                candidate.progress,
                candidate.zone_id,
            )
            self.current_goal = None

            if candidate.kind == "ROOM":
                self.start_room_scan(candidate)
            elif candidate.kind == "SCAN":
                self.scan_wait_until = (
                    rospy.Time.now()
                    + rospy.Duration(self.scan_pause)
                )
                self.publish_status(
                    f"SCAN_PAUSE_{candidate.zone_id}"
                )
            else:
                self.publish_status(
                    f"REACHED_{candidate.kind}"
                )
            return

        if state in (
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.PREEMPTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        ):
            rospy.logwarn(
                "[corridor_room_explorer] goal failed state=%d "
                "kind=%s zone=%s",
                state,
                candidate.kind,
                candidate.zone_id,
            )
            self.add_blacklist(candidate.x, candidate.y)
            if candidate.zone_id:
                self.failed_zones[candidate.zone_id] = (
                    self.failed_zones.get(candidate.zone_id, 0)
                    + 1
                )
            self.current_goal = None
            self.publish_status("GOAL_FAILED")

    def timer_callback(self, _event) -> None:
        if self.finished:
            return

        if not self.mapping_health_ready():
            if self.current_goal is not None and not self.dry_run:
                self.client.cancel_goal()
                self.current_goal = None
            self.publish_status("WAITING_FOR_MAPPING_HEALTH")
            return

        if self.action_goal_in_flight() and self.current_goal is None:
            self.client.cancel_all_goals()
            self.publish_status("WAITING_FOR_MOVE_BASE_CANCEL")
            return

        if self.grid is None or self.covered is None:
            self.publish_status("WAITING_FOR_MAP")
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.publish_status("WAITING_FOR_TF")
            return

        if self.phase == "AUTO_AXIS_ESTIMATION":
            now = rospy.Time.now()
            elapsed = (
                now - self.last_axis_estimate_time
            ).to_sec()

            if elapsed >= self.axis_reestimate_period:
                self.last_axis_estimate_time = now
                estimate = self.estimate_corridor_axis(
                    robot_pose
                )

                if estimate is None:
                    self.axis_history = []
                    self.axis_preview = None
                    self.publish_status(
                        "AUTO_AXIS_LOW_CONFIDENCE"
                    )
                else:
                    stable = self.update_axis_stability(
                        estimate
                    )
                    if stable:
                        self.lock_corridor_axis()

            self.publish_markers(robot_pose, None)
            return

        self.mark_coverage(robot_pose)

        progress, _ = self.project(
            robot_pose[0],
            robot_pose[1],
        )
        self.max_corridor_progress = max(
            self.max_corridor_progress,
            progress,
        )

        if self.current_goal is not None:
            self.handle_current_goal(robot_pose)
            self.publish_markers(
                robot_pose, self.last_selected
            )
            return

        if self.scan_position is not None:
            if rospy.Time.now() >= self.scan_wait_until:
                self.send_next_scan_goal()
            self.publish_markers(
                robot_pose, self.last_selected
            )
            return

        arrays = self.map_arrays()
        if arrays is None:
            return

        selected: Optional[Candidate] = None

        if self.phase == "CORRIDOR_ADVANCE":
            selected = self.select_corridor_candidate(
                arrays, robot_pose
            )

            if selected is None:
                self.corridor_empty_cycles += 1
                rospy.logwarn_throttle(
                    2.0,
                    "[corridor_room_explorer] no forward corridor "
                    "target cycle=%d/%d deepest=%.2f",
                    self.corridor_empty_cycles,
                    self.corridor_empty_cycles_to_finish,
                    self.max_corridor_progress,
                )

                if (
                    self.corridor_empty_cycles
                    >= self.corridor_empty_cycles_to_finish
                    and self.max_corridor_progress
                    >= self.minimum_corridor_progress
                ):
                    self.phase = "ROOM_SWEEP"
                    self.room_empty_cycles = 0
                    self.publish_status("ROOM_SWEEP")
                    rospy.loginfo(
                        "[corridor_room_explorer] phase transition "
                        "CORRIDOR_ADVANCE -> ROOM_SWEEP "
                        "deepest=%.2f",
                        self.max_corridor_progress,
                    )
            else:
                self.corridor_empty_cycles = 0
                self.send_candidate(selected, robot_pose)

        elif self.phase == "ROOM_SWEEP":
            selected = self.select_room_candidate(
                arrays, robot_pose
            )

            if selected is None:
                self.room_empty_cycles += 1
                rospy.logwarn_throttle(
                    2.0,
                    "[corridor_room_explorer] no room target "
                    "cycle=%d/%d completed=%s",
                    self.room_empty_cycles,
                    self.room_empty_cycles_to_finish,
                    sorted(self.completed_zones),
                )

                if (
                    self.room_empty_cycles
                    >= self.room_empty_cycles_to_finish
                ):
                    if self.finish_evidence_ready():
                        self.phase = "FINISHED"
                        self.finished = True
                        self.finished_pub.publish(Bool(data=True))
                        self.publish_status("FINISHED")
                        rospy.loginfo(
                            "[corridor_room_explorer] exploration finished "
                            "rooms=%d dangers=%d",
                            len(self.completed_zones),
                            len(self.confirmed_danger_ids),
                        )
                    else:
                        self.publish_status(
                            "ROOM_SWEEP_WAITING_FOR_EVIDENCE_"
                            f"rooms={len(self.completed_zones)}/"
                            f"{self.required_room_zones}_dangers="
                            f"{len(self.confirmed_danger_ids)}/"
                            f"{self.minimum_confirmed_dangers}"
                        )
                        self.room_empty_cycles = 0
            else:
                self.room_empty_cycles = 0
                self.send_candidate(selected, robot_pose)

        self.publish_markers(robot_pose, selected)


if __name__ == "__main__":
    try:
        CorridorRoomExplorer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
