#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import heapq
import importlib.util
import json
import math
from pathlib import Path
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import actionlib
import cv2
import numpy as np
import rospy
import tf2_ros

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

try:
    from graph_nbv_policy import (
        failure_budget_exhausted,
        freshness_age,
        gate_exhaustion_reached,
        pose_is_close,
        pose_is_reasonable,
        pose_samples_stable,
    )
except ImportError:
    # catkin's devel relay executes this source file while keeping the relay
    # directory on sys.path. Load the sibling helper explicitly so a rebuild
    # is not required just to start the node after a source-only update.
    _policy_path = Path(__file__).resolve().with_name("graph_nbv_policy.py")
    _policy_spec = importlib.util.spec_from_file_location(
        "graph_nbv_policy",
        str(_policy_path),
    )
    if _policy_spec is None or _policy_spec.loader is None:
        raise
    _policy_module = importlib.util.module_from_spec(_policy_spec)
    _policy_spec.loader.exec_module(_policy_module)
    failure_budget_exhausted = _policy_module.failure_budget_exhausted
    freshness_age = _policy_module.freshness_age
    gate_exhaustion_reached = _policy_module.gate_exhaustion_reached
    pose_is_close = _policy_module.pose_is_close
    pose_is_reasonable = _policy_module.pose_is_reasonable
    pose_samples_stable = _policy_module.pose_samples_stable


@dataclass
class GraphNode:
    index: int
    row: int
    col: int
    x: float
    y: float
    clearance: float
    visible_unknown_gain: float = 0.0
    visible_frontier_cells: int = 0
    path_cost: float = float("inf")
    revisit: float = 0.0
    utility: float = -float("inf")
    is_target: bool = False


@dataclass
class GlobalTarget:
    x: float
    y: float
    yaw: float
    utility: float
    path_cost: float
    visible_unknown_gain: float
    frontier_cells: int
    component_id: int


class GraphNBVStageB31ManualGate:
    """
    Stage B.3:
      - performs an initial straight drive before exploration;
      - records a gate plane from the measured map-frame motion;
      - locks local/global candidates and complete global plans to the
        forward side until reachable frontiers there are exhausted;
      - retains Stage B.2 transit-node and global-relocation behavior.
    """

    def __init__(self) -> None:
        rospy.init_node("graph_nbv")

        self.map_topic = rospy.get_param("~map_topic", "/map_confirmed")
        self.global_frame = rospy.get_param("~global_frame", "map_level")
        self.base_frame = rospy.get_param("~base_frame", "body")
        self.move_base_action = rospy.get_param(
            "~move_base_action", "/move_base"
        )
        self.make_plan_service = rospy.get_param(
            "~make_plan_service", "/move_base/make_plan"
        )
        self.dry_run = bool(rospy.get_param("~dry_run", True))

        # Initial straight drive.
        self.initial_forward_enabled = bool(
            rospy.get_param("~initial_forward_enabled", True)
        )
        self.initial_forward_only = bool(
            rospy.get_param("~initial_forward_only", False)
        )
        self.cmd_vel_topic = rospy.get_param(
            "~cmd_vel_topic", "/cmd_vel"
        )
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.odom_base_frame = rospy.get_param(
            "~odom_base_frame", "base"
        )
        self.initial_forward_speed_x = float(
            rospy.get_param("~initial_forward_speed_x", 1.0)
        )
        self.initial_forward_speed_y = float(
            rospy.get_param("~initial_forward_speed_y", -0.005)
        )
        self.initial_forward_slow_speed_x = float(
            rospy.get_param("~initial_forward_slow_speed_x", 0.25)
        )
        self.initial_forward_slow_speed_y = float(
            rospy.get_param("~initial_forward_slow_speed_y", -0.002)
        )
        self.initial_forward_slowdown_distance = float(
            rospy.get_param("~initial_forward_slowdown_distance", 11.0)
        )
        self.initial_forward_stop_distance = float(
            rospy.get_param("~initial_forward_stop_distance", 11.85)
        )
        self.initial_forward_timeout = float(
            rospy.get_param("~initial_forward_timeout", 20.0)
        )
        self.initial_settle_time = float(
            rospy.get_param("~initial_settle_time", 2.0)
        )
        self.initial_control_rate = float(
            rospy.get_param("~initial_control_rate", 20.0)
        )
        self.initial_obstacle_check_enabled = bool(
            rospy.get_param("~initial_obstacle_check_enabled", True)
        )
        self.initial_obstacle_stop_distance = float(
            rospy.get_param("~initial_obstacle_stop_distance", 1.20)
        )
        self.initial_obstacle_check_width = float(
            rospy.get_param("~initial_obstacle_check_width", 0.70)
        )

        # Manual gate mode:
        # The robot is manually driven to the corridor start before this
        # node is launched. The current map pose becomes the gate origin,
        # and the current body heading becomes the allowed forward direction.
        self.manual_gate_enabled = bool(
            rospy.get_param("~manual_gate_enabled", True)
        )
        self.manual_gate_use_robot_heading = bool(
            rospy.get_param("~manual_gate_use_robot_heading", True)
        )
        self.manual_gate_forward_x = float(
            rospy.get_param("~manual_gate_forward_x", 1.0)
        )
        self.manual_gate_forward_y = float(
            rospy.get_param("~manual_gate_forward_y", 0.0)
        )

        # Direction-projection gate.
        self.gate_backtrack_margin = float(
            rospy.get_param("~gate_backtrack_margin", 0.0)
        )
        self.forward_finish_stable_cycles = int(
            rospy.get_param("~forward_finish_stable_cycles", 8)
        )
        self.behind_gate_penalty_after_unlock = float(
            rospy.get_param(
                "~behind_gate_penalty_after_unlock", 10.0
            )
        )
        self.unlock_gate_on_exhaustion = bool(
            rospy.get_param("~unlock_gate_on_exhaustion", True)
        )
        self.manual_gate_stability_samples = max(
            2,
            int(rospy.get_param("~manual_gate_stability_samples", 5)),
        )
        self.manual_gate_stability_position = float(
            rospy.get_param("~manual_gate_stability_position", 0.12)
        )
        self.manual_gate_stability_angle = math.radians(
            float(rospy.get_param("~manual_gate_stability_angle_deg", 8.0))
        )
        self.manual_gate_wait_timeout = float(
            rospy.get_param("~manual_gate_wait_timeout", 30.0)
        )

        self.occupied_threshold = int(
            rospy.get_param("~occupied_threshold", 50)
        )

        # Safe graph sampling
        self.local_sampling_radius = float(
            rospy.get_param("~local_sampling_radius", 8.0)
        )
        self.candidate_spacing = float(
            rospy.get_param("~candidate_spacing", 0.80)
        )
        self.min_candidate_clearance = float(
            rospy.get_param("~min_candidate_clearance", 0.48)
        )
        self.edge_max_length = float(
            rospy.get_param("~edge_max_length", 2.20)
        )
        self.edge_collision_step = float(
            rospy.get_param("~edge_collision_step", 0.10)
        )
        self.max_local_nodes = int(
            rospy.get_param("~max_local_nodes", 550)
        )

        # Visibility-aware information gain
        self.information_radius = float(
            rospy.get_param("~information_radius", 3.50)
        )
        self.visibility_ray_count = int(
            rospy.get_param("~visibility_ray_count", 180)
        )
        self.visibility_ray_step = float(
            rospy.get_param("~visibility_ray_step", 0.10)
        )
        self.visibility_wall_dilation = float(
            rospy.get_param("~visibility_wall_dilation", 0.12)
        )
        self.visibility_unknown_depth = float(
            rospy.get_param("~visibility_unknown_depth", 1.20)
        )
        self.min_visible_unknown_gain = float(
            rospy.get_param("~min_visible_unknown_gain", 0.15)
        )
        self.min_visible_frontier_cells = int(
            rospy.get_param("~min_visible_frontier_cells", 2)
        )

        # Local utility
        self.weight_unknown_gain = float(
            rospy.get_param("~weight_unknown_gain", 2.20)
        )
        self.weight_frontier_gain = float(
            rospy.get_param("~weight_frontier_gain", 0.15)
        )
        self.weight_clearance = float(
            rospy.get_param("~weight_clearance", 1.20)
        )
        self.weight_path_cost = float(
            rospy.get_param("~weight_path_cost", 0.42)
        )
        self.weight_revisit = float(
            rospy.get_param("~weight_revisit", 2.50)
        )
        self.preferred_clearance = float(
            rospy.get_param("~preferred_clearance", 0.75)
        )
        self.weight_wall_proximity = float(
            rospy.get_param("~weight_wall_proximity", 5.0)
        )

        # Global relocation
        self.global_frontier_min_cells = int(
            rospy.get_param("~global_frontier_min_cells", 4)
        )
        self.global_frontier_max_components = int(
            rospy.get_param("~global_frontier_max_components", 20)
        )
        self.global_approach_min_distance = float(
            rospy.get_param("~global_approach_min_distance", 0.45)
        )
        self.global_approach_max_distance = float(
            rospy.get_param("~global_approach_max_distance", 1.60)
        )
        self.global_candidates_per_component = int(
            rospy.get_param("~global_candidates_per_component", 3)
        )
        self.global_max_plan_checks = int(
            rospy.get_param("~global_max_plan_checks", 18)
        )
        self.global_plan_tolerance = float(
            rospy.get_param("~global_plan_tolerance", 0.30)
        )
        self.weight_global_frontier_size = float(
            rospy.get_param("~weight_global_frontier_size", 0.04)
        )
        self.weight_global_path_cost = float(
            rospy.get_param("~weight_global_path_cost", 0.30)
        )
        self.weight_global_revisit = float(
            rospy.get_param("~weight_global_revisit", 2.50)
        )

        # Coverage and navigation
        self.coverage_radius = float(
            rospy.get_param("~coverage_radius", 0.90)
        )
        self.goal_min_distance = float(
            rospy.get_param("~goal_min_distance", 0.70)
        )
        self.goal_max_distance = float(
            rospy.get_param("~goal_max_distance", 30.0)
        )
        self.goal_timeout = float(
            rospy.get_param("~goal_timeout", 120.0)
        )
        self.goal_reached_radius = float(
            rospy.get_param("~goal_reached_radius", 0.60)
        )
        self.no_progress_timeout = float(
            rospy.get_param("~no_progress_timeout", 15.0)
        )
        self.progress_epsilon = float(
            rospy.get_param("~progress_epsilon", 0.20)
        )
        self.blacklist_radius = float(
            rospy.get_param("~blacklist_radius", 1.00)
        )
        self.global_empty_cycles_to_finish = int(
            rospy.get_param("~global_empty_cycles_to_finish", 30)
        )
        self.plan_period = float(
            rospy.get_param("~plan_period", 1.50)
        )
        self.goal_recovery_pause = float(
            rospy.get_param("~goal_recovery_pause", 2.0)
        )
        self.max_consecutive_goal_failures = max(
            1,
            int(rospy.get_param("~max_consecutive_goal_failures", 5)),
        )
        self.map_freshness_timeout = float(
            rospy.get_param("~map_freshness_timeout", 5.0)
        )
        self.tf_freshness_timeout = float(
            rospy.get_param("~tf_freshness_timeout", 2.0)
        )
        self.health_failure_limit = max(
            1,
            int(rospy.get_param("~health_failure_limit", 3)),
        )
        self.max_pose_abs = float(
            rospy.get_param("~max_pose_abs", 200.0)
        )

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rospy.Duration(20.0)
        )
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.client = actionlib.SimpleActionClient(
            self.move_base_action,
            MoveBaseAction,
        )
        self.make_plan_client = rospy.ServiceProxy(
            self.make_plan_service,
            GetPlan,
            persistent=False,
        )
        self.make_plan_available = False

        self.grid: Optional[np.ndarray] = None
        self.covered: Optional[np.ndarray] = None
        self.map_width = 0
        self.map_height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0

        self.current_local_goal: Optional[GraphNode] = None
        self.current_global_goal: Optional[GlobalTarget] = None
        self.current_goal_kind = ""
        self.current_goal_start = rospy.Time(0)
        self.last_goal_distance = float("inf")
        self.last_progress_time = rospy.Time(0)
        self.current_goal_id = ""
        self.goal_sequence = 0
        self.last_goal_id = ""
        self.last_goal_result = ""
        self.last_goal_reason = ""
        self.last_goal_elapsed = 0.0
        self.total_goals_sent = 0
        self.total_goal_successes = 0
        self.total_goal_failures = 0
        self.consecutive_goal_failures = 0
        self.recovery_until = rospy.Time(0)
        self.recovery_reason = ""

        self.blacklist: List[Tuple[float, float]] = []
        self.global_empty_cycles = 0
        self.finished = False
        self.aborted = False
        self.exploration_state = "WAITING_FOR_MAP"
        self.state_changed_at = rospy.Time.now()
        self.last_status_text = ""
        self.last_error = ""
        self.last_map_stamp = rospy.Time(0)
        self.last_map_received_at = rospy.Time(0)
        self.last_tf_stamp = rospy.Time(0)
        self.last_tf_received_at = rospy.Time(0)
        self.last_robot_pose: Optional[Tuple[float, float, float]] = None
        self.health_failure_count = 0
        self.health_reason = ""
        self.manual_gate_wait_start = rospy.Time(0)
        self.manual_gate_samples: List[Tuple[float, float, float]] = []
        self.gate_unlock_count = 0

        if self.manual_gate_enabled:
            self.initial_state = "WAITING_MANUAL_GATE"
            self.initial_complete = False
        elif self.initial_forward_enabled:
            self.initial_state = "WAITING_INITIAL_POSE"
            self.initial_complete = False
        else:
            self.initial_state = "DISABLED"
            self.initial_complete = True
        self.initial_paused = False
        self.initial_start_odom: Optional[
            Tuple[float, float, float]
        ] = None
        self.initial_start_map: Optional[
            Tuple[float, float, float]
        ] = None
        self.initial_start_time = rospy.Time(0)
        self.initial_settle_start = rospy.Time(0)

        self.gate_origin: Optional[Tuple[float, float]] = None
        self.gate_forward: Optional[Tuple[float, float]] = None
        self.gate_locked = False
        self.forward_empty_cycles = 0

        self.last_nodes: List[GraphNode] = []
        self.last_edges: List[Tuple[int, int]] = []
        self.last_local_selected: Optional[GraphNode] = None
        self.last_global_selected: Optional[GlobalTarget] = None
        self.last_global_frontiers: List[Tuple[float, float]] = []

        self.status_pub = rospy.Publisher(
            "~status", String, queue_size=1, latch=True
        )
        self.runtime_status_pub = rospy.Publisher(
            "~runtime_status", String, queue_size=1, latch=True
        )
        self.mode_pub = rospy.Publisher(
            "~mode", String, queue_size=1, latch=True
        )
        self.finished_pub = rospy.Publisher(
            "~finished", Bool, queue_size=1, latch=True
        )
        self.marker_pub = rospy.Publisher(
            "~markers", MarkerArray, queue_size=1, latch=True
        )
        self.gate_state_pub = rospy.Publisher(
            "~gate_state", String, queue_size=1, latch=True
        )
        self.cmd_vel_pub = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=1
        )
        rospy.on_shutdown(self.stop_initial_motion)

        rospy.Subscriber(
            self.map_topic,
            OccupancyGrid,
            self.map_callback,
            queue_size=1,
        )

        if not self.dry_run:
            rospy.loginfo(
                "[graph_nbv] waiting for move_base action: %s",
                self.move_base_action,
            )
            self.client.wait_for_server()
            rospy.loginfo("[graph_nbv] move_base is available")

        try:
            rospy.wait_for_service(
                self.make_plan_service,
                timeout=3.0,
            )
            self.make_plan_available = True
            rospy.loginfo(
                "[graph_nbv] make_plan service available: %s",
                self.make_plan_service,
            )
        except rospy.ROSException:
            rospy.logwarn(
                "[graph_nbv] make_plan is not available yet; "
                "global relocation will retry later"
            )

        rospy.Timer(
            rospy.Duration(
                1.0 / max(1.0, self.initial_control_rate)
            ),
            self.initial_control_callback,
        )
        rospy.Timer(
            rospy.Duration(self.plan_period),
            self.timer_callback,
        )

        self.finished_pub.publish(Bool(data=False))
        self.publish_status("WAITING_FOR_MAP")
        if self.manual_gate_enabled:
            self.mode_pub.publish(String(data="WAITING_MANUAL_GATE"))
        else:
            self.mode_pub.publish(String(data="INITIAL_FORWARD"))
        self.gate_state_pub.publish(String(data="NOT_INITIALIZED"))
        rospy.loginfo(
            "[graph_nbv] Stage B.3.1 started dry_run=%s "
            "manual_gate=%s initial_forward=%s "
            "initial_forward_only=%s",
            self.dry_run,
            self.manual_gate_enabled,
            self.initial_forward_enabled,
            self.initial_forward_only,
        )

    def publish_status(self, text: str) -> None:
        self.last_status_text = text
        self.status_pub.publish(String(data=text))
        self.publish_runtime_status()

    @staticmethod
    def _finite_or_none(value: float) -> Optional[float]:
        if value is None or not math.isfinite(float(value)):
            return None
        return float(value)

    @staticmethod
    def _time_age(
        stamp: rospy.Time,
        now: rospy.Time,
    ) -> Optional[float]:
        if stamp is None or stamp.to_sec() <= 0.0 or now.to_sec() <= 0.0:
            return None
        return max(0.0, (now - stamp).to_sec())

    def publish_runtime_status(self) -> None:
        """Publish a bounded, machine-readable mission heartbeat."""

        now = rospy.Time.now()
        goal_xy = self.current_goal_xy()
        map_stamp_age = self._time_age(self.last_map_stamp, now)
        map_receive_age = self._time_age(
            self.last_map_received_at, now
        )
        map_age = freshness_age(
            self.last_map_stamp.to_sec(),
            self.last_map_received_at.to_sec(),
            now.to_sec(),
        )
        tf_stamp_age = self._time_age(self.last_tf_stamp, now)
        tf_receive_age = self._time_age(
            self.last_tf_received_at, now
        )
        tf_age = freshness_age(
            self.last_tf_stamp.to_sec(),
            self.last_tf_received_at.to_sec(),
            now.to_sec(),
        )
        goal_elapsed = 0.0
        if self.current_goal_start.to_sec() > 0.0 and now.to_sec() > 0.0:
            goal_elapsed = max(
                0.0,
                (now - self.current_goal_start).to_sec(),
            )

        recovery_remaining = 0.0
        if self.recovery_until.to_sec() > 0.0 and now.to_sec() > 0.0:
            recovery_remaining = max(
                0.0,
                (self.recovery_until - now).to_sec(),
            )

        payload = {
            "schema": "graph_nbv_runtime_v1",
            "ros_time": float(now.to_sec()),
            "state": self.exploration_state,
            "state_age": self._time_age(self.state_changed_at, now),
            "status": self.last_status_text,
            "last_error": self.last_error or None,
            "initial_state": self.initial_state,
            "initial_complete": bool(self.initial_complete),
            "finished": bool(self.finished and not self.aborted),
            "aborted": bool(self.aborted),
            "gate": {
                "locked": bool(self.gate_locked),
                "forward_empty_cycles": int(self.forward_empty_cycles),
                "unlock_count": int(self.gate_unlock_count),
                "origin": list(self.gate_origin)
                if self.gate_origin is not None
                else None,
                "forward": list(self.gate_forward)
                if self.gate_forward is not None
                else None,
            },
            "goal": {
                "id": self.current_goal_id or None,
                "kind": self.current_goal_kind or None,
                "xy": list(goal_xy) if goal_xy is not None else None,
                "elapsed": float(goal_elapsed),
                "distance": self._finite_or_none(self.last_goal_distance),
                "last_id": self.last_goal_id or None,
                "last_result": self.last_goal_result or None,
                "last_reason": self.last_goal_reason or None,
                "last_elapsed": float(self.last_goal_elapsed),
            },
            "health": {
                "map_age": map_age,
                "map_stamp_age": map_stamp_age,
                "map_receive_age": map_receive_age,
                "tf_age": tf_age,
                "tf_stamp_age": tf_stamp_age,
                "tf_receive_age": tf_receive_age,
                "failure_count": int(self.health_failure_count),
                "reason": self.health_reason or None,
            },
            "recovery": {
                "reason": self.recovery_reason or None,
                "remaining": float(recovery_remaining),
            },
            "counters": {
                "goals_sent": int(self.total_goals_sent),
                "goal_successes": int(self.total_goal_successes),
                "goal_failures": int(self.total_goal_failures),
                "consecutive_goal_failures": int(
                    self.consecutive_goal_failures
                ),
                "blacklist_size": len(self.blacklist),
                "global_empty_cycles": int(self.global_empty_cycles),
            },
            "map": {
                "width": int(self.map_width),
                "height": int(self.map_height),
                "resolution": float(self.resolution),
            },
            "robot_pose": list(self.last_robot_pose)
            if self.last_robot_pose is not None
            else None,
        }
        self.runtime_status_pub.publish(
            String(
                data=json.dumps(
                    payload,
                    ensure_ascii=False,
                    sort_keys=True,
                )
            )
        )

    def set_exploration_state(self, state: str) -> None:
        if state != self.exploration_state:
            previous = self.exploration_state
            self.exploration_state = state
            self.state_changed_at = rospy.Time.now()
            rospy.loginfo(
                "[graph_nbv] state %s -> %s",
                previous,
                state,
            )
        self.publish_runtime_status()

    def finish_exploration(self, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        self.aborted = False
        self.stop_initial_motion()
        try:
            self.client.cancel_goal()
        except Exception:
            pass
        self.finished_pub.publish(Bool(data=True))
        self.mode_pub.publish(String(data="FINISHED"))
        self.gate_state_pub.publish(String(data="FINISHED"))
        self.set_exploration_state("FINISHED")
        self.publish_status(reason)
        rospy.loginfo("[graph_nbv] exploration finished: %s", reason)

    def abort_safely(self, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        self.aborted = True
        self.last_error = reason
        self.stop_initial_motion()
        try:
            self.client.cancel_goal()
        except Exception:
            pass
        self.clear_current_goal()
        self.finished_pub.publish(Bool(data=False))
        self.mode_pub.publish(String(data="ABORTED_SAFE"))
        self.gate_state_pub.publish(String(data="ABORTED_SAFE"))
        self.set_exploration_state("ABORTED_SAFE")
        self.publish_status("ABORTED_SAFE_" + reason)
        rospy.logerr("[graph_nbv] safe abort: %s", reason)

    def runtime_health_reason(self, now: rospy.Time) -> Optional[str]:
        if not pose_is_reasonable(
            self.last_robot_pose,
            self.max_pose_abs,
        ) and self.last_robot_pose is not None:
            return "INVALID_TF_POSE"

        if self.map_freshness_timeout > 0.0:
            map_age = freshness_age(
                self.last_map_stamp.to_sec(),
                self.last_map_received_at.to_sec(),
                now.to_sec(),
            )
            if (
                map_age is not None
                and map_age > self.map_freshness_timeout
            ):
                return "STALE_MAP"

        if self.tf_freshness_timeout > 0.0:
            tf_age = freshness_age(
                self.last_tf_stamp.to_sec(),
                self.last_tf_received_at.to_sec(),
                now.to_sec(),
            )
            if (
                tf_age is not None
                and tf_age > self.tf_freshness_timeout
            ):
                return "STALE_TF"

        return None

    def handle_health_degraded(self, reason: str) -> None:
        now = rospy.Time.now()
        self.health_failure_count += 1
        self.health_reason = reason
        if self.exploration_state != "HEALTH_DEGRADED":
            if self.current_goal_xy() is not None:
                try:
                    self.client.cancel_goal()
                except Exception:
                    pass
                self.clear_current_goal()
            self.recovery_reason = reason
            self.recovery_until = now + rospy.Duration(
                max(0.0, self.goal_recovery_pause)
            )
            self.set_exploration_state("HEALTH_DEGRADED")
            self.publish_status("HEALTH_DEGRADED_" + reason)

        if self.health_failure_count >= self.health_failure_limit:
            self.abort_safely(reason)

    def recover_health(self) -> None:
        if self.health_failure_count == 0 and self.health_reason == "":
            return
        previous_reason = self.health_reason
        self.health_failure_count = 0
        self.health_reason = ""
        self.recovery_reason = ""
        self.recovery_until = rospy.Time(0)
        if self.exploration_state == "HEALTH_DEGRADED":
            self.set_exploration_state("WAITING_FOR_NEXT_GOAL")
            self.publish_status("HEALTH_RECOVERED_" + previous_reason)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def get_tf_pose(
        self,
        parent_frame: str,
        child_frame: str,
    ) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                rospy.Time(0),
                rospy.Duration(0.25),
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                3.0,
                "[graph_nbv] TF %s -> %s unavailable: %s",
                parent_frame,
                child_frame,
                str(exc),
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        now = rospy.Time.now()
        self.last_tf_stamp = transform.header.stamp
        self.last_tf_received_at = now
        _, _, yaw = euler_from_quaternion(
            [rotation.x, rotation.y, rotation.z, rotation.w]
        )
        return translation.x, translation.y, yaw

    def get_odom_pose(
        self,
    ) -> Optional[Tuple[float, float, float]]:
        return self.get_tf_pose(
            self.odom_frame,
            self.odom_base_frame,
        )

    def publish_initial_velocity(
        self,
        speed_x: float,
        speed_y: float,
    ) -> None:
        command = Twist()
        command.linear.x = speed_x
        command.linear.y = speed_y
        self.cmd_vel_pub.publish(command)

    def stop_initial_motion(self) -> None:
        try:
            self.publish_initial_velocity(0.0, 0.0)
        except Exception:
            pass

    def gate_progress(self, x: float, y: float) -> float:
        if self.gate_origin is None or self.gate_forward is None:
            return float("inf")

        return (
            (x - self.gate_origin[0]) * self.gate_forward[0]
            + (y - self.gate_origin[1]) * self.gate_forward[1]
        )

    def gate_allows(self, x: float, y: float) -> bool:
        # Hard forward gate: once the gate origin/direction exists,
        # keep a small tolerance for the first path samples.  TEB can
        # legitimately back up a few cells while turning through the
        # corridor; rejecting those samples makes every otherwise reachable
        # forward goal look unreachable.
        if self.gate_origin is None or self.gate_forward is None:
            return True
        return self.gate_progress(x, y) >= -self.gate_backtrack_margin

    def behind_gate_penalty(self, x: float, y: float) -> float:
        if (
            self.gate_origin is None
            or self.gate_forward is None
            or self.gate_locked
        ):
            return 0.0
        if self.gate_progress(x, y) >= 0.0:
            return 0.0
        return self.behind_gate_penalty_after_unlock

    def unlock_forward_gate(self) -> None:
        if not self.gate_locked:
            return
        self.gate_locked = False
        self.gate_unlock_count += 1
        self.forward_empty_cycles = 0
        self.gate_state_pub.publish(
            String(data="UNLOCKED_AFTER_FORWARD_EXHAUSTION")
        )
        self.mode_pub.publish(String(data="GLOBAL_RELOCATION"))
        self.set_exploration_state("RELOCATING_GLOBAL")
        self.publish_status("FORWARD_REGION_EXHAUSTED_GATE_UNLOCKED")
        rospy.loginfo(
            "[graph_nbv] forward region exhausted for %d stable cycles; "
            "unlocking gate for global relocation",
            self.forward_finish_stable_cycles,
        )

    def initial_obstacle_ahead(
        self,
        robot_pose: Tuple[float, float, float],
    ) -> bool:
        if (
            not self.initial_obstacle_check_enabled
            or self.grid is None
            or self.resolution <= 0.0
        ):
            return False

        yaw = robot_pose[2]
        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)
        left_x = -math.sin(yaw)
        left_y = math.cos(yaw)

        longitudinal_step = max(self.resolution, 0.10)
        lateral_step = max(self.resolution, 0.10)
        distance = 0.30

        while distance <= self.initial_obstacle_stop_distance:
            lateral = -0.5 * self.initial_obstacle_check_width
            while lateral <= 0.5 * self.initial_obstacle_check_width:
                x = (
                    robot_pose[0]
                    + distance * forward_x
                    + lateral * left_x
                )
                y = (
                    robot_pose[1]
                    + distance * forward_y
                    + lateral * left_y
                )
                cell = self.world_to_grid(x, y)
                if cell is not None:
                    row, col = cell
                    if self.grid[row, col] >= self.occupied_threshold:
                        return True
                lateral += lateral_step
            distance += longitudinal_step

        return False

    def finish_initial_forward(
        self,
        robot_pose: Tuple[float, float, float],
    ) -> None:
        assert self.initial_start_map is not None

        dx = robot_pose[0] - self.initial_start_map[0]
        dy = robot_pose[1] - self.initial_start_map[1]
        distance = math.hypot(dx, dy)

        if distance >= 0.50:
            forward_x = dx / distance
            forward_y = dy / distance
        else:
            forward_x = math.cos(robot_pose[2])
            forward_y = math.sin(robot_pose[2])

        self.gate_origin = (robot_pose[0], robot_pose[1])
        self.gate_forward = (forward_x, forward_y)
        self.gate_locked = True
        self.initial_complete = True
        self.initial_state = "COMPLETE"
        self.forward_empty_cycles = 0
        self.stop_initial_motion()

        self.gate_state_pub.publish(String(data="FORWARD_REGION_LOCKED"))
        self.mode_pub.publish(String(data="FORWARD_REGION_LOCKED"))
        self.set_exploration_state("EXPLORING_LOCAL")
        self.publish_status("INITIAL_FORWARD_REACHED")

        rospy.loginfo(
            "[graph_nbv] gate initialized origin=(%.3f, %.3f) "
            "forward=(%.5f, %.5f)",
            self.gate_origin[0],
            self.gate_origin[1],
            self.gate_forward[0],
            self.gate_forward[1],
        )

        if self.initial_forward_only:
            self.initial_paused = True
            self.publish_status("INITIAL_FORWARD_ONLY_COMPLETE")
            self.mode_pub.publish(String(data="INITIAL_FORWARD_COMPLETE"))
            rospy.loginfo(
                "[graph_nbv] initial-forward-only test complete; "
                "exploration remains stopped"
            )

    def initialize_manual_gate(
        self,
        robot_pose: Tuple[float, float, float],
    ) -> None:
        now = rospy.Time.now()
        if self.manual_gate_wait_start.to_sec() <= 0.0:
            self.manual_gate_wait_start = now

        if (
            self.manual_gate_wait_timeout > 0.0
            and now.to_sec() > 0.0
            and (
                now - self.manual_gate_wait_start
            ).to_sec() > self.manual_gate_wait_timeout
        ):
            self.abort_safely("MANUAL_GATE_UNSTABLE")
            return

        if self.manual_gate_samples and not pose_is_close(
            self.manual_gate_samples[-1],
            robot_pose,
            self.manual_gate_stability_position,
            self.manual_gate_stability_angle,
        ):
            self.manual_gate_samples = [robot_pose]
        else:
            self.manual_gate_samples.append(robot_pose)

        self.manual_gate_samples = self.manual_gate_samples[
            -self.manual_gate_stability_samples:
        ]
        self.set_exploration_state("WAITING_MANUAL_GATE")
        self.publish_status(
            "WAITING_MANUAL_GATE_STABILITY_%d_OF_%d"
            % (
                len(self.manual_gate_samples),
                self.manual_gate_stability_samples,
            )
        )
        if (
            len(self.manual_gate_samples)
            < self.manual_gate_stability_samples
            or not pose_samples_stable(
                self.manual_gate_samples,
                self.manual_gate_stability_position,
                self.manual_gate_stability_angle,
            )
        ):
            return

        mean_x = sum(pose[0] for pose in self.manual_gate_samples) / len(
            self.manual_gate_samples
        )
        mean_y = sum(pose[1] for pose in self.manual_gate_samples) / len(
            self.manual_gate_samples
        )
        mean_cos = sum(
            math.cos(pose[2]) for pose in self.manual_gate_samples
        ) / len(self.manual_gate_samples)
        mean_sin = sum(
            math.sin(pose[2]) for pose in self.manual_gate_samples
        ) / len(self.manual_gate_samples)
        stable_yaw = math.atan2(mean_sin, mean_cos)

        if self.manual_gate_use_robot_heading:
            forward_x = math.cos(stable_yaw)
            forward_y = math.sin(stable_yaw)
        else:
            norm = math.hypot(
                self.manual_gate_forward_x,
                self.manual_gate_forward_y,
            )
            if norm < 1e-6:
                self.publish_status("MANUAL_GATE_DIRECTION_INVALID")
                rospy.logerr(
                    "[graph_nbv] manual gate direction vector is zero"
                )
                return
            forward_x = self.manual_gate_forward_x / norm
            forward_y = self.manual_gate_forward_y / norm

        self.gate_origin = (mean_x, mean_y)
        self.gate_forward = (forward_x, forward_y)
        self.gate_locked = True
        self.initial_complete = True
        self.initial_state = "COMPLETE"
        self.forward_empty_cycles = 0
        self.stop_initial_motion()

        self.gate_state_pub.publish(
            String(data="FORWARD_REGION_LOCKED")
        )
        self.mode_pub.publish(
            String(data="FORWARD_REGION_LOCKED")
        )
        self.set_exploration_state("EXPLORING_LOCAL")
        self.publish_status("MANUAL_GATE_INITIALIZED")

        rospy.loginfo(
            "[graph_nbv] manual gate initialized "
            "origin=(%.3f, %.3f) forward=(%.5f, %.5f) yaw=%.3f",
            self.gate_origin[0],
            self.gate_origin[1],
            self.gate_forward[0],
            self.gate_forward[1],
            stable_yaw,
        )

    def handle_initial_forward(
        self,
        robot_pose: Tuple[float, float, float],
    ) -> None:
        odom_pose = self.get_odom_pose()
        if odom_pose is None:
            self.stop_initial_motion()
            self.publish_status("WAITING_FOR_ODOM_TF")
            return

        now = rospy.Time.now()

        if self.initial_state == "WAITING_INITIAL_POSE":
            self.initial_start_odom = odom_pose
            self.initial_start_map = robot_pose
            self.initial_start_time = now
            self.initial_state = "FAST"
            self.mode_pub.publish(String(data="INITIAL_FORWARD_FAST"))
            self.publish_status("INITIAL_FORWARD_FAST")
            rospy.loginfo(
                "[graph_nbv] initial poses recorded "
                "odom=(%.3f, %.3f) map=(%.3f, %.3f)",
                odom_pose[0],
                odom_pose[1],
                robot_pose[0],
                robot_pose[1],
            )
            return

        if self.initial_state in ("FAST", "SLOW"):
            assert self.initial_start_odom is not None
            travelled = math.hypot(
                odom_pose[0] - self.initial_start_odom[0],
                odom_pose[1] - self.initial_start_odom[1],
            )

            elapsed = (now - self.initial_start_time).to_sec()
            if elapsed > self.initial_forward_timeout:
                self.stop_initial_motion()
                self.initial_state = "ERROR"
                self.initial_paused = True
                self.publish_status("INITIAL_FORWARD_TIMEOUT")
                self.mode_pub.publish(String(data="INITIAL_FORWARD_ERROR"))
                rospy.logerr(
                    "[graph_nbv] initial forward timeout "
                    "travelled=%.3f m",
                    travelled,
                )
                return

            if self.initial_obstacle_ahead(robot_pose):
                self.stop_initial_motion()
                self.initial_state = "ERROR"
                self.initial_paused = True
                self.publish_status("INITIAL_FORWARD_BLOCKED")
                self.mode_pub.publish(String(data="INITIAL_FORWARD_ERROR"))
                rospy.logerr(
                    "[graph_nbv] occupied cell detected ahead "
                    "during initial forward"
                )
                return

            if travelled >= self.initial_forward_stop_distance:
                self.stop_initial_motion()
                self.initial_state = "SETTLE"
                self.initial_settle_start = now
                self.mode_pub.publish(String(data="INITIAL_FORWARD_SETTLE"))
                self.publish_status("INITIAL_FORWARD_SETTLE")
                rospy.loginfo(
                    "[graph_nbv] initial stop distance reached: %.3f m",
                    travelled,
                )
                return

            if travelled >= self.initial_forward_slowdown_distance:
                self.initial_state = "SLOW"
                self.publish_initial_velocity(
                    self.initial_forward_slow_speed_x,
                    self.initial_forward_slow_speed_y,
                )
                self.mode_pub.publish(String(data="INITIAL_FORWARD_SLOW"))
                self.publish_status("INITIAL_FORWARD_SLOW")
            else:
                self.initial_state = "FAST"
                self.publish_initial_velocity(
                    self.initial_forward_speed_x,
                    self.initial_forward_speed_y,
                )
                self.mode_pub.publish(String(data="INITIAL_FORWARD_FAST"))
                self.publish_status("INITIAL_FORWARD_FAST")
            return

        if self.initial_state == "SETTLE":
            self.stop_initial_motion()
            if (
                now - self.initial_settle_start
            ).to_sec() >= self.initial_settle_time:
                self.finish_initial_forward(robot_pose)
            return

    def map_callback(self, msg: OccupancyGrid) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)

        data = np.asarray(msg.data, dtype=np.int16)
        if (
            width <= 0
            or height <= 0
            or resolution <= 0.0
            or data.size != width * height
        ):
            return

        orientation = msg.info.origin.orientation
        _, _, origin_yaw = euler_from_quaternion(
            [
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ]
        )

        geometry_changed = (
            self.covered is None
            or width != self.map_width
            or height != self.map_height
            or abs(resolution - self.resolution) > 1e-9
        )

        self.grid = data.reshape((height, width))
        self.map_width = width
        self.map_height = height
        self.resolution = resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.origin_yaw = origin_yaw
        self.last_map_stamp = msg.header.stamp
        self.last_map_received_at = rospy.Time.now()

        if geometry_changed:
            self.covered = np.zeros(
                (height, width), dtype=np.uint8
            )
            self.global_empty_cycles = 0
            rospy.loginfo(
                "[graph_nbv] map geometry %dx%d res=%.3f",
                width,
                height,
                resolution,
            )
        self.publish_runtime_status()

    def get_robot_pose(
        self,
    ) -> Optional[Tuple[float, float, float]]:
        return self.get_tf_pose(
            self.global_frame,
            self.base_frame,
        )

    def world_to_grid(
        self,
        x: float,
        y: float,
    ) -> Optional[Tuple[int, int]]:
        dx = x - self.origin_x
        dy = y - self.origin_y

        c = math.cos(-self.origin_yaw)
        s = math.sin(-self.origin_yaw)
        local_x = c * dx - s * dy
        local_y = s * dx + c * dy

        col = int(math.floor(local_x / self.resolution))
        row = int(math.floor(local_y / self.resolution))

        if (
            row < 0
            or row >= self.map_height
            or col < 0
            or col >= self.map_width
        ):
            return None
        return row, col

    def grid_to_world(
        self,
        row: int,
        col: int,
    ) -> Tuple[float, float]:
        local_x = (col + 0.5) * self.resolution
        local_y = (row + 0.5) * self.resolution

        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        x = self.origin_x + c * local_x - s * local_y
        y = self.origin_y + s * local_x + c * local_y
        return x, y

    def mark_coverage(
        self,
        robot_pose: Tuple[float, float, float],
    ) -> None:
        if self.covered is None:
            return

        cell = self.world_to_grid(robot_pose[0], robot_pose[1])
        if cell is None:
            return

        row, col = cell
        radius_cells = max(
            1,
            int(round(self.coverage_radius / self.resolution)),
        )
        cv2.circle(
            self.covered,
            (col, row),
            radius_cells,
            1,
            thickness=-1,
        )

    def build_layers(
        self,
    ) -> Optional[Dict[str, np.ndarray]]:
        if self.grid is None or self.covered is None:
            return None

        known_free = (
            (self.grid >= 0)
            & (self.grid < self.occupied_threshold)
        ).astype(np.uint8)
        occupied = (
            self.grid >= self.occupied_threshold
        ).astype(np.uint8)
        unknown = (self.grid < 0).astype(np.uint8)

        clearance_cells = cv2.distanceTransform(
            (occupied == 0).astype(np.uint8),
            cv2.DIST_L2,
            5,
        )
        clearance = clearance_cells * self.resolution

        safe = (
            (known_free > 0)
            & (clearance >= self.min_candidate_clearance)
        )

        cross_kernel = np.asarray(
            [
                [0, 1, 0],
                [1, 1, 1],
                [0, 1, 0],
            ],
            dtype=np.uint8,
        )
        unknown_touch = cv2.dilate(
            unknown,
            cross_kernel,
            iterations=1,
        )

        wall_guard = cv2.dilate(
            occupied,
            np.ones((3, 3), dtype=np.uint8),
            iterations=1,
        )

        frontier = (
            (known_free > 0)
            & (unknown_touch > 0)
            & (wall_guard == 0)
        ).astype(np.uint8)

        dilation_cells = max(
            0,
            int(math.ceil(
                self.visibility_wall_dilation / self.resolution
            )),
        )
        if dilation_cells > 0:
            size = 2 * dilation_cells + 1
            visibility_obstacles = cv2.dilate(
                occupied,
                np.ones((size, size), dtype=np.uint8),
                iterations=1,
            )
        else:
            visibility_obstacles = occupied.copy()

        coverage_distance_cells = cv2.distanceTransform(
            (self.covered == 0).astype(np.uint8),
            cv2.DIST_L2,
            5,
        )
        coverage_distance = (
            coverage_distance_cells * self.resolution
        )

        return {
            "known_free": known_free,
            "occupied": occupied,
            "unknown": unknown,
            "clearance": clearance,
            "safe": safe,
            "frontier": frontier,
            "visibility_obstacles": visibility_obstacles,
            "coverage_distance": coverage_distance,
        }

    def visible_gain(
        self,
        x: float,
        y: float,
        layers: Dict[str, np.ndarray],
    ) -> Tuple[float, int]:
        visible_unknown: Set[Tuple[int, int]] = set()
        visible_frontier: Set[Tuple[int, int]] = set()

        ray_count = max(16, self.visibility_ray_count)
        max_steps = max(
            1,
            int(math.ceil(
                self.information_radius / self.visibility_ray_step
            )),
        )
        unknown_depth_steps = max(
            1,
            int(math.ceil(
                self.visibility_unknown_depth
                / self.visibility_ray_step
            )),
        )

        for ray_index in range(ray_count):
            angle = (
                2.0 * math.pi * ray_index / ray_count
            )
            dx = math.cos(angle)
            dy = math.sin(angle)
            unknown_run = 0
            previous_cell = None

            for step_index in range(1, max_steps + 1):
                distance = (
                    step_index * self.visibility_ray_step
                )
                cell = self.world_to_grid(
                    x + dx * distance,
                    y + dy * distance,
                )
                if cell is None:
                    break
                if cell == previous_cell:
                    continue
                previous_cell = cell

                row, col = cell

                if layers["visibility_obstacles"][row, col] > 0:
                    break

                if layers["frontier"][row, col] > 0:
                    visible_frontier.add((row, col))

                if layers["unknown"][row, col] > 0:
                    visible_unknown.add((row, col))
                    unknown_run += 1
                    if unknown_run >= unknown_depth_steps:
                        break
                else:
                    unknown_run = 0

        gain_m2 = (
            len(visible_unknown)
            * self.resolution
            * self.resolution
        )
        return gain_m2, len(visible_frontier)

    def sample_local_nodes(
        self,
        layers: Dict[str, np.ndarray],
        robot_pose: Tuple[float, float, float],
    ) -> List[GraphNode]:
        robot_cell = self.world_to_grid(
            robot_pose[0], robot_pose[1]
        )
        if robot_cell is None:
            return []

        center_row, center_col = robot_cell
        radius_cells = int(
            math.ceil(
                self.local_sampling_radius / self.resolution
            )
        )
        spacing_cells = max(
            1,
            int(round(
                self.candidate_spacing / self.resolution
            )),
        )

        row_min = max(0, center_row - radius_cells)
        row_max = min(
            self.map_height - 1,
            center_row + radius_cells,
        )
        col_min = max(0, center_col - radius_cells)
        col_max = min(
            self.map_width - 1,
            center_col + radius_cells,
        )

        nodes: List[GraphNode] = []

        for row in range(row_min, row_max + 1, spacing_cells):
            for col in range(col_min, col_max + 1, spacing_cells):
                if not layers["safe"][row, col]:
                    continue

                x, y = self.grid_to_world(row, col)
                if not self.gate_allows(x, y):
                    continue

                distance = math.hypot(
                    x - robot_pose[0],
                    y - robot_pose[1],
                )
                if distance > self.local_sampling_radius:
                    continue

                # Every safe node is retained as a transit node.
                gain, frontier_cells = self.visible_gain(
                    x, y, layers
                )
                coverage_distance = float(
                    layers["coverage_distance"][row, col]
                )
                revisit = max(
                    0.0,
                    self.coverage_radius - coverage_distance,
                )

                node = GraphNode(
                    index=len(nodes),
                    row=row,
                    col=col,
                    x=x,
                    y=y,
                    clearance=float(
                        layers["clearance"][row, col]
                    ),
                    visible_unknown_gain=gain,
                    visible_frontier_cells=frontier_cells,
                    revisit=revisit,
                    is_target=(
                        gain >= self.min_visible_unknown_gain
                        and frontier_cells
                        >= self.min_visible_frontier_cells
                    ),
                )
                nodes.append(node)

        # Preserve graph connectivity: nearest nodes are retained first.
        nodes.sort(
            key=lambda node: (
                math.hypot(
                    node.x - robot_pose[0],
                    node.y - robot_pose[1],
                ),
                -node.clearance,
            )
        )
        nodes = nodes[: self.max_local_nodes]

        for index, node in enumerate(nodes):
            node.index = index

        return nodes

    def edge_is_free(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        layers: Dict[str, np.ndarray],
    ) -> bool:
        distance = math.hypot(x2 - x1, y2 - y1)
        count = max(
            2,
            int(math.ceil(
                distance / self.edge_collision_step
            )),
        )

        for index in range(count + 1):
            ratio = index / count
            point_x = x1 + ratio * (x2 - x1)
            point_y = y1 + ratio * (y2 - y1)
            if not self.gate_allows(point_x, point_y):
                return False

            cell = self.world_to_grid(point_x, point_y)
            if cell is None:
                return False
            row, col = cell
            if not layers["safe"][row, col]:
                return False

        return True

    def build_local_graph(
        self,
        nodes: List[GraphNode],
        robot_pose: Tuple[float, float, float],
        layers: Dict[str, np.ndarray],
    ) -> Tuple[
        Dict[int, List[Tuple[int, float]]],
        List[Tuple[int, int]],
    ]:
        robot_index = len(nodes)
        adjacency: Dict[int, List[Tuple[int, float]]] = {
            index: []
            for index in range(robot_index + 1)
        }
        edges: List[Tuple[int, int]] = []

        positions = [(node.x, node.y) for node in nodes]
        positions.append((robot_pose[0], robot_pose[1]))

        for first in range(len(positions)):
            x1, y1 = positions[first]
            for second in range(first + 1, len(positions)):
                x2, y2 = positions[second]
                distance = math.hypot(x2 - x1, y2 - y1)

                if distance > self.edge_max_length:
                    continue
                if not self.edge_is_free(
                    x1, y1, x2, y2, layers
                ):
                    continue

                adjacency[first].append((second, distance))
                adjacency[second].append((first, distance))
                edges.append((first, second))

        return adjacency, edges

    @staticmethod
    def dijkstra(
        adjacency: Dict[int, List[Tuple[int, float]]],
        start: int,
    ) -> Dict[int, float]:
        distances = {
            node: float("inf")
            for node in adjacency
        }
        distances[start] = 0.0
        queue: List[Tuple[float, int]] = [(0.0, start)]

        while queue:
            distance, node = heapq.heappop(queue)
            if distance > distances[node]:
                continue

            for neighbor, cost in adjacency[node]:
                new_distance = distance + cost
                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    heapq.heappush(
                        queue,
                        (new_distance, neighbor),
                    )

        return distances

    def is_blacklisted(self, x: float, y: float) -> bool:
        return any(
            math.hypot(x - bx, y - by)
            < self.blacklist_radius
            for bx, by in self.blacklist
        )

    def add_blacklist(self, x: float, y: float) -> None:
        self.blacklist.append((x, y))
        if len(self.blacklist) > 200:
            self.blacklist = self.blacklist[-200:]

    def begin_goal(self, kind: str) -> None:
        self.goal_sequence += 1
        self.current_goal_id = "%s-%04d" % (
            kind.lower(),
            self.goal_sequence,
        )
        self.last_goal_id = self.current_goal_id
        self.last_goal_result = "ACTIVE"
        self.last_goal_reason = ""
        self.last_goal_elapsed = 0.0
        self.total_goals_sent += 1
        self.current_goal_start = rospy.Time.now()
        self.last_progress_time = self.current_goal_start
        self.last_goal_distance = float("inf")

    def complete_current_goal(self, result: str, reason: str) -> str:
        kind = self.current_goal_kind
        now = rospy.Time.now()
        elapsed = 0.0
        if self.current_goal_start.to_sec() > 0.0 and now.to_sec() > 0.0:
            elapsed = max(
                0.0,
                (now - self.current_goal_start).to_sec(),
            )
        self.last_goal_id = self.current_goal_id
        self.last_goal_result = result
        self.last_goal_reason = reason
        self.last_goal_elapsed = elapsed
        self.total_goal_successes += 1
        self.consecutive_goal_failures = 0
        self.clear_current_goal()
        self.recovery_until = rospy.Time(0)
        self.recovery_reason = ""
        self.set_exploration_state("WAITING_FOR_NEXT_GOAL")
        return kind

    def fail_current_goal(self, reason: str) -> str:
        kind = self.current_goal_kind
        goal_xy = self.current_goal_xy()
        now = rospy.Time.now()
        elapsed = 0.0
        if self.current_goal_start.to_sec() > 0.0 and now.to_sec() > 0.0:
            elapsed = max(
                0.0,
                (now - self.current_goal_start).to_sec(),
            )

        try:
            self.client.cancel_goal()
        except Exception:
            pass

        if goal_xy is not None:
            self.add_blacklist(goal_xy[0], goal_xy[1])
        self.last_goal_id = self.current_goal_id
        self.last_goal_result = "FAILED"
        self.last_goal_reason = reason
        self.last_goal_elapsed = elapsed
        self.total_goal_failures += 1
        self.consecutive_goal_failures += 1
        self.clear_current_goal()
        self.recovery_reason = reason
        self.recovery_until = now + rospy.Duration(
            max(0.0, self.goal_recovery_pause)
        )
        self.set_exploration_state("GOAL_RECOVERY")
        self.publish_status("GOAL_RECOVERY_%s" % reason)

        if failure_budget_exhausted(
            self.consecutive_goal_failures,
            self.max_consecutive_goal_failures,
        ):
            self.abort_safely("GOAL_FAILURE_BUDGET")
        return kind

    def select_local_target(
        self,
        nodes: List[GraphNode],
        distances: Dict[int, float],
        robot_pose: Tuple[float, float, float],
    ) -> Optional[GraphNode]:
        best = None

        for node in nodes:
            node.path_cost = distances.get(
                node.index, float("inf")
            )

            if not node.is_target:
                continue
            if not math.isfinite(node.path_cost):
                continue
            if self.is_blacklisted(node.x, node.y):
                continue

            distance = math.hypot(
                node.x - robot_pose[0],
                node.y - robot_pose[1],
            )
            if (
                distance < self.goal_min_distance
                or distance > self.goal_max_distance
            ):
                continue

            wall_penalty = max(
                0.0,
                self.preferred_clearance - node.clearance,
            )

            node.utility = (
                self.weight_unknown_gain
                * node.visible_unknown_gain
                + self.weight_frontier_gain
                * node.visible_frontier_cells
                + self.weight_clearance
                * min(node.clearance, 1.50)
                - self.weight_path_cost
                * node.path_cost
                - self.weight_revisit
                * node.revisit
                - self.weight_wall_proximity
                * wall_penalty
                - self.behind_gate_penalty(node.x, node.y)
            )

            if best is None or node.utility > best.utility:
                best = node

        return best

    def make_pose(
        self,
        x: float,
        y: float,
        yaw: float,
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y

        quaternion = quaternion_from_euler(
            0.0, 0.0, yaw
        )
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose

    @staticmethod
    def path_length(poses: List[PoseStamped]) -> float:
        total = 0.0
        for first, second in zip(poses[:-1], poses[1:]):
            total += math.hypot(
                second.pose.position.x
                - first.pose.position.x,
                second.pose.position.y
                - first.pose.position.y,
            )
        return total

    def request_global_plan(
        self,
        robot_pose: Tuple[float, float, float],
        x: float,
        y: float,
        yaw: float,
    ) -> Optional[float]:
        if not self.make_plan_available:
            try:
                rospy.wait_for_service(
                    self.make_plan_service,
                    timeout=0.5,
                )
                self.make_plan_available = True
            except rospy.ROSException:
                return None

        start = self.make_pose(
            robot_pose[0],
            robot_pose[1],
            robot_pose[2],
        )
        goal = self.make_pose(x, y, yaw)

        try:
            response = self.make_plan_client(
                start=start,
                goal=goal,
                tolerance=self.global_plan_tolerance,
            )
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(
                3.0,
                "[graph_nbv] make_plan failed: %s",
                str(exc),
            )
            self.make_plan_available = False
            return None

        if len(response.plan.poses) < 2:
            return None

        if self.gate_locked:
            for pose in response.plan.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                if not self.gate_allows(x, y):
                    return None

        return self.path_length(response.plan.poses)

    def global_frontier_targets(
        self,
        layers: Dict[str, np.ndarray],
        robot_pose: Tuple[float, float, float],
    ) -> Tuple[
        Optional[GlobalTarget],
        List[Tuple[float, float]],
    ]:
        frontier = layers["frontier"].astype(np.uint8)

        component_count, labels, stats, centroids = (
            cv2.connectedComponentsWithStats(
                frontier,
                connectivity=8,
            )
        )

        component_ids = [
            component_id
            for component_id in range(1, component_count)
            if int(
                stats[component_id, cv2.CC_STAT_AREA]
            ) >= self.global_frontier_min_cells
        ]
        component_ids.sort(
            key=lambda component_id: int(
                stats[component_id, cv2.CC_STAT_AREA]
            ),
            reverse=True,
        )
        component_ids = component_ids[
            : self.global_frontier_max_components
        ]

        raw_candidates = []
        frontier_points: List[Tuple[float, float]] = []

        for component_id in component_ids:
            component = (
                labels == component_id
            ).astype(np.uint8)
            area = int(
                stats[component_id, cv2.CC_STAT_AREA]
            )

            if self.gate_locked:
                component_cells = np.argwhere(component > 0)
                gate_visible = False
                stride = max(1, len(component_cells) // 40)
                for row, col in component_cells[::stride]:
                    cell_x, cell_y = self.grid_to_world(
                        int(row), int(col)
                    )
                    if self.gate_allows(cell_x, cell_y):
                        gate_visible = True
                        break
                if not gate_visible:
                    continue

            centroid_col = float(
                centroids[component_id][0]
            )
            centroid_row = float(
                centroids[component_id][1]
            )
            centroid_x, centroid_y = self.grid_to_world(
                int(round(centroid_row)),
                int(round(centroid_col)),
            )
            frontier_points.append(
                (centroid_x, centroid_y)
            )

            distance_cells = cv2.distanceTransform(
                (component == 0).astype(np.uint8),
                cv2.DIST_L2,
                5,
            )
            distance_to_frontier = (
                distance_cells * self.resolution
            )

            approach_mask = (
                layers["safe"]
                & (
                    distance_to_frontier
                    >= self.global_approach_min_distance
                )
                & (
                    distance_to_frontier
                    <= self.global_approach_max_distance
                )
            )

            indices = np.argwhere(approach_mask)
            if indices.size == 0:
                continue

            approach_scores = (
                2.0 * layers["clearance"][approach_mask]
                - 0.30
                * distance_to_frontier[approach_mask]
                + 0.15
                * layers["coverage_distance"][approach_mask]
            )
            order = np.argsort(
                approach_scores
            )[::-1]

            selected_count = 0
            for order_index in order:
                row, col = indices[int(order_index)]
                x, y = self.grid_to_world(
                    int(row), int(col)
                )

                if not self.gate_allows(x, y):
                    continue
                if self.is_blacklisted(x, y):
                    continue

                gain, visible_frontier = self.visible_gain(
                    x, y, layers
                )
                if (
                    visible_frontier
                    < self.min_visible_frontier_cells
                ):
                    continue

                yaw = math.atan2(
                    centroid_y - y,
                    centroid_x - x,
                )

                raw_candidates.append(
                    (
                        component_id,
                        area,
                        x,
                        y,
                        yaw,
                        gain,
                        visible_frontier,
                        float(
                            layers["clearance"][row, col]
                        ),
                        float(
                            layers["coverage_distance"][
                                row, col
                            ]
                        ),
                    )
                )
                selected_count += 1

                if (
                    selected_count
                    >= self.global_candidates_per_component
                ):
                    break

        raw_candidates.sort(
            key=lambda item: (
                item[5],
                item[1],
                item[7],
            ),
            reverse=True,
        )
        raw_candidates = raw_candidates[
            : self.global_max_plan_checks
        ]

        best: Optional[GlobalTarget] = None

        for (
            component_id,
            area,
            x,
            y,
            yaw,
            gain,
            visible_frontier,
            clearance,
            coverage_distance,
        ) in raw_candidates:
            straight_distance = math.hypot(
                x - robot_pose[0],
                y - robot_pose[1],
            )
            if (
                straight_distance < self.goal_min_distance
                or straight_distance > self.goal_max_distance
            ):
                continue

            plan_cost = self.request_global_plan(
                robot_pose,
                x,
                y,
                yaw,
            )
            if plan_cost is None:
                continue

            revisit = max(
                0.0,
                self.coverage_radius - coverage_distance,
            )
            wall_penalty = max(
                0.0,
                self.preferred_clearance - clearance,
            )

            utility = (
                self.weight_unknown_gain * gain
                + self.weight_frontier_gain
                * visible_frontier
                + self.weight_global_frontier_size
                * area
                + self.weight_clearance
                * min(clearance, 1.50)
                - self.weight_global_path_cost
                * plan_cost
                - self.weight_global_revisit
                * revisit
                - self.weight_wall_proximity
                * wall_penalty
                - self.behind_gate_penalty(x, y)
            )

            target = GlobalTarget(
                x=x,
                y=y,
                yaw=yaw,
                utility=utility,
                path_cost=plan_cost,
                visible_unknown_gain=gain,
                frontier_cells=area,
                component_id=component_id,
            )

            if (
                best is None
                or target.utility > best.utility
            ):
                best = target

        return best, frontier_points

    def send_local_goal(
        self,
        node: GraphNode,
        robot_pose: Tuple[float, float, float],
    ) -> None:
        yaw = math.atan2(
            node.y - robot_pose[1],
            node.x - robot_pose[0],
        )

        self.last_local_selected = node
        self.last_global_selected = None

        rospy.loginfo(
            "[graph_nbv] LOCAL target x=%.2f y=%.2f "
            "U=%.2f gain=%.2f frontier=%d path=%.2f",
            node.x,
            node.y,
            node.utility,
            node.visible_unknown_gain,
            node.visible_frontier_cells,
            node.path_cost,
        )

        if self.dry_run:
            self.mode_pub.publish(String(data="LOCAL_NBV"))
            self.publish_status("DRY_RUN_LOCAL_SELECTED")
            return

        goal = MoveBaseGoal()
        goal.target_pose = self.make_pose(
            node.x, node.y, yaw
        )
        self.client.send_goal(goal)

        self.current_local_goal = node
        self.current_global_goal = None
        self.current_goal_kind = "LOCAL"
        self.begin_goal("LOCAL")

        self.mode_pub.publish(String(data="LOCAL_NBV"))
        self.set_exploration_state("EXPLORING_LOCAL")
        self.publish_status("NAVIGATING_LOCAL_VIEWPOINT")

    def send_global_goal(
        self,
        target: GlobalTarget,
    ) -> None:
        self.last_global_selected = target
        self.last_local_selected = None

        rospy.loginfo(
            "[graph_nbv] GLOBAL relocation x=%.2f y=%.2f "
            "U=%.2f gain=%.2f frontier_component=%d "
            "path=%.2f",
            target.x,
            target.y,
            target.utility,
            target.visible_unknown_gain,
            target.component_id,
            target.path_cost,
        )

        if self.dry_run:
            self.mode_pub.publish(
                String(data="GLOBAL_RELOCATION")
            )
            self.publish_status(
                "DRY_RUN_GLOBAL_RELOCATION_SELECTED"
            )
            return

        goal = MoveBaseGoal()
        goal.target_pose = self.make_pose(
            target.x,
            target.y,
            target.yaw,
        )
        self.client.send_goal(goal)

        self.current_global_goal = target
        self.current_local_goal = None
        self.current_goal_kind = "GLOBAL"
        self.begin_goal("GLOBAL")

        self.mode_pub.publish(
            String(data="GLOBAL_RELOCATION")
        )
        self.set_exploration_state("RELOCATING_GLOBAL")
        self.publish_status(
            "NAVIGATING_GLOBAL_RELOCATION"
        )

    def current_goal_xy(
        self,
    ) -> Optional[Tuple[float, float]]:
        if (
            self.current_goal_kind == "LOCAL"
            and self.current_local_goal is not None
        ):
            return (
                self.current_local_goal.x,
                self.current_local_goal.y,
            )

        if (
            self.current_goal_kind == "GLOBAL"
            and self.current_global_goal is not None
        ):
            return (
                self.current_global_goal.x,
                self.current_global_goal.y,
            )

        return None

    def clear_current_goal(self) -> None:
        self.current_local_goal = None
        self.current_global_goal = None
        self.current_goal_kind = ""
        self.current_goal_id = ""
        self.current_goal_start = rospy.Time(0)
        self.last_progress_time = rospy.Time(0)
        self.last_goal_distance = float("inf")

    def handle_goal(
        self,
        robot_pose: Tuple[float, float, float],
    ) -> None:
        goal_xy = self.current_goal_xy()
        if goal_xy is None:
            return

        state = self.client.get_state()
        now = rospy.Time.now()

        if state in (
            GoalStatus.PENDING,
            GoalStatus.ACTIVE,
        ):
            distance = math.hypot(
                goal_xy[0] - robot_pose[0],
                goal_xy[1] - robot_pose[1],
            )

            # Exploration viewpoints do not require centimeter-level final
            # alignment.  Accept the goal once the robot enters the configured
            # radius, cancel TEB's remaining fine adjustment, and continue NBV.
            if distance <= self.goal_reached_radius:
                completed_kind = self.current_goal_kind
                rospy.loginfo(
                    "[graph_nbv] %s goal accepted near target "
                    "distance=%.3f radius=%.3f",
                    completed_kind,
                    distance,
                    self.goal_reached_radius,
                )
                self.client.cancel_goal()
                self.complete_current_goal(
                    "REACHED_RADIUS",
                    "distance_within_goal_radius",
                )
                self.global_empty_cycles = 0

                if completed_kind == "GLOBAL":
                    self.mode_pub.publish(
                        String(data="LOCAL_NBV")
                    )
                    self.publish_status(
                        "GLOBAL_RELOCATION_REACHED"
                    )
                else:
                    self.publish_status(
                        "LOCAL_VIEWPOINT_REACHED"
                    )
                return

            if (
                self.last_goal_distance - distance
                >= self.progress_epsilon
            ):
                self.last_goal_distance = distance
                self.last_progress_time = now

            total_time = (
                now - self.current_goal_start
            ).to_sec()
            no_progress_time = (
                now - self.last_progress_time
            ).to_sec()

            if (
                total_time > self.goal_timeout
                or no_progress_time > self.no_progress_timeout
            ):
                rospy.logwarn(
                    "[graph_nbv] %s goal timeout "
                    "distance=%.2f total=%.1f no_progress=%.1f",
                    self.current_goal_kind,
                    distance,
                    total_time,
                    no_progress_time,
                )
                self.fail_current_goal("TIMEOUT")
            return

        if state == GoalStatus.SUCCEEDED:
            completed_kind = self.complete_current_goal(
                "SUCCEEDED",
                "action_succeeded",
            )
            rospy.loginfo(
                "[graph_nbv] %s goal reached",
                completed_kind,
            )
            self.global_empty_cycles = 0

            if completed_kind == "GLOBAL":
                self.mode_pub.publish(
                    String(data="LOCAL_NBV")
                )
                self.publish_status(
                    "GLOBAL_RELOCATION_REACHED"
                )
            else:
                self.publish_status(
                    "LOCAL_VIEWPOINT_REACHED"
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
                "[graph_nbv] %s goal failed state=%d",
                self.current_goal_kind,
                state,
            )
            self.fail_current_goal("ACTION_%d" % state)

    def publish_markers(
        self,
        nodes: List[GraphNode],
        edges: List[Tuple[int, int]],
        robot_pose: Tuple[float, float, float],
        local_selected: Optional[GraphNode],
        global_selected: Optional[GlobalTarget],
        global_frontiers: List[Tuple[float, float]],
    ) -> None:
        markers = MarkerArray()
        now = rospy.Time.now()

        clear = Marker()
        clear.header.frame_id = self.global_frame
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        positions = [(node.x, node.y) for node in nodes]
        positions.append((robot_pose[0], robot_pose[1]))

        edge_marker = Marker()
        edge_marker.header.frame_id = self.global_frame
        edge_marker.header.stamp = now
        edge_marker.ns = "graph_edges"
        edge_marker.id = 1
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.025
        edge_marker.color.r = 0.20
        edge_marker.color.g = 0.60
        edge_marker.color.b = 1.00
        edge_marker.color.a = 0.35

        for first, second in edges:
            for index in (first, second):
                point = Point()
                point.x = positions[index][0]
                point.y = positions[index][1]
                point.z = 0.08
                edge_marker.points.append(point)

        markers.markers.append(edge_marker)

        transit_marker = Marker()
        transit_marker.header.frame_id = self.global_frame
        transit_marker.header.stamp = now
        transit_marker.ns = "transit_nodes"
        transit_marker.id = 2
        transit_marker.type = Marker.SPHERE_LIST
        transit_marker.action = Marker.ADD
        transit_marker.scale.x = 0.07
        transit_marker.scale.y = 0.07
        transit_marker.scale.z = 0.07
        transit_marker.color.r = 0.25
        transit_marker.color.g = 0.65
        transit_marker.color.b = 1.00
        transit_marker.color.a = 0.55

        target_marker = Marker()
        target_marker.header.frame_id = self.global_frame
        target_marker.header.stamp = now
        target_marker.ns = "nbv_nodes"
        target_marker.id = 3
        target_marker.type = Marker.SPHERE_LIST
        target_marker.action = Marker.ADD
        target_marker.scale.x = 0.12
        target_marker.scale.y = 0.12
        target_marker.scale.z = 0.12
        target_marker.color.r = 0.15
        target_marker.color.g = 1.00
        target_marker.color.b = 0.30
        target_marker.color.a = 0.85

        for node in nodes:
            point = Point()
            point.x = node.x
            point.y = node.y
            point.z = 0.10

            if node.is_target:
                target_marker.points.append(point)
            else:
                transit_marker.points.append(point)

        markers.markers.extend(
            [transit_marker, target_marker]
        )

        frontier_marker = Marker()
        frontier_marker.header.frame_id = self.global_frame
        frontier_marker.header.stamp = now
        frontier_marker.ns = "global_frontiers"
        frontier_marker.id = 4
        frontier_marker.type = Marker.CUBE_LIST
        frontier_marker.action = Marker.ADD
        frontier_marker.scale.x = 0.18
        frontier_marker.scale.y = 0.18
        frontier_marker.scale.z = 0.18
        frontier_marker.color.r = 1.00
        frontier_marker.color.g = 0.85
        frontier_marker.color.b = 0.10
        frontier_marker.color.a = 0.90

        for x, y in global_frontiers:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.12
            frontier_marker.points.append(point)

        markers.markers.append(frontier_marker)

        if local_selected is not None:
            selected = Marker()
            selected.header.frame_id = self.global_frame
            selected.header.stamp = now
            selected.ns = "selected_local"
            selected.id = 5
            selected.type = Marker.SPHERE
            selected.action = Marker.ADD
            selected.pose.position.x = local_selected.x
            selected.pose.position.y = local_selected.y
            selected.pose.position.z = 0.25
            selected.pose.orientation.w = 1.0
            selected.scale.x = 0.35
            selected.scale.y = 0.35
            selected.scale.z = 0.35
            selected.color.r = 1.00
            selected.color.g = 0.15
            selected.color.b = 0.10
            selected.color.a = 1.00
            markers.markers.append(selected)

        if global_selected is not None:
            selected = Marker()
            selected.header.frame_id = self.global_frame
            selected.header.stamp = now
            selected.ns = "selected_global"
            selected.id = 6
            selected.type = Marker.CUBE
            selected.action = Marker.ADD
            selected.pose.position.x = global_selected.x
            selected.pose.position.y = global_selected.y
            selected.pose.position.z = 0.28
            selected.pose.orientation.w = 1.0
            selected.scale.x = 0.42
            selected.scale.y = 0.42
            selected.scale.z = 0.42
            selected.color.r = 0.95
            selected.color.g = 0.10
            selected.color.b = 1.00
            selected.color.a = 1.00
            markers.markers.append(selected)

        if self.gate_origin is not None and self.gate_forward is not None:
            gate_line = Marker()
            gate_line.header.frame_id = self.global_frame
            gate_line.header.stamp = now
            gate_line.ns = "gate_line"
            gate_line.id = 20
            gate_line.type = Marker.LINE_LIST
            gate_line.action = Marker.ADD
            gate_line.scale.x = 0.08
            if self.gate_locked:
                gate_line.color.r = 1.0
                gate_line.color.g = 0.15
                gate_line.color.b = 0.15
            else:
                gate_line.color.r = 0.55
                gate_line.color.g = 0.55
                gate_line.color.b = 0.55
            gate_line.color.a = 0.95

            perpendicular_x = -self.gate_forward[1]
            perpendicular_y = self.gate_forward[0]
            for sign in (-1.0, 1.0):
                point = Point()
                point.x = self.gate_origin[0] + sign * 8.0 * perpendicular_x
                point.y = self.gate_origin[1] + sign * 8.0 * perpendicular_y
                point.z = 0.16
                gate_line.points.append(point)
            markers.markers.append(gate_line)

            gate_arrow = Marker()
            gate_arrow.header.frame_id = self.global_frame
            gate_arrow.header.stamp = now
            gate_arrow.ns = "gate_forward"
            gate_arrow.id = 21
            gate_arrow.type = Marker.ARROW
            gate_arrow.action = Marker.ADD
            gate_arrow.scale.x = 0.12
            gate_arrow.scale.y = 0.24
            gate_arrow.scale.z = 0.30
            gate_arrow.color.r = 0.10
            gate_arrow.color.g = 1.00
            gate_arrow.color.b = 0.20
            gate_arrow.color.a = 1.00

            start = Point()
            start.x = self.gate_origin[0]
            start.y = self.gate_origin[1]
            start.z = 0.20
            end = Point()
            end.x = self.gate_origin[0] + 1.5 * self.gate_forward[0]
            end.y = self.gate_origin[1] + 1.5 * self.gate_forward[1]
            end.z = 0.20
            gate_arrow.points = [start, end]
            markers.markers.append(gate_arrow)

        self.marker_pub.publish(markers)

    def initial_control_callback(self, _event) -> None:
        if (
            self.finished
            or self.initial_complete
            or self.initial_paused
        ):
            return

        if self.grid is None or self.covered is None:
            self.stop_initial_motion()
            self.publish_status("WAITING_FOR_MAP")
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.stop_initial_motion()
            self.publish_status("WAITING_FOR_TF")
            return

        if not pose_is_reasonable(robot_pose, self.max_pose_abs):
            self.abort_safely("INVALID_TF_POSE")
            return

        if self.manual_gate_enabled:
            self.initialize_manual_gate(robot_pose)
            return

        self.handle_initial_forward(robot_pose)

    def timer_callback(self, _event) -> None:
        if self.finished:
            return

        if self.grid is None or self.covered is None:
            self.set_exploration_state("WAITING_FOR_MAP")
            self.publish_status("WAITING_FOR_MAP")
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            if self.initial_complete:
                self.handle_health_degraded("TF_UNAVAILABLE")
            else:
                self.set_exploration_state("WAITING_FOR_TF")
                self.publish_status("WAITING_FOR_TF")
            return

        self.last_robot_pose = robot_pose
        if not pose_is_reasonable(robot_pose, self.max_pose_abs):
            if self.initial_complete:
                self.handle_health_degraded("INVALID_TF_POSE")
            else:
                self.abort_safely("INVALID_TF_POSE")
            return

        if not self.initial_complete:
            return

        now = rospy.Time.now()
        health_reason = self.runtime_health_reason(now)
        if health_reason is not None:
            self.handle_health_degraded(health_reason)
            return
        self.recover_health()

        if (
            self.recovery_until.to_sec() > 0.0
            and now.to_sec() > 0.0
            and now < self.recovery_until
        ):
            if self.exploration_state != "HEALTH_DEGRADED":
                self.set_exploration_state("GOAL_RECOVERY")
            self.publish_status(
                "RECOVERY_WAIT_%.1f" % (
                    (self.recovery_until - now).to_sec(),
                )
            )
            self.publish_markers(
                self.last_nodes,
                self.last_edges,
                robot_pose,
                self.last_local_selected,
                self.last_global_selected,
                self.last_global_frontiers,
            )
            return

        if self.recovery_until.to_sec() > 0.0:
            self.recovery_until = rospy.Time(0)
            self.recovery_reason = ""
            if self.exploration_state in (
                "GOAL_RECOVERY",
                "HEALTH_DEGRADED",
            ):
                self.set_exploration_state("WAITING_FOR_NEXT_GOAL")

        if self.initial_paused:
            self.stop_initial_motion()
            self.publish_markers(
                [],
                [],
                robot_pose,
                None,
                None,
                [],
            )
            return

        self.mark_coverage(robot_pose)

        if self.current_goal_xy() is not None:
            self.handle_goal(robot_pose)
            self.publish_markers(
                self.last_nodes,
                self.last_edges,
                robot_pose,
                self.last_local_selected,
                self.last_global_selected,
                self.last_global_frontiers,
            )
            return

        layers = self.build_layers()
        if layers is None:
            return

        nodes = self.sample_local_nodes(
            layers, robot_pose
        )
        adjacency, edges = self.build_local_graph(
            nodes, robot_pose, layers
        )

        robot_index = len(nodes)
        distances = self.dijkstra(
            adjacency, robot_index
        )

        local_target = self.select_local_target(
            nodes, distances, robot_pose
        )

        self.last_nodes = nodes
        self.last_edges = edges
        self.last_local_selected = local_target
        self.last_global_selected = None
        self.last_global_frontiers = []

        if local_target is not None:
            self.global_empty_cycles = 0
            self.forward_empty_cycles = 0
            self.send_local_goal(
                local_target, robot_pose
            )
            self.publish_markers(
                nodes,
                edges,
                robot_pose,
                local_target,
                None,
                [],
            )
            return

        # Local room/area is exhausted. Do not finish. Search the whole
        # occupancy map for another reachable frontier.
        self.mode_pub.publish(
            String(data="GLOBAL_RELOCATION")
        )
        self.set_exploration_state("RELOCATING_GLOBAL")
        self.publish_status(
            "LOCAL_EXHAUSTED_SEARCHING_GLOBAL"
        )

        global_target, frontier_points = (
            self.global_frontier_targets(
                layers, robot_pose
            )
        )

        self.last_global_frontiers = frontier_points
        self.last_global_selected = global_target

        if global_target is not None:
            self.global_empty_cycles = 0
            self.forward_empty_cycles = 0
            self.send_global_goal(global_target)
        else:
            if self.gate_locked:
                self.forward_empty_cycles += 1
                self.publish_status(
                    "FORWARD_REGION_EMPTY_%d_OF_%d"
                    % (
                        self.forward_empty_cycles,
                        self.forward_finish_stable_cycles,
                    )
                )
                rospy.logwarn_throttle(
                    3.0,
                    "[graph_nbv] no reachable frontier in locked "
                    "forward region cycle=%d/%d",
                    self.forward_empty_cycles,
                    self.forward_finish_stable_cycles,
                )

                if gate_exhaustion_reached(
                    self.forward_empty_cycles,
                    self.forward_finish_stable_cycles,
                ):
                    if self.unlock_gate_on_exhaustion:
                        self.unlock_forward_gate()
                    else:
                        self.finish_exploration(
                            "FINISHED_FORWARD_REGION_LEGACY_POLICY"
                        )

                self.publish_markers(
                    nodes,
                    edges,
                    robot_pose,
                    None,
                    None,
                    frontier_points,
                )
                return

            self.global_empty_cycles += 1
            self.publish_status(
                "NO_REACHABLE_GLOBAL_FRONTIER"
            )
            rospy.logwarn_throttle(
                3.0,
                "[graph_nbv] no reachable global frontier "
                "cycle=%d/%d",
                self.global_empty_cycles,
                self.global_empty_cycles_to_finish,
            )

            if (
                self.global_empty_cycles
                >= self.global_empty_cycles_to_finish
            ):
                self.finish_exploration("FINISHED_NO_REACHABLE_FRONTIER")

        self.publish_markers(
            nodes,
            edges,
            robot_pose,
            None,
            global_target,
            frontier_points,
        )


if __name__ == "__main__":
    try:
        GraphNBVStageB31ManualGate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
