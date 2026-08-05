#!/usr/bin/env bash
set -euo pipefail

WS="${1:-$HOME/catkin_ws}"
PKG="$WS/src/danger_search_robot"
STAMP="$(date +%Y%m%d_%H%M%S)"

if [[ ! -f "$PKG/package.xml" || ! -f "$PKG/CMakeLists.txt" ]]; then
  echo "ERROR: danger_search_robot not found under: $PKG" >&2
  exit 1
fi

echo "[1/7] Checking Python/OpenCV..."
if ! python3 - <<'PY'
import cv2
import numpy
print("cv2:", cv2.__version__)
print("numpy:", numpy.__version__)
PY
then
  echo
  echo "python3-opencv is missing. Install it first:"
  echo "  sudo apt update"
  echo "  sudo apt install -y python3-opencv"
  exit 2
fi

echo "[2/7] Backing up package metadata..."
cp "$PKG/package.xml" "$PKG/package.xml.bak_$STAMP"
cp "$PKG/CMakeLists.txt" "$PKG/CMakeLists.txt.bak_$STAMP"

mkdir -p   "$PKG/exploration/scripts"   "$PKG/exploration/config"   "$PKG/exploration/launch"

echo "[3/7] Writing hybrid exploration node..."
cat > "$PKG/exploration/scripts/hybrid_exploration_node.py" <<'PY_NODE'
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
from typing import Dict, List, Optional, Tuple

import actionlib
import cv2
import numpy as np
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import GetPlan, OccupancyGrid
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


TERMINAL_STATES = {
    GoalStatus.PREEMPTED,
    GoalStatus.SUCCEEDED,
    GoalStatus.ABORTED,
    GoalStatus.REJECTED,
    GoalStatus.RECALLED,
    GoalStatus.LOST,
}


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class HybridExplorationNode:
    """Frontier + physical coverage + in-place visual scan exploration."""

    def __init__(self) -> None:
        rospy.init_node("hybrid_exploration")

        self.map_topic = rospy.get_param("~map_topic", "/map_confirmed")
        self.map_frame = rospy.get_param("~map_frame", "map_level")
        self.base_frame = rospy.get_param("~base_frame", "body")
        self.move_base_action = rospy.get_param("~move_base_action", "/move_base")
        self.make_plan_service = rospy.get_param(
            "~make_plan_service", "/move_base/make_plan"
        )

        self.occupied_threshold = int(rospy.get_param("~occupied_threshold", 50))
        self.planning_period = float(rospy.get_param("~planning_period", 3.0))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.5))

        self.coverage_radius = float(rospy.get_param("~coverage_radius", 0.90))
        self.trajectory_step = float(rospy.get_param("~trajectory_step", 0.20))
        self.min_goal_clearance = float(
            rospy.get_param("~min_goal_clearance", 0.35)
        )
        self.min_goal_distance = float(rospy.get_param("~min_goal_distance", 0.70))
        self.max_goal_distance = float(rospy.get_param("~max_goal_distance", 25.0))

        self.min_frontier_length = float(
            rospy.get_param("~min_frontier_length", 0.60)
        )
        self.frontier_search_radius = float(
            rospy.get_param("~frontier_search_radius", 1.50)
        )
        self.frontier_gain_radius = float(
            rospy.get_param("~frontier_gain_radius", 2.00)
        )
        self.small_frontier_gain = float(
            rospy.get_param("~small_frontier_gain", 1.50)
        )
        self.max_frontier_streak = int(
            rospy.get_param("~max_frontier_streak", 2)
        )

        self.min_coverage_area = float(
            rospy.get_param("~min_coverage_area", 1.50)
        )
        self.coverage_gain_radius = float(
            rospy.get_param("~coverage_gain_radius", 1.50)
        )

        self.frontier_unknown_weight = float(
            rospy.get_param("~frontier_unknown_weight", 3.0)
        )
        self.frontier_size_weight = float(
            rospy.get_param("~frontier_size_weight", 0.8)
        )
        self.coverage_area_weight = float(
            rospy.get_param("~coverage_area_weight", 1.5)
        )
        self.clearance_weight = float(
            rospy.get_param("~clearance_weight", 1.0)
        )
        self.distance_weight = float(rospy.get_param("~distance_weight", 0.25))

        self.max_plan_candidates = int(
            rospy.get_param("~max_plan_candidates", 8)
        )
        self.make_plan_tolerance = float(
            rospy.get_param("~make_plan_tolerance", 0.30)
        )
        self.use_make_plan = bool(rospy.get_param("~use_make_plan", True))

        self.goal_timeout = float(rospy.get_param("~goal_timeout", 90.0))
        self.progress_timeout = float(rospy.get_param("~progress_timeout", 30.0))
        self.progress_epsilon = float(rospy.get_param("~progress_epsilon", 0.20))
        self.blacklist_radius = float(rospy.get_param("~blacklist_radius", 0.80))
        self.blacklist_timeout = float(
            rospy.get_param("~blacklist_timeout", 180.0)
        )

        self.scan_count = max(1, int(rospy.get_param("~scan_count", 4)))
        self.scan_dwell = float(rospy.get_param("~scan_dwell", 2.0))
        self.scan_goal_timeout = float(
            rospy.get_param("~scan_goal_timeout", 20.0)
        )
        self.empty_cycles_to_finish = max(
            1, int(rospy.get_param("~empty_cycles_to_finish", 5))
        )

        self.map_lock = threading.RLock()
        self.grid: Optional[np.ndarray] = None
        self.covered: Optional[np.ndarray] = None
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_origin_yaw = 0.0
        self.map_width = 0
        self.map_height = 0

        self.last_trajectory_pose: Optional[Tuple[float, float]] = None
        self.blacklist: List[Tuple[float, float, rospy.Time]] = []

        self.current_goal: Optional[Dict] = None
        self.current_goal_started = rospy.Time(0)
        self.current_best_distance = float("inf")
        self.last_progress_time = rospy.Time(0)

        self.scan_queue: List[float] = []
        self.scan_position: Optional[Tuple[float, float]] = None
        self.scan_wait_until = rospy.Time(0)

        self.frontier_streak = 0
        self.empty_cycles = 0
        self.finished = False
        self.last_plan_time = rospy.Time(0)
        self.make_plan_ready = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.move_client = actionlib.SimpleActionClient(
            self.move_base_action, MoveBaseAction
        )
        self.make_plan = rospy.ServiceProxy(self.make_plan_service, GetPlan)

        self.marker_pub = rospy.Publisher(
            "~markers", MarkerArray, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher("~status", String, queue_size=10, latch=True)
        self.finished_pub = rospy.Publisher(
            "~finished", Bool, queue_size=1, latch=True
        )

        rospy.Subscriber(
            self.map_topic, OccupancyGrid, self.map_callback, queue_size=1
        )

        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        self.publish_status("WAITING_FOR_MAP")
        self.finished_pub.publish(Bool(data=False))
        rospy.loginfo(
            "[hybrid_exploration] map=%s frame=%s base=%s action=%s",
            self.map_topic,
            self.map_frame,
            self.base_frame,
            self.move_base_action,
        )

    def publish_status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))

    def map_callback(self, msg: OccupancyGrid) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0:
            return

        grid = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        q = msg.info.origin.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        with self.map_lock:
            geometry_changed = (
                self.grid is None
                or width != self.map_width
                or height != self.map_height
                or abs(msg.info.resolution - self.map_resolution) > 1e-9
                or abs(msg.info.origin.position.x - self.map_origin_x) > 1e-6
                or abs(msg.info.origin.position.y - self.map_origin_y) > 1e-6
                or abs(yaw - self.map_origin_yaw) > 1e-6
            )

            self.grid = grid
            self.map_width = width
            self.map_height = height
            self.map_resolution = float(msg.info.resolution)
            self.map_origin_x = float(msg.info.origin.position.x)
            self.map_origin_y = float(msg.info.origin.position.y)
            self.map_origin_yaw = float(yaw)

            if geometry_changed:
                self.covered = np.zeros((height, width), dtype=np.uint8)
                self.last_trajectory_pose = None
                rospy.loginfo(
                    "[hybrid_exploration] map geometry: %dx%d res=%.3f",
                    width,
                    height,
                    self.map_resolution,
                )

    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout),
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0, "[hybrid_exploration] TF unavailable: %s", str(exc)
            )
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return float(t.x), float(t.y), float(yaw)

    def world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        dx = x - self.map_origin_x
        dy = y - self.map_origin_y
        cos_yaw = math.cos(self.map_origin_yaw)
        sin_yaw = math.sin(self.map_origin_yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        col = int(math.floor(local_x / self.map_resolution))
        row = int(math.floor(local_y / self.map_resolution))
        if row < 0 or row >= self.map_height or col < 0 or col >= self.map_width:
            return None
        return row, col

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        local_x = (col + 0.5) * self.map_resolution
        local_y = (row + 0.5) * self.map_resolution
        cos_yaw = math.cos(self.map_origin_yaw)
        sin_yaw = math.sin(self.map_origin_yaw)
        x = self.map_origin_x + cos_yaw * local_x - sin_yaw * local_y
        y = self.map_origin_y + sin_yaw * local_x + cos_yaw * local_y
        return x, y

    def mark_trajectory(self, pose: Tuple[float, float, float]) -> None:
        x, y, _ = pose
        if self.last_trajectory_pose is not None:
            if math.hypot(x - self.last_trajectory_pose[0], y - self.last_trajectory_pose[1]) < self.trajectory_step:
                return

        with self.map_lock:
            if self.covered is None or self.map_resolution <= 0.0:
                return
            rc = self.world_to_grid(x, y)
            if rc is None:
                return
            row, col = rc
            radius_cells = max(1, int(round(self.coverage_radius / self.map_resolution)))
            cv2.circle(self.covered, (col, row), radius_cells, 1, thickness=-1)
            self.last_trajectory_pose = (x, y)

    def cleanup_blacklist(self) -> None:
        now = rospy.Time.now()
        self.blacklist = [
            entry
            for entry in self.blacklist
            if (now - entry[2]).to_sec() < self.blacklist_timeout
        ]

    def is_blacklisted(self, x: float, y: float) -> bool:
        self.cleanup_blacklist()
        return any(
            math.hypot(x - bx, y - by) < self.blacklist_radius
            for bx, by, _ in self.blacklist
        )

    def add_blacklist(self, x: float, y: float) -> None:
        self.blacklist.append((x, y, rospy.Time.now()))
        rospy.logwarn(
            "[hybrid_exploration] blacklisted goal (%.2f, %.2f)", x, y
        )

    @staticmethod
    def crop_bounds(known: np.ndarray, margin: int = 20) -> Optional[Tuple[int, int, int, int]]:
        rows, cols = np.nonzero(known)
        if rows.size == 0:
            return None
        r0 = max(0, int(rows.min()) - margin)
        r1 = min(known.shape[0], int(rows.max()) + margin + 1)
        c0 = max(0, int(cols.min()) - margin)
        c1 = min(known.shape[1], int(cols.max()) + margin + 1)
        return r0, r1, c0, c1

    def generate_candidates(
        self, robot_pose: Tuple[float, float, float]
    ) -> Tuple[List[Dict], List[Dict]]:
        with self.map_lock:
            if self.grid is None or self.covered is None:
                return [], []
            grid = self.grid.copy()
            covered = self.covered.copy()
            resolution = self.map_resolution

        known = grid >= 0
        bounds = self.crop_bounds(known)
        if bounds is None:
            return [], []
        r0, r1, c0, c1 = bounds

        grid_c = grid[r0:r1, c0:c1]
        covered_c = covered[r0:r1, c0:c1].astype(bool)
        free = ((grid_c >= 0) & (grid_c < self.occupied_threshold)).astype(np.uint8)
        unknown = (grid_c < 0).astype(np.uint8)

        if int(free.sum()) == 0:
            return [], []

        clearance = cv2.distanceTransform(free, cv2.DIST_L2, 5) * resolution
        safe = (free > 0) & (clearance >= self.min_goal_clearance)

        frontier_neighbors = cv2.dilate(
            unknown, np.ones((3, 3), dtype=np.uint8), iterations=1
        )
        frontier_mask = ((free > 0) & (frontier_neighbors > 0)).astype(np.uint8)

        fg_cells = max(1, int(round(self.frontier_gain_radius / resolution)))
        fg_kernel = 2 * fg_cells + 1
        unknown_gain = cv2.boxFilter(
            unknown.astype(np.float32),
            ddepth=-1,
            ksize=(fg_kernel, fg_kernel),
            normalize=False,
            borderType=cv2.BORDER_CONSTANT,
        ) * (resolution * resolution)

        cg_cells = max(1, int(round(self.coverage_gain_radius / resolution)))
        cg_kernel = 2 * cg_cells + 1
        uncovered_free = ((free > 0) & (~covered_c)).astype(np.uint8)
        coverage_gain = cv2.boxFilter(
            uncovered_free.astype(np.float32),
            ddepth=-1,
            ksize=(cg_kernel, cg_kernel),
            normalize=False,
            borderType=cv2.BORDER_CONSTANT,
        ) * (resolution * resolution)

        frontier_candidates: List[Dict] = []
        min_frontier_cells = max(3, int(round(self.min_frontier_length / resolution)))
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(
            frontier_mask, connectivity=8
        )
        search_cells = max(1, int(round(self.frontier_search_radius / resolution)))

        for label in range(1, count):
            size = int(stats[label, cv2.CC_STAT_AREA])
            if size < min_frontier_cells:
                continue

            center_c = int(round(centroids[label][0]))
            center_r = int(round(centroids[label][1]))
            sr0 = max(0, center_r - search_cells)
            sr1 = min(grid_c.shape[0], center_r + search_cells + 1)
            sc0 = max(0, center_c - search_cells)
            sc1 = min(grid_c.shape[1], center_c + search_cells + 1)

            safe_patch = safe[sr0:sr1, sc0:sc1]
            if not np.any(safe_patch):
                continue

            rr, cc = np.indices(safe_patch.shape)
            global_rr = rr + sr0
            global_cc = cc + sc0
            distance_to_frontier = np.hypot(
                global_rr - center_r, global_cc - center_c
            ) * resolution

            objective = (
                2.0 * unknown_gain[sr0:sr1, sc0:sc1]
                + self.clearance_weight * clearance[sr0:sr1, sc0:sc1]
                + 0.4 * coverage_gain[sr0:sr1, sc0:sc1]
                - 0.6 * distance_to_frontier
            )
            objective = np.where(safe_patch, objective, -np.inf)
            flat_index = int(np.argmax(objective))
            pr, pc = np.unravel_index(flat_index, objective.shape)
            local_r = sr0 + int(pr)
            local_c = sc0 + int(pc)
            row = r0 + local_r
            col = c0 + local_c
            x, y = self.grid_to_world(row, col)
            robot_distance = math.hypot(x - robot_pose[0], y - robot_pose[1])
            if not (self.min_goal_distance <= robot_distance <= self.max_goal_distance):
                continue
            if self.is_blacklisted(x, y):
                continue

            frontier_world = self.grid_to_world(r0 + center_r, c0 + center_c)
            yaw = math.atan2(frontier_world[1] - y, frontier_world[0] - x)
            gain = float(unknown_gain[local_r, local_c])
            score = (
                self.frontier_unknown_weight * gain
                + self.frontier_size_weight * size * resolution
                + self.clearance_weight * float(clearance[local_r, local_c])
                - self.distance_weight * robot_distance
            )
            frontier_candidates.append(
                {
                    "type": "frontier",
                    "x": x,
                    "y": y,
                    "yaw": yaw,
                    "score": score,
                    "unknown_gain": gain,
                    "size": size,
                    "distance": robot_distance,
                }
            )

        coverage_candidates: List[Dict] = []
        coverage_mask = (safe & (~covered_c)).astype(np.uint8)
        coverage_distance = cv2.distanceTransform(
            coverage_mask, cv2.DIST_L2, 5
        ) * resolution
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            coverage_mask, connectivity=8
        )
        min_coverage_cells = max(
            1, int(round(self.min_coverage_area / (resolution * resolution)))
        )

        for label in range(1, count):
            area_cells = int(stats[label, cv2.CC_STAT_AREA])
            if area_cells < min_coverage_cells:
                continue
            mask = labels == label
            objective = (
                coverage_distance
                + 0.7 * clearance
                + 0.15 * coverage_gain
            )
            objective = np.where(mask, objective, -np.inf)
            flat_index = int(np.argmax(objective))
            local_r, local_c = np.unravel_index(flat_index, objective.shape)
            row = r0 + int(local_r)
            col = c0 + int(local_c)
            x, y = self.grid_to_world(row, col)
            robot_distance = math.hypot(x - robot_pose[0], y - robot_pose[1])
            if not (self.min_goal_distance <= robot_distance <= self.max_goal_distance):
                continue
            if self.is_blacklisted(x, y):
                continue

            area_m2 = area_cells * resolution * resolution
            yaw = math.atan2(y - robot_pose[1], x - robot_pose[0])
            score = (
                self.coverage_area_weight * area_m2
                + self.clearance_weight * float(clearance[local_r, local_c])
                + 0.5 * float(coverage_distance[local_r, local_c])
                - self.distance_weight * robot_distance
            )
            coverage_candidates.append(
                {
                    "type": "coverage",
                    "x": x,
                    "y": y,
                    "yaw": yaw,
                    "score": score,
                    "area": area_m2,
                    "distance": robot_distance,
                }
            )

        frontier_candidates.sort(key=lambda item: item["score"], reverse=True)
        coverage_candidates.sort(key=lambda item: item["score"], reverse=True)
        return frontier_candidates, coverage_candidates

    def pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def candidate_has_plan(
        self, candidate: Dict, robot_pose: Tuple[float, float, float]
    ) -> bool:
        if not self.use_make_plan:
            return True

        if not self.make_plan_ready:
            try:
                rospy.wait_for_service(self.make_plan_service, timeout=0.3)
                self.make_plan_ready = True
            except rospy.ROSException:
                rospy.logwarn_throttle(
                    5.0,
                    "[hybrid_exploration] %s unavailable; temporarily skipping plan validation",
                    self.make_plan_service,
                )
                return True

        start = self.pose_stamped(robot_pose[0], robot_pose[1], robot_pose[2])
        goal = self.pose_stamped(candidate["x"], candidate["y"], candidate["yaw"])
        try:
            response = self.make_plan(
                start=start, goal=goal, tolerance=self.make_plan_tolerance
            )
            return len(response.plan.poses) >= 2
        except rospy.ServiceException as exc:
            self.make_plan_ready = False
            rospy.logwarn(
                "[hybrid_exploration] make_plan failed: %s; accepting candidate",
                str(exc),
            )
            return True

    def choose_candidate(
        self,
        robot_pose: Tuple[float, float, float],
        frontier_candidates: List[Dict],
        coverage_candidates: List[Dict],
    ) -> Optional[Dict]:
        prefer_coverage = False
        if coverage_candidates:
            if self.frontier_streak >= self.max_frontier_streak:
                prefer_coverage = True
            elif not frontier_candidates:
                prefer_coverage = True
            elif frontier_candidates[0].get("unknown_gain", 0.0) < self.small_frontier_gain:
                prefer_coverage = True

        ordered_groups = (
            [coverage_candidates, frontier_candidates]
            if prefer_coverage
            else [frontier_candidates, coverage_candidates]
        )

        checked = 0
        for group in ordered_groups:
            for candidate in group:
                if checked >= self.max_plan_candidates:
                    return None
                checked += 1
                if self.candidate_has_plan(candidate, robot_pose):
                    return candidate
                self.add_blacklist(candidate["x"], candidate["y"])
        return None

    def send_goal(self, candidate: Dict) -> None:
        goal = MoveBaseGoal()
        goal.target_pose = self.pose_stamped(
            candidate["x"], candidate["y"], candidate["yaw"]
        )
        self.move_client.send_goal(goal)
        self.current_goal = candidate
        self.current_goal_started = rospy.Time.now()
        self.last_progress_time = self.current_goal_started
        self.current_best_distance = float("inf")
        self.publish_status(
            "NAVIGATING_%s x=%.2f y=%.2f"
            % (candidate["type"].upper(), candidate["x"], candidate["y"])
        )
        rospy.loginfo(
            "[hybrid_exploration] sent %s goal (%.2f, %.2f), score=%.2f",
            candidate["type"],
            candidate["x"],
            candidate["y"],
            candidate.get("score", 0.0),
        )

    def start_visual_scan(self, robot_pose: Tuple[float, float, float]) -> None:
        self.scan_position = (robot_pose[0], robot_pose[1])
        base_yaw = robot_pose[2]
        self.scan_queue = [
            normalize_angle(base_yaw + 2.0 * math.pi * index / self.scan_count)
            for index in range(self.scan_count)
        ]
        self.scan_wait_until = rospy.Time.now() + rospy.Duration(self.scan_dwell)
        self.publish_status("VISUAL_SCAN_PREPARE")
        rospy.loginfo(
            "[hybrid_exploration] coverage goal reached; starting %d-direction scan",
            self.scan_count,
        )

    def send_next_scan_goal(self) -> None:
        if not self.scan_queue or self.scan_position is None:
            self.scan_queue = []
            self.scan_position = None
            self.current_goal = None
            self.publish_status("SELECT_GOAL")
            return

        yaw = self.scan_queue.pop(0)
        candidate = {
            "type": "scan",
            "x": self.scan_position[0],
            "y": self.scan_position[1],
            "yaw": yaw,
            "score": 0.0,
        }
        self.send_goal(candidate)

    def handle_goal_terminal(
        self, state: int, robot_pose: Tuple[float, float, float]
    ) -> None:
        candidate = self.current_goal
        self.current_goal = None
        if candidate is None:
            return

        goal_type = candidate["type"]
        if goal_type == "scan":
            self.scan_wait_until = rospy.Time.now() + rospy.Duration(self.scan_dwell)
            if state != GoalStatus.SUCCEEDED:
                rospy.logwarn(
                    "[hybrid_exploration] scan orientation ended with state=%d; continuing",
                    state,
                )
            return

        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo(
                "[hybrid_exploration] reached %s goal", goal_type
            )
            if goal_type == "frontier":
                self.frontier_streak += 1
                self.publish_status("FRONTIER_REACHED")
            else:
                self.frontier_streak = 0
                self.publish_status("COVERAGE_REACHED")
                self.start_visual_scan(robot_pose)
        else:
            rospy.logwarn(
                "[hybrid_exploration] %s goal failed with state=%d",
                goal_type,
                state,
            )
            self.add_blacklist(candidate["x"], candidate["y"])
            self.publish_status("GOAL_FAILED")

    def monitor_current_goal(
        self, robot_pose: Tuple[float, float, float]
    ) -> None:
        if self.current_goal is None:
            return

        state = self.move_client.get_state()
        if state in TERMINAL_STATES:
            self.handle_goal_terminal(state, robot_pose)
            return

        now = rospy.Time.now()
        elapsed = (now - self.current_goal_started).to_sec()
        timeout = (
            self.scan_goal_timeout
            if self.current_goal["type"] == "scan"
            else self.goal_timeout
        )
        distance = math.hypot(
            robot_pose[0] - self.current_goal["x"],
            robot_pose[1] - self.current_goal["y"],
        )

        if distance < self.current_best_distance - self.progress_epsilon:
            self.current_best_distance = distance
            self.last_progress_time = now

        stalled = (now - self.last_progress_time).to_sec() > self.progress_timeout
        if elapsed > timeout or (
            stalled and self.current_goal["type"] != "scan"
        ):
            candidate = self.current_goal
            self.move_client.cancel_goal()
            self.current_goal = None
            if candidate["type"] != "scan":
                self.add_blacklist(candidate["x"], candidate["y"])
            rospy.logwarn(
                "[hybrid_exploration] cancelled %s goal: elapsed=%.1f stalled=%s",
                candidate["type"],
                elapsed,
                stalled,
            )
            self.publish_status("GOAL_TIMEOUT")

    def publish_markers(
        self,
        frontier_candidates: List[Dict],
        coverage_candidates: List[Dict],
        selected: Optional[Dict],
    ) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.header.stamp = rospy.Time.now()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        marker_id = 0
        for namespace, candidates, rgb in (
            ("frontier_candidates", frontier_candidates[:30], (0.1, 0.4, 1.0)),
            ("coverage_candidates", coverage_candidates[:30], (1.0, 0.6, 0.1)),
        ):
            for candidate in candidates:
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = rospy.Time.now()
                marker.ns = namespace
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position = Point(
                    candidate["x"], candidate["y"], 0.15
                )
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.20
                marker.scale.y = 0.20
                marker.scale.z = 0.20
                marker.color.r = rgb[0]
                marker.color.g = rgb[1]
                marker.color.b = rgb[2]
                marker.color.a = 0.85
                marker.lifetime = rospy.Duration(self.planning_period * 2.0)
                markers.markers.append(marker)

        if selected is not None:
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "selected_goal"
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = self.pose_stamped(
                selected["x"], selected["y"], selected["yaw"]
            ).pose
            marker.scale.x = 0.70
            marker.scale.y = 0.12
            marker.scale.z = 0.12
            marker.color.r = 0.9
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(self.planning_period * 2.0)
            markers.markers.append(marker)

        self.marker_pub.publish(markers)

    def timer_callback(self, _event: rospy.TimerEvent) -> None:
        if self.finished or rospy.is_shutdown():
            return

        with self.map_lock:
            if self.grid is None:
                return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        self.mark_trajectory(robot_pose)

        if not self.move_client.wait_for_server(rospy.Duration(0.01)):
            rospy.logwarn_throttle(
                5.0, "[hybrid_exploration] waiting for move_base action server"
            )
            self.publish_status("WAITING_FOR_MOVE_BASE")
            return

        if self.current_goal is not None:
            self.monitor_current_goal(robot_pose)
            return

        now = rospy.Time.now()
        if self.scan_queue:
            if now >= self.scan_wait_until:
                self.send_next_scan_goal()
            return

        if (now - self.last_plan_time).to_sec() < self.planning_period:
            return
        self.last_plan_time = now

        frontier_candidates, coverage_candidates = self.generate_candidates(
            robot_pose
        )
        selected = self.choose_candidate(
            robot_pose, frontier_candidates, coverage_candidates
        )
        self.publish_markers(frontier_candidates, coverage_candidates, selected)

        rospy.loginfo(
            "[hybrid_exploration] candidates: frontier=%d coverage=%d streak=%d",
            len(frontier_candidates),
            len(coverage_candidates),
            self.frontier_streak,
        )

        if selected is None:
            self.empty_cycles += 1
            self.publish_status(
                "NO_GOAL %d/%d" % (self.empty_cycles, self.empty_cycles_to_finish)
            )
            if self.empty_cycles >= self.empty_cycles_to_finish:
                self.finished = True
                self.finished_pub.publish(Bool(data=True))
                self.publish_status("FINISHED")
                rospy.loginfo(
                    "[hybrid_exploration] exploration finished: no reachable frontier or uncovered region"
                )
            return

        self.empty_cycles = 0
        self.send_goal(selected)


if __name__ == "__main__":
    try:
        HybridExplorationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

PY_NODE

echo "[4/7] Writing configuration..."
cat > "$PKG/exploration/config/hybrid_exploration.yaml" <<'YAML_CONFIG'
map_topic: /map_confirmed
map_frame: map_level
base_frame: body
move_base_action: /move_base
make_plan_service: /move_base/make_plan

occupied_threshold: 50
planning_period: 3.0
tf_timeout: 0.5

# 机器人实际轨迹附近视为已经进入/覆盖。
coverage_radius: 0.90
trajectory_step: 0.20

# 候选点必须留有足够净空，并避免反复发送脚边目标。
min_goal_clearance: 0.35
min_goal_distance: 0.70
max_goal_distance: 25.0

# Frontier 参数。
min_frontier_length: 0.60
frontier_search_radius: 1.50
frontier_gain_radius: 2.00
small_frontier_gain: 1.50
max_frontier_streak: 2

# 即使地图已经被走廊中的雷达扫出，只要机器人没有进入，
# 大面积未覆盖自由区仍会生成 coverage 目标。
min_coverage_area: 1.50
coverage_gain_radius: 1.50

frontier_unknown_weight: 3.0
frontier_size_weight: 0.8
coverage_area_weight: 1.5
clearance_weight: 1.0
distance_weight: 0.25

# 对候选点调用 move_base/make_plan 进行可达性验证。
use_make_plan: true
max_plan_candidates: 8
make_plan_tolerance: 0.30

# 卡死与失败保护。
goal_timeout: 90.0
progress_timeout: 30.0
progress_epsilon: 0.20
blacklist_radius: 0.80
blacklist_timeout: 180.0

# 到达内部覆盖点后，原地观察四个方向。
scan_count: 4
scan_dwell: 2.0
scan_goal_timeout: 20.0

# 连续多轮没有可达 frontier 或未覆盖区域才结束。
empty_cycles_to_finish: 5

YAML_CONFIG

echo "[5/7] Writing launch file..."
cat > "$PKG/exploration/launch/hybrid_exploration.launch" <<'XML_LAUNCH'
<launch>
  <arg name="map_topic" default="/map_confirmed"/>
  <arg name="map_frame" default="map_level"/>
  <arg name="base_frame" default="body"/>

  <node pkg="danger_search_robot"
        type="hybrid_exploration_node.py"
        name="hybrid_exploration"
        output="screen"
        respawn="false">
    <rosparam command="load"
              file="$(find danger_search_robot)/exploration/config/hybrid_exploration.yaml"/>

    <param name="map_topic" value="$(arg map_topic)"/>
    <param name="map_frame" value="$(arg map_frame)"/>
    <param name="base_frame" value="$(arg base_frame)"/>
  </node>
</launch>

XML_LAUNCH

chmod +x "$PKG/exploration/scripts/hybrid_exploration_node.py"

echo "[6/7] Patching package.xml and CMakeLists.txt..."
PKG_DIR="$PKG" python3 - <<'PY_PATCH'
from pathlib import Path
import os
import re

pkg_dir = Path(os.environ["PKG_DIR"])
package_xml = pkg_dir / "package.xml"
cmake_file = pkg_dir / "CMakeLists.txt"

deps = [
    "actionlib",
    "actionlib_msgs",
    "move_base_msgs",
    "visualization_msgs",
]

text = package_xml.read_text()
insert_lines = []
for dep in deps:
    if f"<depend>{dep}</depend>" not in text:
        insert_lines.append(f"  <depend>{dep}</depend>")

if insert_lines:
    marker = "  <export>"
    if marker not in text:
        raise RuntimeError("Could not find <export> in package.xml")
    text = text.replace(marker, "\n".join(insert_lines) + "\n\n" + marker, 1)
    package_xml.write_text(text)

cmake = cmake_file.read_text()
match = re.search(
    r"find_package\(catkin REQUIRED COMPONENTS\s*(.*?)\n\)",
    cmake,
    flags=re.S,
)
if not match:
    raise RuntimeError("Could not find catkin COMPONENTS block in CMakeLists.txt")

block = match.group(0)
missing = [dep for dep in deps if re.search(rf"(?m)^\s*{re.escape(dep)}\s*$", block) is None]
if missing:
    replacement = block[:-2] + "".join(f"  {dep}\n" for dep in missing) + ")"
    cmake = cmake[:match.start()] + replacement + cmake[match.end():]

install_target = "exploration/scripts/hybrid_exploration_node.py"
if install_target not in cmake:
    install_block = (
        "\ncatkin_install_python(PROGRAMS\n"
        f"  {install_target}\n"
        "  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}\n"
        ")\n"
    )
    marker = "## Mark executable scripts (Python etc.) for installation"
    if marker not in cmake:
        raise RuntimeError("Could not find Python install section in CMakeLists.txt")
    cmake = cmake.replace(marker, install_block + "\n" + marker, 1)

cmake_file.write_text(cmake)
PY_PATCH

echo "[7/7] Syntax check and catkin build..."
python3 -m py_compile "$PKG/exploration/scripts/hybrid_exploration_node.py"

source /opt/ros/noetic/setup.bash
cd "$WS"
catkin_make -j4

echo
echo "Hybrid exploration installed successfully."
echo "Launch command:"
echo "  source /opt/ros/noetic/setup.bash"
echo "  source $WS/devel/setup.bash"
echo "  roslaunch danger_search_robot hybrid_exploration.launch"
echo
echo "Do NOT launch explore_lite.launch at the same time."
