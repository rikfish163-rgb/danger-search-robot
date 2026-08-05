#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import actionlib
import cv2
import numpy as np
import rospy
import tf2_ros

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


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
    Two-stage explorer for the current first-floor layout.

    Phase 1: CORRIDOR_ADVANCE
      - Lock the robot's initial heading as the corridor direction.
      - Only accept targets at or ahead of the deepest reached progress.
      - Never allow the front hall to compete with forward corridor targets.
      - Choose targets by forward progress first, not frontier area.

    Phase 2: ROOM_SWEEP
      - Divide both sides of the traversed corridor into progress bins.
      - Find physically unvisited free-space regions in each side/bin.
      - Enter each region and perform four-direction visual scanning.

    This node intentionally does not attempt generic building exploration.
    """

    def __init__(self) -> None:
        rospy.init_node("corridor_room_explorer")

        self.map_topic = rospy.get_param("~map_topic", "/map_confirmed")
        self.global_frame = rospy.get_param("~global_frame", "map_level")
        self.base_frame = rospy.get_param("~base_frame", "body")
        self.action_name = rospy.get_param("~move_base_action", "/move_base")
        self.dry_run = bool(rospy.get_param("~dry_run", True))

        self.occupied_threshold = int(
            rospy.get_param("~occupied_threshold", 50)
        )
        self.min_goal_clearance = float(
            rospy.get_param("~min_goal_clearance", 0.38)
        )
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
            rospy.get_param("~corridor_center_weight", 2.0)
        )
        self.corridor_empty_cycles_to_finish = int(
            rospy.get_param("~corridor_empty_cycles_to_finish", 6)
        )
        self.minimum_corridor_progress = float(
            rospy.get_param("~minimum_corridor_progress", 3.0)
        )

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

        self.min_goal_distance = float(
            rospy.get_param("~min_goal_distance", 0.55)
        )
        self.max_goal_distance = float(
            rospy.get_param("~max_goal_distance", 12.0)
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

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.client = actionlib.SimpleActionClient(
            self.action_name,
            MoveBaseAction,
        )

        self.map_msg: Optional[OccupancyGrid] = None
        self.grid: Optional[np.ndarray] = None
        self.covered: Optional[np.ndarray] = None

        self.map_resolution = 0.0
        self.map_width = 0
        self.map_height = 0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_origin_yaw = 0.0

        self.mission_origin: Optional[Tuple[float, float, float]] = None
        self.max_corridor_progress = 0.0
        self.last_mark_pose: Optional[Tuple[float, float]] = None

        self.phase = "CORRIDOR_ADVANCE"
        self.corridor_empty_cycles = 0
        self.room_empty_cycles = 0
        self.finished = False

        self.current_goal: Optional[Candidate] = None
        self.current_goal_start = rospy.Time(0)
        self.last_goal_distance = float("inf")
        self.last_goal_progress_time = rospy.Time(0)

        self.scan_queue: List[float] = []
        self.scan_position: Optional[Tuple[float, float]] = None
        self.scan_zone_id = ""
        self.scan_wait_until = rospy.Time(0)

        self.completed_zones: Set[str] = set()
        self.failed_zones: Dict[str, int] = {}
        self.blacklist: List[Tuple[float, float]] = []

        self.last_selected: Optional[Candidate] = None

        self.status_pub = rospy.Publisher(
            "~status",
            String,
            queue_size=1,
            latch=True,
        )
        self.finished_pub = rospy.Publisher(
            "~finished",
            Bool,
            queue_size=1,
            latch=True,
        )
        self.marker_pub = rospy.Publisher(
            "~markers",
            MarkerArray,
            queue_size=1,
            latch=True,
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
            self.client.wait_for_server()
            rospy.loginfo(
                "[corridor_room_explorer] move_base action available"
            )

        rospy.Timer(
            rospy.Duration(self.plan_period),
            self.timer_callback,
        )

        self.publish_status("WAITING_FOR_MAP")

        rospy.loginfo(
            "[corridor_room_explorer] started dry_run=%s "
            "map=%s frame=%s base=%s",
            self.dry_run,
            self.map_topic,
            self.global_frame,
            self.base_frame,
        )

    def publish_status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))

    def map_callback(self, msg: OccupancyGrid) -> None:
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

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
        )

        self.map_msg = msg
        self.grid = grid
        self.map_width = width
        self.map_height = height
        self.map_resolution = resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_origin_yaw = origin_yaw

        if geometry_changed:
            self.covered = np.zeros(
                (height, width),
                dtype=np.uint8,
            )
            self.last_mark_pose = None
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
        self,
        x: float,
        y: float,
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
        self,
        row: int,
        col: int,
    ) -> Tuple[float, float]:
        local_x = (col + 0.5) * self.map_resolution
        local_y = (row + 0.5) * self.map_resolution

        c = math.cos(self.map_origin_yaw)
        s = math.sin(self.map_origin_yaw)

        x = self.map_origin_x + c * local_x - s * local_y
        y = self.map_origin_y + s * local_x + c * local_y
        return x, y

    def project(
        self,
        x: float,
        y: float,
    ) -> Tuple[float, float]:
        if self.mission_origin is None:
            return 0.0, 0.0

        ox, oy, yaw = self.mission_origin
        dx = x - ox
        dy = y - oy

        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)
        left_x = -forward_y
        left_y = forward_x

        progress = forward_x * dx + forward_y * dy
        lateral = left_x * dx + left_y * dy
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
            int(round(self.coverage_radius / self.map_resolution)),
        )
        cv2.circle(
            self.covered,
            (col, row),
            radius_cells,
            1,
            thickness=-1,
        )
        self.last_mark_pose = (x, y)

    def capture_origin(
        self,
        pose: Tuple[float, float, float],
    ) -> None:
        if self.mission_origin is not None:
            return

        self.mission_origin = pose
        self.max_corridor_progress = 0.0

        rospy.loginfo(
            "[corridor_room_explorer] mission origin x=%.2f y=%.2f "
            "yaw=%.1f deg. This heading is locked as corridor forward.",
            pose[0],
            pose[1],
            math.degrees(pose[2]),
        )
        self.publish_status("CORRIDOR_ADVANCE")

    def map_arrays(
        self,
    ) -> Optional[Dict[str, np.ndarray]]:
        if (
            self.grid is None
            or self.covered is None
            or self.mission_origin is None
        ):
            return None

        grid = self.grid.copy()
        free = ((grid >= 0) & (grid < self.occupied_threshold)).astype(
            np.uint8
        )
        occupied = (grid >= self.occupied_threshold).astype(np.uint8)
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
        distance = math.hypot(x - robot_pose[0], y - robot_pose[1])
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

        # Hard constraints:
        # 1. only corridor band;
        # 2. never meaningfully behind deepest reached position.
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
            (2 * radius_cells + 1, 2 * radius_cells + 1),
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
                    >= self.max_corridor_progress
                    - self.backtrack_tolerance
                )
            )

            valid_indices = np.argwhere(valid)
            if valid_indices.size == 0:
                continue

            scores = (
                8.0 * progress[valid]
                + 2.5 * clearance[valid]
                - self.corridor_center_weight
                * np.abs(lateral[valid])
            )

            order = np.argsort(scores)[::-1]

            chosen = None
            for idx in order[:100]:
                row, col = valid_indices[int(idx)]
                x, y = self.grid_to_world(int(row), int(col))

                if self.is_blacklisted(x, y):
                    continue
                if not self.candidate_distance_ok(x, y, robot_pose):
                    continue

                chosen = (
                    int(row),
                    int(col),
                    x,
                    y,
                    float(scores[int(idx)]),
                )
                break

            if chosen is None:
                continue

            row, col, x, y, score = chosen
            p = float(progress[row, col])
            lat = float(lateral[row, col])

            candidate = Candidate(
                x=x,
                y=y,
                yaw=self.mission_origin[2],
                kind="CORRIDOR_FRONTIER",
                score=score,
                progress=p,
                lateral=lat,
            )

            # Primary ordering is deepest reachable frontier progress.
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

        # Fallback: move to the deepest already-known safe corridor cell.
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
            10.0 * progress[probe_mask]
            + 2.0 * clearance[probe_mask]
            - self.corridor_center_weight
            * np.abs(lateral[probe_mask])
        )
        order = np.argsort(scores)[::-1]

        for idx in order[:200]:
            row, col = indices[int(idx)]
            x, y = self.grid_to_world(int(row), int(col))

            if self.is_blacklisted(x, y):
                continue
            if not self.candidate_distance_ok(x, y, robot_pose):
                continue

            return Candidate(
                x=x,
                y=y,
                yaw=self.mission_origin[2],
                kind="CORRIDOR_PROBE",
                score=float(scores[int(idx)]),
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
                np.abs(lateral) >= self.room_lateral_min
            )
        )

        if not np.any(unvisited):
            return None

        # Depth inside an unvisited region rewards room interiors,
        # rather than door thresholds or corridor edges.
        unvisited_depth_cells = cv2.distanceTransform(
            unvisited.astype(np.uint8),
            cv2.DIST_L2,
            5,
        )
        unvisited_depth = (
            unvisited_depth_cells * self.map_resolution
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
                    side_mask = lateral >= self.room_lateral_min
                else:
                    side_mask = lateral <= -self.room_lateral_min

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
                for idx in order[:200]:
                    row, col = indices[int(idx)]
                    x, y = self.grid_to_world(int(row), int(col))

                    if self.is_blacklisted(x, y):
                        continue
                    if not self.candidate_distance_ok(
                        x,
                        y,
                        robot_pose,
                    ):
                        continue

                    selected = (
                        int(row),
                        int(col),
                        x,
                        y,
                        float(scores[int(idx)]),
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

        # Start with far-end rooms, then work back toward the entrance.
        # This avoids repeatedly returning to the front hall.
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

        if self.mission_origin is not None:
            ox, oy, yaw = self.mission_origin

            axis = Marker()
            axis.header.frame_id = self.global_frame
            axis.header.stamp = now
            axis.ns = "corridor_axis"
            axis.id = 1
            axis.type = Marker.LINE_STRIP
            axis.action = Marker.ADD
            axis.scale.x = 0.08
            axis.color.r = 0.1
            axis.color.g = 0.8
            axis.color.b = 1.0
            axis.color.a = 1.0

            p0 = Point()
            p0.x = ox
            p0.y = oy
            p0.z = 0.15

            p1 = Point()
            p1.x = ox + math.cos(yaw) * 20.0
            p1.y = oy + math.sin(yaw) * 20.0
            p1.z = 0.15

            axis.points = [p0, p1]
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

            q = quaternion_from_euler(0.0, 0.0, selected.yaw)
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]

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
            text.pose.position.z = 0.8
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

    def send_candidate(self, candidate: Candidate) -> None:
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
            return

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = candidate.x
        goal.target_pose.pose.position.y = candidate.y

        q = quaternion_from_euler(0.0, 0.0, candidate.yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)

        self.current_goal = candidate
        self.current_goal_start = rospy.Time.now()
        self.last_goal_progress_time = rospy.Time.now()
        self.last_goal_distance = float("inf")

        self.publish_status(
            f"NAVIGATING_{candidate.kind}"
        )

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
        if (
            self.scan_position is None
            or not self.scan_queue
        ):
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
        self.send_candidate(candidate)

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
                        self.failed_zones.get(candidate.zone_id, 0) + 1
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
                    self.failed_zones.get(candidate.zone_id, 0) + 1
                )
            self.current_goal = None
            self.publish_status("GOAL_FAILED")

    def timer_callback(self, _event) -> None:
        if self.finished:
            return

        if self.grid is None or self.covered is None:
            self.publish_status("WAITING_FOR_MAP")
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.publish_status("WAITING_FOR_TF")
            return

        self.capture_origin(robot_pose)
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
            self.publish_markers(robot_pose, self.last_selected)
            return

        if self.scan_position is not None:
            if rospy.Time.now() >= self.scan_wait_until:
                self.send_next_scan_goal()
            self.publish_markers(robot_pose, self.last_selected)
            return

        arrays = self.map_arrays()
        if arrays is None:
            return

        selected: Optional[Candidate] = None

        if self.phase == "CORRIDOR_ADVANCE":
            selected = self.select_corridor_candidate(
                arrays,
                robot_pose,
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
                        "CORRIDOR_ADVANCE -> ROOM_SWEEP, deepest=%.2f",
                        self.max_corridor_progress,
                    )
            else:
                self.corridor_empty_cycles = 0
                self.send_candidate(selected)

        elif self.phase == "ROOM_SWEEP":
            selected = self.select_room_candidate(
                arrays,
                robot_pose,
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
                    self.phase = "FINISHED"
                    self.finished = True
                    self.finished_pub.publish(Bool(data=True))
                    self.publish_status("FINISHED")
                    rospy.loginfo(
                        "[corridor_room_explorer] exploration finished"
                    )
            else:
                self.room_empty_cycles = 0
                self.send_candidate(selected)

        self.publish_markers(robot_pose, selected)


if __name__ == "__main__":
    try:
        CorridorRoomExplorer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
