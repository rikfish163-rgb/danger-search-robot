#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

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
class Viewpoint:
    index: int
    row: int
    col: int
    x: float
    y: float
    clearance: float
    unknown_gain: float
    frontier_gain: float
    path_cost: float = float("inf")
    revisit: float = 0.0
    utility: float = -float("inf")


class GraphNBVNode:
    """
    Stage-B map-only graph NBV explorer.

    Inputs:
      - OccupancyGrid map
      - TF global_frame -> base_frame

    Processing:
      1. Sample safe viewpoints in a local radius.
      2. Build a collision-free local graph.
      3. Run Dijkstra from the robot node.
      4. Estimate map information gain around each viewpoint.
      5. Score reachable viewpoints.
      6. Publish the best viewpoint or send it to move_base.

    This stage intentionally does not contain:
      - corridor-axis assumptions;
      - room/door segmentation;
      - visual coverage gain;
      - global branch persistence.
    """

    def __init__(self) -> None:
        rospy.init_node("graph_nbv")

        self.map_topic = rospy.get_param("~map_topic", "/map_confirmed")
        self.global_frame = rospy.get_param("~global_frame", "map_level")
        self.base_frame = rospy.get_param("~base_frame", "body")
        self.move_base_action = rospy.get_param(
            "~move_base_action", "/move_base"
        )
        self.dry_run = bool(rospy.get_param("~dry_run", True))

        self.occupied_threshold = int(
            rospy.get_param("~occupied_threshold", 50)
        )
        self.local_sampling_radius = float(
            rospy.get_param("~local_sampling_radius", 6.0)
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
        self.max_candidate_count = int(
            rospy.get_param("~max_candidate_count", 240)
        )

        self.information_radius = float(
            rospy.get_param("~information_radius", 3.50)
        )
        self.frontier_radius = float(
            rospy.get_param("~frontier_radius", 2.20)
        )
        self.min_unknown_gain_m2 = float(
            rospy.get_param("~min_unknown_gain_m2", 0.35)
        )

        self.weight_unknown_gain = float(
            rospy.get_param("~weight_unknown_gain", 2.20)
        )
        self.weight_frontier_gain = float(
            rospy.get_param("~weight_frontier_gain", 0.12)
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

        self.coverage_radius = float(
            rospy.get_param("~coverage_radius", 0.90)
        )
        self.goal_min_distance = float(
            rospy.get_param("~goal_min_distance", 0.70)
        )
        self.goal_max_distance = float(
            rospy.get_param("~goal_max_distance", 9.0)
        )
        self.goal_timeout = float(
            rospy.get_param("~goal_timeout", 90.0)
        )
        self.no_progress_timeout = float(
            rospy.get_param("~no_progress_timeout", 30.0)
        )
        self.progress_epsilon = float(
            rospy.get_param("~progress_epsilon", 0.20)
        )
        self.blacklist_radius = float(
            rospy.get_param("~blacklist_radius", 1.00)
        )
        self.empty_cycles_to_finish = int(
            rospy.get_param("~empty_cycles_to_finish", 8)
        )
        self.plan_period = float(
            rospy.get_param("~plan_period", 1.5)
        )

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rospy.Duration(20.0)
        )
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer
        )

        self.client = actionlib.SimpleActionClient(
            self.move_base_action,
            MoveBaseAction,
        )

        self.grid: Optional[np.ndarray] = None
        self.covered: Optional[np.ndarray] = None
        self.map_width = 0
        self.map_height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0

        self.current_goal: Optional[Viewpoint] = None
        self.current_goal_start = rospy.Time(0)
        self.last_goal_distance = float("inf")
        self.last_progress_time = rospy.Time(0)

        self.blacklist: List[Tuple[float, float]] = []
        self.empty_cycles = 0
        self.finished = False
        self.last_selected: Optional[Viewpoint] = None
        self.last_viewpoints: List[Viewpoint] = []
        self.last_edges: List[Tuple[int, int]] = []

        self.status_pub = rospy.Publisher(
            "~status",
            String,
            queue_size=1,
            latch=True,
        )
        self.selected_goal_pub = rospy.Publisher(
            "~selected_goal",
            Marker,
            queue_size=1,
            latch=True,
        )
        self.marker_pub = rospy.Publisher(
            "~markers",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.finished_pub = rospy.Publisher(
            "~finished",
            Bool,
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
                "[graph_nbv] waiting for move_base action: %s",
                self.move_base_action,
            )
            self.client.wait_for_server()
            rospy.loginfo("[graph_nbv] move_base is available")

        rospy.Timer(
            rospy.Duration(self.plan_period),
            self.timer_callback,
        )

        self.publish_status("WAITING_FOR_MAP")
        rospy.loginfo(
            "[graph_nbv] started dry_run=%s map=%s frame=%s base=%s",
            self.dry_run,
            self.map_topic,
            self.global_frame,
            self.base_frame,
        )

    def publish_status(self, value: str) -> None:
        self.status_pub.publish(String(data=value))

    def map_callback(self, msg: OccupancyGrid) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)

        if width <= 0 or height <= 0 or resolution <= 0.0:
            return

        data = np.asarray(msg.data, dtype=np.int16)
        if data.size != width * height:
            rospy.logwarn_throttle(
                5.0,
                "[graph_nbv] invalid map data size",
            )
            return

        new_grid = data.reshape((height, width))

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

        self.grid = new_grid
        self.map_width = width
        self.map_height = height
        self.resolution = resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.origin_yaw = origin_yaw

        if geometry_changed:
            self.covered = np.zeros(
                (height, width),
                dtype=np.uint8,
            )
            rospy.loginfo(
                "[graph_nbv] map geometry %dx%d resolution=%.3f",
                width,
                height,
                resolution,
            )

    def get_robot_pose(
        self,
    ) -> Optional[Tuple[float, float, float]]:
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
                "[graph_nbv] TF unavailable: %s",
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

        cell = self.world_to_grid(
            robot_pose[0],
            robot_pose[1],
        )
        if cell is None:
            return

        row, col = cell
        radius_cells = max(
            1,
            int(round(
                self.coverage_radius / self.resolution
            )),
        )
        cv2.circle(
            self.covered,
            (col, row),
            radius_cells,
            1,
            thickness=-1,
        )

    def build_map_layers(
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

        kernel = np.ones((3, 3), dtype=np.uint8)
        unknown_touch = cv2.dilate(
            unknown,
            kernel,
            iterations=1,
        )
        frontier = (
            (known_free > 0)
            & (unknown_touch > 0)
        ).astype(np.uint8)

        info_radius_cells = max(
            1,
            int(round(
                self.information_radius / self.resolution
            )),
        )
        info_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (
                2 * info_radius_cells + 1,
                2 * info_radius_cells + 1,
            ),
        ).astype(np.float32)

        unknown_count = cv2.filter2D(
            unknown.astype(np.float32),
            cv2.CV_32F,
            info_kernel,
            borderType=cv2.BORDER_CONSTANT,
        )
        unknown_gain_m2 = (
            unknown_count
            * self.resolution
            * self.resolution
        )

        frontier_radius_cells = max(
            1,
            int(round(
                self.frontier_radius / self.resolution
            )),
        )
        frontier_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (
                2 * frontier_radius_cells + 1,
                2 * frontier_radius_cells + 1,
            ),
        ).astype(np.float32)

        frontier_gain = cv2.filter2D(
            frontier.astype(np.float32),
            cv2.CV_32F,
            frontier_kernel,
            borderType=cv2.BORDER_CONSTANT,
        )

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
            "unknown_gain": unknown_gain_m2,
            "frontier_gain": frontier_gain,
            "coverage_distance": coverage_distance,
        }

    def sample_viewpoints(
        self,
        layers: Dict[str, np.ndarray],
        robot_pose: Tuple[float, float, float],
    ) -> List[Viewpoint]:
        center = self.world_to_grid(
            robot_pose[0],
            robot_pose[1],
        )
        if center is None:
            return []

        center_row, center_col = center
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

        viewpoints: List[Viewpoint] = []

        row_start = row_min + (
            (center_row - row_min) % spacing_cells
        )
        col_start = col_min + (
            (center_col - col_min) % spacing_cells
        )

        for row in range(
            row_start,
            row_max + 1,
            spacing_cells,
        ):
            for col in range(
                col_start,
                col_max + 1,
                spacing_cells,
            ):
                if not layers["safe"][row, col]:
                    continue

                x, y = self.grid_to_world(row, col)
                distance = math.hypot(
                    x - robot_pose[0],
                    y - robot_pose[1],
                )

                if distance > self.local_sampling_radius:
                    continue

                unknown_gain = float(
                    layers["unknown_gain"][row, col]
                )
                if unknown_gain < self.min_unknown_gain_m2:
                    continue

                frontier_gain = float(
                    layers["frontier_gain"][row, col]
                )

                coverage_distance = float(
                    layers["coverage_distance"][row, col]
                )
                revisit = max(
                    0.0,
                    self.coverage_radius - coverage_distance,
                )

                viewpoints.append(
                    Viewpoint(
                        index=len(viewpoints),
                        row=row,
                        col=col,
                        x=x,
                        y=y,
                        clearance=float(
                            layers["clearance"][row, col]
                        ),
                        unknown_gain=unknown_gain,
                        frontier_gain=frontier_gain,
                        revisit=revisit,
                    )
                )

        viewpoints.sort(
            key=lambda node: (
                node.unknown_gain,
                node.frontier_gain,
                node.clearance,
            ),
            reverse=True,
        )

        viewpoints = viewpoints[
            : self.max_candidate_count
        ]

        for index, viewpoint in enumerate(viewpoints):
            viewpoint.index = index

        return viewpoints

    def edge_is_free(
        self,
        start_x: float,
        start_y: float,
        end_x: float,
        end_y: float,
        layers: Dict[str, np.ndarray],
    ) -> bool:
        distance = math.hypot(
            end_x - start_x,
            end_y - start_y,
        )
        sample_count = max(
            2,
            int(math.ceil(
                distance / self.edge_collision_step
            )),
        )

        for index in range(sample_count + 1):
            ratio = index / sample_count
            x = start_x + ratio * (end_x - start_x)
            y = start_y + ratio * (end_y - start_y)

            cell = self.world_to_grid(x, y)
            if cell is None:
                return False

            row, col = cell
            if not layers["safe"][row, col]:
                return False

        return True

    def build_graph(
        self,
        viewpoints: List[Viewpoint],
        robot_pose: Tuple[float, float, float],
        layers: Dict[str, np.ndarray],
    ) -> Tuple[
        Dict[int, List[Tuple[int, float]]],
        List[Tuple[int, int]],
    ]:
        robot_index = len(viewpoints)
        adjacency: Dict[int, List[Tuple[int, float]]] = {
            index: []
            for index in range(robot_index + 1)
        }
        edges: List[Tuple[int, int]] = []

        positions = [
            (viewpoint.x, viewpoint.y)
            for viewpoint in viewpoints
        ]
        positions.append(
            (robot_pose[0], robot_pose[1])
        )

        for first in range(len(positions)):
            for second in range(first + 1, len(positions)):
                x1, y1 = positions[first]
                x2, y2 = positions[second]
                distance = math.hypot(
                    x2 - x1,
                    y2 - y1,
                )

                if distance > self.edge_max_length:
                    continue

                if not self.edge_is_free(
                    x1,
                    y1,
                    x2,
                    y2,
                    layers,
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

        queue: List[Tuple[float, int]] = [
            (0.0, start)
        ]

        while queue:
            current_distance, node = heapq.heappop(queue)

            if current_distance > distances[node]:
                continue

            for neighbor, edge_cost in adjacency[node]:
                new_distance = (
                    current_distance + edge_cost
                )
                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    heapq.heappush(
                        queue,
                        (new_distance, neighbor),
                    )

        return distances

    def is_blacklisted(
        self,
        x: float,
        y: float,
    ) -> bool:
        return any(
            math.hypot(x - bx, y - by)
            < self.blacklist_radius
            for bx, by in self.blacklist
        )

    def score_viewpoints(
        self,
        viewpoints: List[Viewpoint],
        distances: Dict[int, float],
        robot_pose: Tuple[float, float, float],
    ) -> Optional[Viewpoint]:
        best: Optional[Viewpoint] = None

        for viewpoint in viewpoints:
            path_cost = distances.get(
                viewpoint.index,
                float("inf"),
            )
            viewpoint.path_cost = path_cost

            if not math.isfinite(path_cost):
                continue

            straight_distance = math.hypot(
                viewpoint.x - robot_pose[0],
                viewpoint.y - robot_pose[1],
            )
            if (
                straight_distance < self.goal_min_distance
                or straight_distance > self.goal_max_distance
            ):
                continue

            if self.is_blacklisted(
                viewpoint.x,
                viewpoint.y,
            ):
                continue

            viewpoint.utility = (
                self.weight_unknown_gain
                * viewpoint.unknown_gain
                + self.weight_frontier_gain
                * viewpoint.frontier_gain
                + self.weight_clearance
                * min(viewpoint.clearance, 1.50)
                - self.weight_path_cost
                * viewpoint.path_cost
                - self.weight_revisit
                * viewpoint.revisit
            )

            if (
                best is None
                or viewpoint.utility > best.utility
            ):
                best = viewpoint

        return best

    def send_goal(
        self,
        viewpoint: Viewpoint,
        robot_pose: Tuple[float, float, float],
    ) -> None:
        self.last_selected = viewpoint

        yaw = math.atan2(
            viewpoint.y - robot_pose[1],
            viewpoint.x - robot_pose[0],
        )

        rospy.loginfo(
            "[graph_nbv] selected x=%.2f y=%.2f "
            "utility=%.2f unknown=%.2f m2 frontier=%.0f "
            "clearance=%.2f path=%.2f revisit=%.2f "
            "dry_run=%s",
            viewpoint.x,
            viewpoint.y,
            viewpoint.utility,
            viewpoint.unknown_gain,
            viewpoint.frontier_gain,
            viewpoint.clearance,
            viewpoint.path_cost,
            viewpoint.revisit,
            self.dry_run,
        )

        if self.dry_run:
            self.publish_status("DRY_RUN_VIEWPOINT_SELECTED")
            return

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = viewpoint.x
        goal.target_pose.pose.position.y = viewpoint.y

        quaternion = quaternion_from_euler(
            0.0,
            0.0,
            yaw,
        )
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        self.client.send_goal(goal)

        self.current_goal = viewpoint
        self.current_goal_start = rospy.Time.now()
        self.last_progress_time = rospy.Time.now()
        self.last_goal_distance = float("inf")
        self.publish_status("NAVIGATING_TO_VIEWPOINT")

    def handle_goal(
        self,
        robot_pose: Tuple[float, float, float],
    ) -> None:
        if self.current_goal is None:
            return

        state = self.client.get_state()
        now = rospy.Time.now()

        if state in (
            GoalStatus.PENDING,
            GoalStatus.ACTIVE,
        ):
            distance = math.hypot(
                self.current_goal.x - robot_pose[0],
                self.current_goal.y - robot_pose[1],
            )

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
                    "[graph_nbv] goal timeout distance=%.2f "
                    "total=%.1f no_progress=%.1f",
                    distance,
                    total_time,
                    no_progress_time,
                )
                self.client.cancel_goal()
                self.blacklist.append(
                    (
                        self.current_goal.x,
                        self.current_goal.y,
                    )
                )
                self.current_goal = None
                self.publish_status("GOAL_TIMEOUT")
            return

        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("[graph_nbv] viewpoint reached")
            self.current_goal = None
            self.empty_cycles = 0
            self.publish_status("VIEWPOINT_REACHED")
            return

        if state in (
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.PREEMPTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        ):
            rospy.logwarn(
                "[graph_nbv] goal failed state=%d",
                state,
            )
            self.blacklist.append(
                (
                    self.current_goal.x,
                    self.current_goal.y,
                )
            )
            self.current_goal = None
            self.publish_status("GOAL_FAILED")

    def publish_markers(
        self,
        viewpoints: List[Viewpoint],
        edges: List[Tuple[int, int]],
        robot_pose: Tuple[float, float, float],
        selected: Optional[Viewpoint],
    ) -> None:
        markers = MarkerArray()
        now = rospy.Time.now()

        clear = Marker()
        clear.header.frame_id = self.global_frame
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        edge_marker = Marker()
        edge_marker.header.frame_id = self.global_frame
        edge_marker.header.stamp = now
        edge_marker.ns = "graph_edges"
        edge_marker.id = 1
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.025
        edge_marker.color.r = 0.25
        edge_marker.color.g = 0.65
        edge_marker.color.b = 1.0
        edge_marker.color.a = 0.40

        positions = [
            (viewpoint.x, viewpoint.y)
            for viewpoint in viewpoints
        ]
        positions.append(
            (robot_pose[0], robot_pose[1])
        )

        for first, second in edges:
            for index in (first, second):
                point = Point()
                point.x = positions[index][0]
                point.y = positions[index][1]
                point.z = 0.10
                edge_marker.points.append(point)

        markers.markers.append(edge_marker)

        node_marker = Marker()
        node_marker.header.frame_id = self.global_frame
        node_marker.header.stamp = now
        node_marker.ns = "graph_nodes"
        node_marker.id = 2
        node_marker.type = Marker.SPHERE_LIST
        node_marker.action = Marker.ADD
        node_marker.scale.x = 0.10
        node_marker.scale.y = 0.10
        node_marker.scale.z = 0.10
        node_marker.color.r = 0.20
        node_marker.color.g = 0.90
        node_marker.color.b = 1.0
        node_marker.color.a = 0.75

        for viewpoint in viewpoints:
            if not math.isfinite(viewpoint.path_cost):
                continue
            point = Point()
            point.x = viewpoint.x
            point.y = viewpoint.y
            point.z = 0.12
            node_marker.points.append(point)

        markers.markers.append(node_marker)

        if selected is not None:
            goal_marker = Marker()
            goal_marker.header.frame_id = self.global_frame
            goal_marker.header.stamp = now
            goal_marker.ns = "selected_viewpoint"
            goal_marker.id = 3
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = selected.x
            goal_marker.pose.position.y = selected.y
            goal_marker.pose.position.z = 0.25
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = 0.35
            goal_marker.scale.y = 0.35
            goal_marker.scale.z = 0.35
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.25
            goal_marker.color.b = 0.10
            goal_marker.color.a = 1.0
            markers.markers.append(goal_marker)

            text_marker = Marker()
            text_marker.header.frame_id = self.global_frame
            text_marker.header.stamp = now
            text_marker.ns = "selected_viewpoint_text"
            text_marker.id = 4
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = selected.x
            text_marker.pose.position.y = selected.y
            text_marker.pose.position.z = 0.75
            text_marker.scale.z = 0.28
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = (
                f"U={selected.utility:.1f}\n"
                f"G={selected.unknown_gain:.1f}m2 "
                f"L={selected.path_cost:.1f}m"
            )
            markers.markers.append(text_marker)

        self.marker_pub.publish(markers)

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

        self.mark_coverage(robot_pose)

        if self.current_goal is not None:
            self.handle_goal(robot_pose)
            self.publish_markers(
                self.last_viewpoints,
                self.last_edges,
                robot_pose,
                self.last_selected,
            )
            return

        layers = self.build_map_layers()
        if layers is None:
            return

        viewpoints = self.sample_viewpoints(
            layers,
            robot_pose,
        )

        if not viewpoints:
            self.empty_cycles += 1
            self.publish_status("NO_VIEWPOINT_CANDIDATES")
            rospy.logwarn_throttle(
                3.0,
                "[graph_nbv] no viewpoint candidates cycle=%d/%d",
                self.empty_cycles,
                self.empty_cycles_to_finish,
            )
            if self.empty_cycles >= self.empty_cycles_to_finish:
                self.finished = True
                self.finished_pub.publish(Bool(data=True))
                self.publish_status("FINISHED")
            return

        adjacency, edges = self.build_graph(
            viewpoints,
            robot_pose,
            layers,
        )

        robot_index = len(viewpoints)
        distances = self.dijkstra(
            adjacency,
            robot_index,
        )

        selected = self.score_viewpoints(
            viewpoints,
            distances,
            robot_pose,
        )

        self.last_viewpoints = viewpoints
        self.last_edges = edges
        self.last_selected = selected

        if selected is None:
            self.empty_cycles += 1
            self.publish_status("NO_REACHABLE_VIEWPOINT")
            rospy.logwarn_throttle(
                3.0,
                "[graph_nbv] no reachable viewpoint cycle=%d/%d",
                self.empty_cycles,
                self.empty_cycles_to_finish,
            )
            if self.empty_cycles >= self.empty_cycles_to_finish:
                self.finished = True
                self.finished_pub.publish(Bool(data=True))
                self.publish_status("FINISHED")
        else:
            self.empty_cycles = 0
            self.send_goal(selected, robot_pose)

        self.publish_markers(
            viewpoints,
            edges,
            robot_pose,
            selected,
        )


if __name__ == "__main__":
    try:
        GraphNBVNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
