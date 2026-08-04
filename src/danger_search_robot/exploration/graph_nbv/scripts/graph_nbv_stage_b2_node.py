#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import actionlib
import cv2
import numpy as np
import rospy
import tf2_ros

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


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


class GraphNBVStageB2:
    """
    Stage B.2:
      - local graph contains both transit nodes and NBV target nodes;
      - ray-cast information gain ignores unknown cells behind walls;
      - no local target triggers global frontier relocation;
      - local exhaustion is not treated as mission completion.
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
        self.no_progress_timeout = float(
            rospy.get_param("~no_progress_timeout", 35.0)
        )
        self.progress_epsilon = float(
            rospy.get_param("~progress_epsilon", 0.20)
        )
        self.blacklist_radius = float(
            rospy.get_param("~blacklist_radius", 1.00)
        )
        self.global_empty_cycles_to_finish = int(
            rospy.get_param("~global_empty_cycles_to_finish", 10)
        )
        self.plan_period = float(
            rospy.get_param("~plan_period", 1.50)
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

        self.blacklist: List[Tuple[float, float]] = []
        self.global_empty_cycles = 0
        self.finished = False

        self.last_nodes: List[GraphNode] = []
        self.last_edges: List[Tuple[int, int]] = []
        self.last_local_selected: Optional[GraphNode] = None
        self.last_global_selected: Optional[GlobalTarget] = None
        self.last_global_frontiers: List[Tuple[float, float]] = []

        self.status_pub = rospy.Publisher(
            "~status", String, queue_size=1, latch=True
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
            rospy.Duration(self.plan_period),
            self.timer_callback,
        )

        self.publish_status("WAITING_FOR_MAP")
        self.mode_pub.publish(String(data="LOCAL_NBV"))
        rospy.loginfo(
            "[graph_nbv] Stage B.2 started dry_run=%s",
            self.dry_run,
        )

    def publish_status(self, text: str) -> None:
        self.status_pub.publish(String(data=text))

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

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
            cell = self.world_to_grid(
                x1 + ratio * (x2 - x1),
                y1 + ratio * (y2 - y1),
            )
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
        self.current_goal_start = rospy.Time.now()
        self.last_progress_time = rospy.Time.now()
        self.last_goal_distance = float("inf")

        self.mode_pub.publish(String(data="LOCAL_NBV"))
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
        self.current_goal_start = rospy.Time.now()
        self.last_progress_time = rospy.Time.now()
        self.last_goal_distance = float("inf")

        self.mode_pub.publish(
            String(data="GLOBAL_RELOCATION")
        )
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
                self.client.cancel_goal()
                self.blacklist.append(goal_xy)
                self.clear_current_goal()
                self.publish_status("GOAL_TIMEOUT")
            return

        if state == GoalStatus.SUCCEEDED:
            completed_kind = self.current_goal_kind
            rospy.loginfo(
                "[graph_nbv] %s goal reached",
                completed_kind,
            )
            self.clear_current_goal()
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
            self.blacklist.append(goal_xy)
            self.clear_current_goal()
            self.publish_status("GOAL_FAILED")

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
            self.send_global_goal(global_target)
        else:
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
                self.finished = True
                self.finished_pub.publish(Bool(data=True))
                self.publish_status("FINISHED")
                rospy.loginfo(
                    "[graph_nbv] no local or global frontier; "
                    "exploration finished"
                )

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
        GraphNBVStageB2()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
