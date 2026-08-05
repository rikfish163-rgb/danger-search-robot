#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler


class RoomEntryDetector:
    """
    第一版房门/房间入口候选检测器。

    核心思路：
    1. 在 OccupancyGrid 的自由空间中寻找“窄通道”；
    2. 窄通道两侧由墙体限制，宽度类似房门；
    3. 沿通道方向走一段距离后，自由空间明显变宽；
    4. 将这个位置认为是“房门/入口候选”；
    5. 只发布可视化和候选目标，不控制机器人。
    """

    def __init__(self):
        rospy.init_node("room_entry_detector")

        self.map_topic = rospy.get_param("~map_topic", "/map_confirmed")

        # OccupancyGrid 中 >= 此值认为是障碍
        self.occupied_threshold = rospy.get_param(
            "~occupied_threshold", 50
        )

        # 房门宽度范围，单位 m
        self.door_min_width = rospy.get_param(
            "~door_min_width", 0.50
        )
        self.door_max_width = rospy.get_param(
            "~door_max_width", 1.40
        )

        # 从门中心向两侧探测多少距离来判断空间是否变宽
        self.probe_distance = rospy.get_param(
            "~probe_distance", 1.20
        )

        # 门后空间至少多宽，才认为可能连接到较大空间
        self.room_min_width = rospy.get_param(
            "~room_min_width", 1.80
        )

        # 相比门宽，后方空间至少增加多少
        self.room_widening = rospy.get_param(
            "~room_widening", 0.60
        )

        # 门前后必须至少有这么长的已知自由空间
        self.free_run = rospy.get_param(
            "~free_run", 0.60
        )

        # 生成的“门内观察点”离门中心多少米
        self.entry_offset = rospy.get_param(
            "~entry_offset", 1.20
        )

        # 候选点之间太近时合并
        self.cluster_radius = rospy.get_param(
            "~cluster_radius", 1.20
        )

        # 多久重新分析一次地图
        self.process_period = rospy.get_param(
            "~process_period", 3.0
        )

        self.last_process = rospy.Time(0)

        self.pose_pub = rospy.Publisher(
            "/room_entry_candidates",
            PoseArray,
            queue_size=1,
            latch=True
        )

        self.marker_pub = rospy.Publisher(
            "/room_entry_markers",
            MarkerArray,
            queue_size=1,
            latch=True
        )

        rospy.Subscriber(
            self.map_topic,
            OccupancyGrid,
            self.map_callback,
            queue_size=1
        )

        rospy.loginfo(
            "[room_entry_detector] started, map=%s",
            self.map_topic
        )

    def is_free(self, grid, r, c):
        if r < 0 or r >= grid.shape[0]:
            return False
        if c < 0 or c >= grid.shape[1]:
            return False

        value = grid[r, c]

        return 0 <= value < self.occupied_threshold

    def free_line(self, grid, r, c, dr, dc, cells):
        """
        检查指定方向上一段距离是否都是已知自由空间。
        """
        for i in range(1, cells + 1):
            rr = r + dr * i
            cc = c + dc * i

            if not self.is_free(grid, rr, cc):
                return False

        return True

    def clearance_to_wall(
        self,
        grid,
        r,
        c,
        dr,
        dc,
        max_cells
    ):
        """
        从某个自由格沿方向寻找最近墙体。

        返回：
        - 正整数：距离墙体的格数
        - max_cells：一直是自由空间，没有在范围内遇到墙
        - None：遇到未知区域或越界
        """
        for i in range(1, max_cells + 1):
            rr = r + dr * i
            cc = c + dc * i

            if (
                rr < 0
                or rr >= grid.shape[0]
                or cc < 0
                or cc >= grid.shape[1]
            ):
                return None

            value = grid[rr, cc]

            if value < 0:
                return None

            if value >= self.occupied_threshold:
                return i

        return max_cells

    def lateral_width(
        self,
        grid,
        r,
        c,
        lateral_dr,
        lateral_dc,
        resolution,
        max_cells
    ):
        """
        计算当前位置横向可用宽度。
        """

        d1 = self.clearance_to_wall(
            grid,
            r,
            c,
            lateral_dr,
            lateral_dc,
            max_cells
        )

        d2 = self.clearance_to_wall(
            grid,
            r,
            c,
            -lateral_dr,
            -lateral_dc,
            max_cells
        )

        if d1 is None or d2 is None:
            return None

        return (d1 + d2) * resolution

    def check_candidate(
        self,
        grid,
        r,
        c,
        passage_dr,
        passage_dc,
        resolution
    ):
        """
        检查一个格子是否像门。

        passage_dr / passage_dc：
        表示机器人穿门时的运动方向。

        横向方向则用于测量“门宽”。
        """

        if not self.is_free(grid, r, c):
            return None

        lateral_dr = -passage_dc
        lateral_dc = passage_dr

        door_search_cells = max(
            1,
            int(math.ceil(
                self.door_max_width / resolution
            ))
        )

        door_width = self.lateral_width(
            grid,
            r,
            c,
            lateral_dr,
            lateral_dc,
            resolution,
            door_search_cells
        )

        if door_width is None:
            return None

        if not (
            self.door_min_width
            <= door_width
            <= self.door_max_width
        ):
            return None

        free_cells = max(
            1,
            int(round(self.free_run / resolution))
        )

        # 门的两边必须都有连续自由空间
        if not self.free_line(
            grid,
            r,
            c,
            passage_dr,
            passage_dc,
            free_cells
        ):
            return None

        if not self.free_line(
            grid,
            r,
            c,
            -passage_dr,
            -passage_dc,
            free_cells
        ):
            return None

        probe_cells = max(
            1,
            int(round(
                self.probe_distance / resolution
            ))
        )

        room_search_cells = max(
            1,
            int(round(3.0 / resolution))
        )

        results = []

        # 正方向和反方向分别观察空间是否明显变宽
        for sign in (1, -1):

            pr = r + sign * passage_dr * probe_cells
            pc = c + sign * passage_dc * probe_cells

            if not self.is_free(grid, pr, pc):
                continue

            width = self.lateral_width(
                grid,
                pr,
                pc,
                lateral_dr,
                lateral_dc,
                resolution,
                room_search_cells
            )

            if width is None:
                continue

            widening = width - door_width

            if (
                width >= self.room_min_width
                and widening >= self.room_widening
            ):
                results.append(
                    (
                        widening,
                        width,
                        sign * passage_dr,
                        sign * passage_dc
                    )
                )

        if not results:
            return None

        # 哪一侧空间更大，就暂时认为哪一侧更像“房间内部”
        results.sort(
            key=lambda x: x[0],
            reverse=True
        )

        widening, wide_width, room_dr, room_dc = results[0]

        return {
            "r": r,
            "c": c,
            "door_width": door_width,
            "wide_width": wide_width,
            "room_dr": room_dr,
            "room_dc": room_dc,
            "score": widening
        }

    def grid_to_world(
        self,
        row,
        col,
        resolution,
        origin_x,
        origin_y,
        origin_yaw
    ):
        local_x = (col + 0.5) * resolution
        local_y = (row + 0.5) * resolution

        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)

        world_x = (
            origin_x
            + cos_yaw * local_x
            - sin_yaw * local_y
        )

        world_y = (
            origin_y
            + sin_yaw * local_x
            + cos_yaw * local_y
        )

        return world_x, world_y

    def cluster_candidates(
        self,
        candidates,
        resolution
    ):
        """
        同一个门会产生很多相邻候选点。
        按得分排序后进行简单空间去重。
        """

        candidates.sort(
            key=lambda x: x["score"],
            reverse=True
        )

        selected = []

        radius_cells = (
            self.cluster_radius / resolution
        )

        radius_sq = radius_cells * radius_cells

        for candidate in candidates:

            too_close = False

            for existing in selected:

                dr = candidate["r"] - existing["r"]
                dc = candidate["c"] - existing["c"]

                if dr * dr + dc * dc < radius_sq:
                    too_close = True
                    break

            if not too_close:
                selected.append(candidate)

        return selected

    def publish_results(
        self,
        msg,
        candidates,
        resolution,
        origin_x,
        origin_y,
        origin_yaw
    ):

        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = msg.header.frame_id

        markers = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        marker_id = 0

        for i, candidate in enumerate(candidates):

            door_x, door_y = self.grid_to_world(
                candidate["r"],
                candidate["c"],
                resolution,
                origin_x,
                origin_y,
                origin_yaw
            )

            # grid 中的房间方向
            grid_dx = candidate["room_dc"]
            grid_dy = candidate["room_dr"]

            # 转为世界坐标方向
            cos_yaw = math.cos(origin_yaw)
            sin_yaw = math.sin(origin_yaw)

            world_dx = (
                cos_yaw * grid_dx
                - sin_yaw * grid_dy
            )

            world_dy = (
                sin_yaw * grid_dx
                + cos_yaw * grid_dy
            )

            norm = math.hypot(
                world_dx,
                world_dy
            )

            if norm < 1e-6:
                continue

            world_dx /= norm
            world_dy /= norm

            entry_x = (
                door_x
                + world_dx * self.entry_offset
            )

            entry_y = (
                door_y
                + world_dy * self.entry_offset
            )

            heading = math.atan2(
                world_dy,
                world_dx
            )

            q = quaternion_from_euler(
                0.0,
                0.0,
                heading
            )

            pose = Pose()

            pose.position.x = entry_x
            pose.position.y = entry_y
            pose.position.z = 0.0

            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            pose_array.poses.append(pose)

            # 门中心：球
            door_marker = Marker()
            door_marker.header.frame_id = msg.header.frame_id
            door_marker.header.stamp = rospy.Time.now()

            door_marker.ns = "door_candidates"
            door_marker.id = marker_id
            marker_id += 1

            door_marker.type = Marker.SPHERE
            door_marker.action = Marker.ADD

            door_marker.pose.position.x = door_x
            door_marker.pose.position.y = door_y
            door_marker.pose.position.z = 0.12

            door_marker.pose.orientation.w = 1.0

            door_marker.scale.x = 0.30
            door_marker.scale.y = 0.30
            door_marker.scale.z = 0.30

            door_marker.color.r = 1.0
            door_marker.color.g = 0.6
            door_marker.color.b = 0.0
            door_marker.color.a = 0.9

            markers.markers.append(
                door_marker
            )

            # 箭头：门 -> 预测房间内部
            arrow = Marker()
            arrow.header.frame_id = msg.header.frame_id
            arrow.header.stamp = rospy.Time.now()

            arrow.ns = "room_directions"
            arrow.id = marker_id
            marker_id += 1

            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD

            p0 = Point()
            p0.x = door_x
            p0.y = door_y
            p0.z = 0.20

            p1 = Point()
            p1.x = entry_x
            p1.y = entry_y
            p1.z = 0.20

            arrow.points.append(p0)
            arrow.points.append(p1)

            arrow.scale.x = 0.08
            arrow.scale.y = 0.16
            arrow.scale.z = 0.20

            arrow.color.r = 0.0
            arrow.color.g = 1.0
            arrow.color.b = 0.2
            arrow.color.a = 0.95

            markers.markers.append(
                arrow
            )

            # 门内目标：立方体
            goal_marker = Marker()
            goal_marker.header.frame_id = msg.header.frame_id
            goal_marker.header.stamp = rospy.Time.now()

            goal_marker.ns = "room_entry_goals"
            goal_marker.id = marker_id
            marker_id += 1

            goal_marker.type = Marker.CUBE
            goal_marker.action = Marker.ADD

            goal_marker.pose = pose
            goal_marker.pose.position.z = 0.15

            goal_marker.scale.x = 0.30
            goal_marker.scale.y = 0.30
            goal_marker.scale.z = 0.30

            goal_marker.color.r = 0.0
            goal_marker.color.g = 0.4
            goal_marker.color.b = 1.0
            goal_marker.color.a = 0.95

            markers.markers.append(
                goal_marker
            )

            # 编号
            text = Marker()
            text.header.frame_id = msg.header.frame_id
            text.header.stamp = rospy.Time.now()

            text.ns = "room_entry_labels"
            text.id = marker_id
            marker_id += 1

            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD

            text.pose.position.x = entry_x
            text.pose.position.y = entry_y
            text.pose.position.z = 0.55

            text.pose.orientation.w = 1.0

            text.scale.z = 0.30

            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0

            text.text = "Room %d" % (i + 1)

            markers.markers.append(
                text
            )

        self.pose_pub.publish(
            pose_array
        )

        self.marker_pub.publish(
            markers
        )

        rospy.loginfo(
            "[room_entry_detector] detected %d candidates",
            len(pose_array.poses)
        )

    def map_callback(self, msg):

        now = rospy.Time.now()

        if (
            now - self.last_process
        ).to_sec() < self.process_period:
            return

        self.last_process = now

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        if width == 0 or height == 0:
            return

        grid = np.array(
            msg.data,
            dtype=np.int16
        ).reshape(
            (height, width)
        )

        known = np.argwhere(
            grid >= 0
        )

        if known.size == 0:
            return

        r_min = max(
            0,
            int(np.min(known[:, 0]))
        )

        r_max = min(
            height - 1,
            int(np.max(known[:, 0]))
        )

        c_min = max(
            0,
            int(np.min(known[:, 1]))
        )

        c_max = min(
            width - 1,
            int(np.max(known[:, 1]))
        )

        # 每约 20 cm 取一个检测点，减少计算量
        sample_step = max(
            1,
            int(round(
                0.20 / resolution
            ))
        )

        candidates = []

        for r in range(
            r_min,
            r_max + 1,
            sample_step
        ):

            for c in range(
                c_min,
                c_max + 1,
                sample_step
            ):

                if not self.is_free(
                    grid,
                    r,
                    c
                ):
                    continue

                # 穿门方向：X
                result_x = self.check_candidate(
                    grid,
                    r,
                    c,
                    0,
                    1,
                    resolution
                )

                if result_x is not None:
                    candidates.append(
                        result_x
                    )

                # 穿门方向：Y
                result_y = self.check_candidate(
                    grid,
                    r,
                    c,
                    1,
                    0,
                    resolution
                )

                if result_y is not None:
                    candidates.append(
                        result_y
                    )

        candidates = self.cluster_candidates(
            candidates,
            resolution
        )

        origin = msg.info.origin

        q = origin.orientation

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        origin_yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        self.publish_results(
            msg,
            candidates,
            resolution,
            origin.position.x,
            origin.position.y,
            origin_yaw
        )


if __name__ == "__main__":
    try:
        RoomEntryDetector()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
