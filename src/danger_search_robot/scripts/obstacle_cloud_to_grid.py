#!/usr/bin/env python3

import math

import numpy as np
import rospy
import tf

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class ObstacleCloudToGrid:
    def __init__(self):

        self.input_topic = rospy.get_param(
            "~input_topic",
            "/cloud_obstacles"
        )

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/map_2d"
        )

        self.map_frame = rospy.get_param(
            "~map_frame",
            "map_level"
        )

        self.resolution = rospy.get_param(
            "~resolution",
            0.10
        )

        self.map_width_m = rospy.get_param(
            "~map_width",
            80.0
        )

        self.map_height_m = rospy.get_param(
            "~map_height",
            80.0
        )

        self.origin_x = -self.map_width_m / 2.0
        self.origin_y = -self.map_height_m / 2.0

        self.width = int(
            self.map_width_m / self.resolution
        )

        self.height = int(
            self.map_height_m / self.resolution
        )

        # ==================================================
        # 占据证据地图
        #
        # score > 0：越来越像障碍
        # score < 0：越来越像自由空间
        # ==================================================

        self.score = np.zeros(
            (self.height, self.width),
            dtype=np.int16
        )

        self.observed = np.zeros(
            (self.height, self.width),
            dtype=bool
        )

        # 一次有效障碍观测增加的分数
        self.hit_increment = rospy.get_param(
            "~hit_increment",
            3
        )

        # 自由射线经过时降低的分数
        self.miss_decrement = rospy.get_param(
            "~miss_decrement",
            1
        )

        # 必须累计到这个分数才真正成为occupied
        self.occupied_threshold = rospy.get_param(
            "~occupied_threshold",
            6
        )

        # score上下限，避免无限增长
        self.max_score = rospy.get_param(
            "~max_score",
            20
        )

        self.min_score = rospy.get_param(
            "~min_score",
            -20
        )

        # 同一个0.1m栅格内至少多少个点，
        # 才认为这一帧真的检测到了障碍。
        self.min_points_per_cell = rospy.get_param(
            "~min_points_per_cell",
            2
        )

        # 点云高度范围
        self.min_relative_height = rospy.get_param(
            "~min_relative_height",
            -0.30
        )

        self.max_relative_height = rospy.get_param(
            "~max_relative_height",
            1.80
        )

        # 暂时限制较远的点，减少远距离离群点。
        self.max_range = rospy.get_param(
            "~max_range",
            10.0
        )

        # 机器人实际能经过的区域必定是自由空间。
        # 在机器人周围这个半径内主动清理假障碍。
        self.robot_clear_radius = rospy.get_param(
            "~robot_clear_radius",
            0.55
        )

        self.robot_clear_strength = rospy.get_param(
            "~robot_clear_strength",
            5
        )

        self.tf_listener = tf.TransformListener()

        self.pub = rospy.Publisher(
            self.output_topic,
            OccupancyGrid,
            queue_size=1,
            latch=True
        )

        rospy.Subscriber(
            self.input_topic,
            PointCloud2,
            self.callback,
            queue_size=1
        )

        self.publish_every_n_scans = rospy.get_param(
            "~publish_every_n_scans",
            5
        )

        self.scan_counter = 0

        rospy.loginfo(
            "=========================================="
        )
        rospy.loginfo(
            "Evidence-based 2D OccupancyGrid started"
        )
        rospy.loginfo(
            "Input  : %s",
            self.input_topic
        )
        rospy.loginfo(
            "Output : %s",
            self.output_topic
        )
        rospy.loginfo(
            "resolution = %.2f m",
            self.resolution
        )
        rospy.loginfo(
            "occupied threshold = %d",
            self.occupied_threshold
        )
        rospy.loginfo(
            "min points/cell = %d",
            self.min_points_per_cell
        )
        rospy.loginfo(
            "=========================================="
        )

    def world_to_grid(self, x, y):

        gx = int(
            (x - self.origin_x)
            / self.resolution
        )

        gy = int(
            (y - self.origin_y)
            / self.resolution
        )

        if (
            gx < 0
            or gx >= self.width
            or gy < 0
            or gy >= self.height
        ):
            return None

        return gx, gy

    @staticmethod
    def bresenham(x0, y0, x1, y1):

        cells = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        x = x0
        y = y0

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        if dx > dy:

            err = dx / 2.0

            while x != x1:

                cells.append((x, y))

                err -= dy

                if err < 0:
                    y += sy
                    err += dx

                x += sx

        else:

            err = dy / 2.0

            while y != y1:

                cells.append((x, y))

                err -= dx

                if err < 0:
                    x += sx
                    err += dy

                y += sy

        cells.append((x1, y1))

        return cells

    def get_robot_pose(self):

        try:

            trans, _ = self.tf_listener.lookupTransform(
                self.map_frame,
                "body",
                rospy.Time(0)
            )

            return (
                float(trans[0]),
                float(trans[1]),
                float(trans[2])
            )

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException
        ) as e:

            rospy.logwarn_throttle(
                2.0,
                "Cannot get robot TF: %s",
                str(e)
            )

            return None

    def clear_robot_footprint(
        self,
        robot_x,
        robot_y
    ):

        center = self.world_to_grid(
            robot_x,
            robot_y
        )

        if center is None:
            return

        cx, cy = center

        radius_cells = int(
            self.robot_clear_radius
            / self.resolution
        )

        for dy in range(
            -radius_cells,
            radius_cells + 1
        ):

            for dx in range(
                -radius_cells,
                radius_cells + 1
            ):

                if (
                    dx * dx + dy * dy
                    > radius_cells * radius_cells
                ):
                    continue

                gx = cx + dx
                gy = cy + dy

                if (
                    0 <= gx < self.width
                    and
                    0 <= gy < self.height
                ):

                    self.observed[gy, gx] = True

                    self.score[gy, gx] = max(
                        self.min_score,
                        int(self.score[gy, gx])
                        - self.robot_clear_strength
                    )

    def callback(self, msg):

        robot_pose = self.get_robot_pose()

        if robot_pose is None:
            return

        robot_x, robot_y, robot_z = robot_pose

        robot_cell = self.world_to_grid(
            robot_x,
            robot_y
        )

        if robot_cell is None:
            return

        rx, ry = robot_cell

        # --------------------------------------------------
        # 统计每个栅格这一帧收到了多少障碍点
        # --------------------------------------------------

        cell_point_count = {}

        for p in pc2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        ):

            x = float(p[0])
            y = float(p[1])
            z = float(p[2])

            relative_z = z - robot_z

            if not (
                self.min_relative_height
                <= relative_z
                <= self.max_relative_height
            ):
                continue

            dx = x - robot_x
            dy = y - robot_y

            distance = math.sqrt(
                dx * dx + dy * dy
            )

            if distance > self.max_range:
                continue

            cell = self.world_to_grid(
                x,
                y
            )

            if cell is None:
                continue

            cell_point_count[cell] = (
                cell_point_count.get(cell, 0)
                + 1
            )

        # 至少有多个点支持，才成为本帧有效障碍
        obstacle_cells = {
            cell
            for cell, count
            in cell_point_count.items()
            if count >= self.min_points_per_cell
        }

        # --------------------------------------------------
        # 先更新自由空间证据
        # --------------------------------------------------

        for ox, oy in obstacle_cells:

            ray = self.bresenham(
                rx,
                ry,
                ox,
                oy
            )

            for gx, gy in ray[:-1]:

                if (
                    0 <= gx < self.width
                    and
                    0 <= gy < self.height
                ):

                    self.observed[gy, gx] = True

                    # 关键：
                    # 即使以前是障碍，也允许自由观测降低其置信度。
                    self.score[gy, gx] = max(
                        self.min_score,
                        int(self.score[gy, gx])
                        - self.miss_decrement
                    )

        # --------------------------------------------------
        # 再增加障碍证据
        # --------------------------------------------------

        for ox, oy in obstacle_cells:

            self.observed[oy, ox] = True

            self.score[oy, ox] = min(
                self.max_score,
                int(self.score[oy, ox])
                + self.hit_increment
            )

        # 机器人实际走过的位置一定是free
        self.clear_robot_footprint(
            robot_x,
            robot_y
        )

        self.scan_counter += 1

        if (
            self.scan_counter
            >= self.publish_every_n_scans
        ):

            self.scan_counter = 0

            self.publish_map()

        occupied_number = int(
            np.sum(
                self.score
                >= self.occupied_threshold
            )
        )

        rospy.loginfo_throttle(
            1.0,
            (
                "2D mapping | "
                "current candidates=%d "
                "confirmed occupied=%d"
            )
            % (
                len(obstacle_cells),
                occupied_number
            )
        )

    def publish_map(self):

        # 默认unknown
        grid = np.full(
            (self.height, self.width),
            -1,
            dtype=np.int8
        )

        # 所有真正观测过的非障碍区域先设为free
        grid[self.observed] = 0

        # 只有达到足够证据的格子才设为occupied
        occupied_mask = (
            self.score
            >= self.occupied_threshold
        )

        grid[occupied_mask] = 100

        msg = OccupancyGrid()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = (
            self.origin_x
        )

        msg.info.origin.position.y = (
            self.origin_y
        )

        msg.info.origin.position.z = 0.0

        msg.info.origin.orientation.w = 1.0

        msg.data = grid.flatten().tolist()

        self.pub.publish(msg)


if __name__ == "__main__":

    rospy.init_node(
        "obstacle_cloud_to_grid"
    )

    ObstacleCloudToGrid()

    rospy.spin()
