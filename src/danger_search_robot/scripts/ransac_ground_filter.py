#!/usr/bin/env python3

import math
import random

import numpy as np
import rospy
import tf

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import quaternion_matrix


class RansacGroundFilter:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic",
            "/cloud_registered_body"
        )

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/cloud_obstacles"
        )

        # 在这个水平坐标系中执行RANSAC
        self.level_frame = rospy.get_param(
            "~level_frame",
            "map_level"
        )

        self.ransac_iterations = rospy.get_param(
            "~ransac_iterations",
            100
        )

        # 点距离地面平面小于该值，判定为地面
        self.distance_threshold = rospy.get_param(
            "~distance_threshold",
            0.08
        )

        # 注意：
        # 这里不再直接使用body坐标的z。
        # 点先变换到map_level，然后计算相对于机器人当前高度的相对高度。
        #
        # 地面通常位于机器人/雷达原点下方。
        self.ground_candidate_min_height = rospy.get_param(
            "~ground_candidate_min_height",
            -1.2
        )

        self.ground_candidate_max_height = rospy.get_param(
            "~ground_candidate_max_height",
            0.3
        )

        # 最大传感器处理距离
        self.max_range = rospy.get_param(
            "~max_range",
            12.0
        )

        # 在map_level中，真正的地面应该接近水平，
        # 因此法向量应该接近Z轴。
        self.min_normal_z = rospy.get_param(
            "~min_normal_z",
            0.90
        )

        self.tf_listener = tf.TransformListener()

        self.pub = rospy.Publisher(
            self.output_topic,
            PointCloud2,
            queue_size=1
        )

        rospy.Subscriber(
            self.input_topic,
            PointCloud2,
            self.callback,
            queue_size=1
        )

        rospy.loginfo("==========================================")
        rospy.loginfo("RANSAC ground filter started")
        rospy.loginfo("Input       : %s", self.input_topic)
        rospy.loginfo("Output      : %s", self.output_topic)
        rospy.loginfo("Level frame : %s", self.level_frame)
        rospy.loginfo("==========================================")

    @staticmethod
    def plane_from_points(p1, p2, p3):
        """
        三点确定平面：
        ax + by + cz + d = 0
        """

        ux = p2[0] - p1[0]
        uy = p2[1] - p1[1]
        uz = p2[2] - p1[2]

        vx = p3[0] - p1[0]
        vy = p3[1] - p1[1]
        vz = p3[2] - p1[2]

        a = uy * vz - uz * vy
        b = uz * vx - ux * vz
        c = ux * vy - uy * vx

        norm = math.sqrt(
            a * a +
            b * b +
            c * c
        )

        if norm < 1e-6:
            return None

        a /= norm
        b /= norm
        c /= norm

        d = -(
            a * p1[0] +
            b * p1[1] +
            c * p1[2]
        )

        # 统一让法向量朝map_level的+Z
        if c < 0:
            a = -a
            b = -b
            c = -c
            d = -d

        return a, b, c, d

    @staticmethod
    def point_plane_distance(point, plane):
        a, b, c, d = plane

        return abs(
            a * point[0] +
            b * point[1] +
            c * point[2] +
            d
        )

    def find_ground_plane(self, candidates):
        if len(candidates) < 50:
            return None, 0

        best_plane = None
        best_inlier_count = 0

        for _ in range(self.ransac_iterations):

            try:
                p1, p2, p3 = random.sample(
                    candidates,
                    3
                )
            except ValueError:
                break

            plane = self.plane_from_points(
                p1,
                p2,
                p3
            )

            if plane is None:
                continue

            a, b, c, d = plane

            # 点已经变换到了水平的map_level坐标系，
            # 所以地面法向量应该接近Z轴。
            if c < self.min_normal_z:
                continue

            count = 0

            for point in candidates:
                if self.point_plane_distance(
                    point,
                    plane
                ) < self.distance_threshold:
                    count += 1

            if count > best_inlier_count:
                best_inlier_count = count
                best_plane = plane

        return best_plane, best_inlier_count

    def get_transform(self, source_frame, stamp):
        """
        获取 source_frame -> map_level 的变换。
        优先使用点云时间戳；
        如果发生外推异常，则退回最新TF。
        """

        try:
            trans, rot = self.tf_listener.lookupTransform(
                self.level_frame,
                source_frame,
                stamp
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException
        ):
            try:
                trans, rot = self.tf_listener.lookupTransform(
                    self.level_frame,
                    source_frame,
                    rospy.Time(0)
                )
            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException
            ) as e:
                rospy.logwarn_throttle(
                    2.0,
                    "Cannot transform %s -> %s: %s",
                    source_frame,
                    self.level_frame,
                    str(e)
                )
                return None, None

        return trans, rot

    def callback(self, msg):

        source_frame = msg.header.frame_id

        if not source_frame:
            source_frame = "body"

        trans, rot = self.get_transform(
            source_frame,
            msg.header.stamp
        )

        if trans is None:
            return

        # quaternion -> rotation matrix
        matrix = quaternion_matrix(rot)

        rotation = matrix[0:3, 0:3]

        translation = np.array(
            [
                trans[0],
                trans[1],
                trans[2]
            ],
            dtype=np.float64
        )

        # body原点在map_level中的当前高度
        robot_level_z = trans[2]

        all_points_level = []
        ground_candidates = []

        for p in pc2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        ):

            x = float(p[0])
            y = float(p[1])
            z = float(p[2])

            # 用真实三维距离限制处理范围，
            # 不再假定body XY是水平面。
            sensor_range = math.sqrt(
                x * x +
                y * y +
                z * z
            )

            if sensor_range > self.max_range:
                continue

            p_source = np.array(
                [x, y, z],
                dtype=np.float64
            )

            # body/current frame -> map_level
            p_level = (
                rotation.dot(p_source)
                + translation
            )

            point = (
                float(p_level[0]),
                float(p_level[1]),
                float(p_level[2])
            )

            all_points_level.append(point)

            # 相对于机器人当前高度判断，
            # 因此上二楼、三楼后阈值仍然有效。
            relative_height = (
                p_level[2]
                - robot_level_z
            )

            if (
                self.ground_candidate_min_height
                <= relative_height
                <= self.ground_candidate_max_height
            ):
                ground_candidates.append(point)

        if len(all_points_level) == 0:
            return

        plane, inlier_count = self.find_ground_plane(
            ground_candidates
        )

        if plane is None:
            rospy.logwarn_throttle(
                2.0,
                (
                    "No ground plane | "
                    "all=%d candidates=%d"
                )
                % (
                    len(all_points_level),
                    len(ground_candidates)
                )
            )

            # 没找到地面时不丢数据：
            # 所有点暂时作为障碍输出。
            obstacle_points = all_points_level

        else:
            obstacle_points = []

            for point in all_points_level:

                distance_to_ground = \
                    self.point_plane_distance(
                        point,
                        plane
                    )

                if (
                    distance_to_ground
                    > self.distance_threshold
                ):
                    obstacle_points.append(point)

            removed_count = (
                len(all_points_level)
                - len(obstacle_points)
            )

            a, b, c, d = plane

            rospy.loginfo_throttle(
                1.0,
                (
                    "[GROUND] "
                    "all=%d "
                    "candidates=%d "
                    "ransac_inliers=%d "
                    "removed=%d "
                    "obstacles=%d "
                    "normal=(%.3f, %.3f, %.3f)"
                )
                % (
                    len(all_points_level),
                    len(ground_candidates),
                    inlier_count,
                    removed_count,
                    len(obstacle_points),
                    a,
                    b,
                    c
                )
            )

        fields = [
            PointField(
                "x",
                0,
                PointField.FLOAT32,
                1
            ),
            PointField(
                "y",
                4,
                PointField.FLOAT32,
                1
            ),
            PointField(
                "z",
                8,
                PointField.FLOAT32,
                1
            )
        ]

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.level_frame

        output = pc2.create_cloud(
            header,
            fields,
            obstacle_points
        )

        self.pub.publish(output)


if __name__ == "__main__":
    rospy.init_node(
        "ransac_ground_filter"
    )

    RansacGroundFilter()

    rospy.spin()
