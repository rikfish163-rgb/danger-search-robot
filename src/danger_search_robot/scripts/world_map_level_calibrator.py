#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
使用五次实验的平均站稳 world->base 位姿，完成初始 world->map_level 标定。

保持原有逻辑：
    T_world_map_level
      = T_world_base_fixed
      * T_base_body
      * inverse(T_map_level_body)

与旧版唯一的本质差别：
    不再订阅 Gazebo 模型真值话题；
    T_world_base 改为五次实验平均常量。

原有文件名、节点名、launch 命令和 calibrated 参数保持不变。
"""

import math
import sys
from typing import Iterable, List, Sequence, Tuple

import numpy as np
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped


# ============================================================
# 五次完整重启实验后，a1_gazebo::base 站稳位姿的平均值
# roll/pitch 在原实验输出中未记录，因此按水平世界坐标设为 0。
# ============================================================
DEFAULT_WORLD_BASE_X = 0.0066335955980417504
DEFAULT_WORLD_BASE_Y = -3.619166509247556
DEFAULT_WORLD_BASE_Z = 0.25858445372763265
DEFAULT_WORLD_BASE_ROLL = 0.0
DEFAULT_WORLD_BASE_PITCH = 0.0
DEFAULT_WORLD_BASE_YAW = 1.6010323201553712


def matrix_from_tf(
    translation: Sequence[float],
    quaternion: Sequence[float],
) -> np.ndarray:
    matrix = tf.transformations.quaternion_matrix(quaternion)
    matrix[0:3, 3] = np.asarray(translation, dtype=np.float64)
    return matrix


def matrix_from_xyz_rpy(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> np.ndarray:
    quaternion = tf.transformations.quaternion_from_euler(
        roll, pitch, yaw
    )
    return matrix_from_tf((x, y, z), quaternion)


def average_quaternion(
    quaternions: Iterable[Sequence[float]],
) -> np.ndarray:
    quaternion_list: List[np.ndarray] = [
        np.asarray(q, dtype=np.float64) for q in quaternions
    ]
    if not quaternion_list:
        raise ValueError("没有可平均的四元数")

    reference = quaternion_list[0]
    aligned = []

    for quaternion in quaternion_list:
        norm = np.linalg.norm(quaternion)
        if norm <= 1e-12:
            raise ValueError("检测到无效的零四元数")

        quaternion = quaternion / norm

        # q 和 -q 表示相同旋转；先统一符号再平均。
        if np.dot(quaternion, reference) < 0.0:
            quaternion = -quaternion

        aligned.append(quaternion)

    mean = np.mean(np.vstack(aligned), axis=0)
    norm = np.linalg.norm(mean)

    if norm <= 1e-12:
        raise ValueError("四元数平均结果无效")

    return mean / norm


class WorldMapLevelCalibrator:
    def __init__(self) -> None:
        rospy.init_node("world_map_level_calibrator")

        # 保留旧 launch 中的参数名，robot_model 现在只用于兼容和日志。
        self.robot_model = rospy.get_param("~robot_model", "a1_gazebo")
        self.sample_count = int(rospy.get_param("~sample_count", 100))
        self.sample_rate = float(rospy.get_param("~sample_rate", 10.0))
        self.max_translation_std = float(
            rospy.get_param("~max_translation_std", 0.02)
        )
        self.parent_frame = rospy.get_param("~parent_frame", "world")
        self.child_frame = rospy.get_param(
            "~child_frame", "map_level"
        )

        self.map_level_frame = rospy.get_param(
            "~map_level_frame", self.child_frame
        )
        self.camera_init_frame = rospy.get_param(
            "~camera_init_frame", "camera_init"
        )
        self.body_frame = rospy.get_param("~body_frame", "body")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.base_body_frame = rospy.get_param(
            "~base_body_frame", "livox_imu_link"
        )
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 15.0))

        # 平均站稳位姿允许通过 ROS 参数覆盖，但默认就是五次平均值。
        self.world_base_x = float(
            rospy.get_param(
                "~fixed_world_base_x", DEFAULT_WORLD_BASE_X
            )
        )
        self.world_base_y = float(
            rospy.get_param(
                "~fixed_world_base_y", DEFAULT_WORLD_BASE_Y
            )
        )
        self.world_base_z = float(
            rospy.get_param(
                "~fixed_world_base_z", DEFAULT_WORLD_BASE_Z
            )
        )
        self.world_base_roll = float(
            rospy.get_param(
                "~fixed_world_base_roll", DEFAULT_WORLD_BASE_ROLL
            )
        )
        self.world_base_pitch = float(
            rospy.get_param(
                "~fixed_world_base_pitch", DEFAULT_WORLD_BASE_PITCH
            )
        )
        self.world_base_yaw = float(
            rospy.get_param(
                "~fixed_world_base_yaw", DEFAULT_WORLD_BASE_YAW
            )
        )

        if self.sample_count <= 0:
            raise ValueError("sample_count 必须大于 0")
        if self.sample_rate <= 0.0:
            raise ValueError("sample_rate 必须大于 0")
        if self.max_translation_std <= 0.0:
            raise ValueError("max_translation_std 必须大于 0")

        self.listener = tf.TransformListener()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        rospy.set_param("~calibrated", False)
        rospy.set_param("~truth_source", "fixed_five_run_average")
        rospy.set_param(
            "~uses_gazebo_model_states",
            False,
        )

    def wait_for_required_transforms(self) -> None:
        # 不直接等待整条 map_level -> body 链。
        # 某些启动顺序下，水平固定变换和 FAST-LIO 动态变换的
        # TF 时间区间短暂不重叠，整链查询会触发 extrapolation。
        required_edges = (
            (self.map_level_frame, self.camera_init_frame),
            (self.camera_init_frame, self.body_frame),
            (self.base_frame, self.base_body_frame),
        )

        for target_frame, source_frame in required_edges:
            rospy.loginfo(
                "Waiting for TF %s -> %s",
                target_frame,
                source_frame,
            )
            self.listener.waitForTransform(
                target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout),
            )

    def lookup_matrix(
        self,
        target_frame: str,
        source_frame: str,
    ) -> np.ndarray:
        translation, quaternion = self.listener.lookupTransform(
            target_frame,
            source_frame,
            rospy.Time(0),
        )
        return matrix_from_tf(translation, quaternion)

    def lookup_map_body_matrix(self) -> np.ndarray:
        # 分别读取两段各自的最新 TF，再在本节点中组合：
        #
        # T_map_level_body
        #   = T_map_level_camera_init * T_camera_init_body
        #
        # 这样不要求静态水平变换与 FAST-LIO 动态变换的时间戳
        # 落在同一 TF 缓冲时间区间内。
        map_camera_init = self.lookup_matrix(
            self.map_level_frame,
            self.camera_init_frame,
        )
        camera_init_body = self.lookup_matrix(
            self.camera_init_frame,
            self.body_frame,
        )
        return map_camera_init @ camera_init_body

    def collect_samples(
        self,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        world_base = matrix_from_xyz_rpy(
            self.world_base_x,
            self.world_base_y,
            self.world_base_z,
            self.world_base_roll,
            self.world_base_pitch,
            self.world_base_yaw,
        )

        translations: List[np.ndarray] = []
        quaternions: List[np.ndarray] = []

        rate = rospy.Rate(self.sample_rate)

        rospy.loginfo(
            "Collecting %d stationary samples. Keep the robot still.",
            self.sample_count,
        )
        rospy.loginfo(
            "Fixed world->base average: "
            "xyz=(%.9f, %.9f, %.9f), "
            "rpy=(%.9f, %.9f, %.9f)",
            self.world_base_x,
            self.world_base_y,
            self.world_base_z,
            self.world_base_roll,
            self.world_base_pitch,
            self.world_base_yaw,
        )

        while len(translations) < self.sample_count:
            if rospy.is_shutdown():
                raise rospy.ROSInterruptException()

            try:
                # 与原程序相同：
                # T_map_level_body
                map_body = self.lookup_map_body_matrix()

                # 与原程序相同：
                # T_base_body（当前工程中由 base -> livox_imu_link 表示）
                base_body = self.lookup_matrix(
                    self.base_frame,
                    self.base_body_frame,
                )

                # 保持原公式，只将 world_base 从 Gazebo 真值改为固定平均值。
                world_map_level = (
                    world_base
                    @ base_body
                    @ np.linalg.inv(map_body)
                )

                translation = world_map_level[0:3, 3].copy()
                quaternion = (
                    tf.transformations.quaternion_from_matrix(
                        world_map_level
                    )
                )

                translations.append(translation)
                quaternions.append(
                    np.asarray(quaternion, dtype=np.float64)
                )

            except (
                tf.Exception,
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
                np.linalg.LinAlgError,
            ) as error:
                rospy.logwarn_throttle(
                    2.0,
                    "Calibration sample skipped: %s",
                    str(error),
                )

            rate.sleep()

        translation_array = np.vstack(translations)
        translation_mean = np.mean(translation_array, axis=0)
        translation_std = np.std(translation_array, axis=0)
        quaternion_mean = average_quaternion(quaternions)

        return translation_mean, translation_std, quaternion_mean

    def validate_stationary_samples(
        self,
        translation_std: np.ndarray,
    ) -> None:
        rospy.loginfo(
            "world->map_level translation std: "
            "[%.6f, %.6f, %.6f] m",
            translation_std[0],
            translation_std[1],
            translation_std[2],
        )

        if np.any(translation_std > self.max_translation_std):
            raise RuntimeError(
                "机器人未保持静止，world->map_level 平移标准差 "
                "{} 超过阈值 {:.6f} m".format(
                    translation_std.tolist(),
                    self.max_translation_std,
                )
            )

    def make_transform_message(
        self,
        translation: np.ndarray,
        quaternion: np.ndarray,
    ) -> TransformStamped:
        message = TransformStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.parent_frame
        message.child_frame_id = self.child_frame

        message.transform.translation.x = float(translation[0])
        message.transform.translation.y = float(translation[1])
        message.transform.translation.z = float(translation[2])

        message.transform.rotation.x = float(quaternion[0])
        message.transform.rotation.y = float(quaternion[1])
        message.transform.rotation.z = float(quaternion[2])
        message.transform.rotation.w = float(quaternion[3])

        return message

    def run(self) -> None:
        self.wait_for_required_transforms()

        (
            translation_mean,
            translation_std,
            quaternion_mean,
        ) = self.collect_samples()

        self.validate_stationary_samples(translation_std)

        transform_message = self.make_transform_message(
            translation_mean,
            quaternion_mean,
        )

        self.static_broadcaster.sendTransform(transform_message)

        rospy.set_param("~calibrated", True)
        rospy.set_param(
            "~fixed_world_base_pose",
            {
                "x": self.world_base_x,
                "y": self.world_base_y,
                "z": self.world_base_z,
                "roll": self.world_base_roll,
                "pitch": self.world_base_pitch,
                "yaw": self.world_base_yaw,
            },
        )
        rospy.set_param(
            "~world_map_level_transform",
            {
                "x": float(translation_mean[0]),
                "y": float(translation_mean[1]),
                "z": float(translation_mean[2]),
                "qx": float(quaternion_mean[0]),
                "qy": float(quaternion_mean[1]),
                "qz": float(quaternion_mean[2]),
                "qw": float(quaternion_mean[3]),
            },
        )

        rospy.loginfo(
            "Published static TF %s -> %s",
            self.parent_frame,
            self.child_frame,
        )
        rospy.loginfo(
            "translation = [%.9f, %.9f, %.9f]",
            translation_mean[0],
            translation_mean[1],
            translation_mean[2],
        )
        rospy.loginfo(
            "quaternion = [%.9f, %.9f, %.9f, %.9f]",
            quaternion_mean[0],
            quaternion_mean[1],
            quaternion_mean[2],
            quaternion_mean[3],
        )
        rospy.loginfo(
            "Calibration source: fixed five-run average; "
            "Gazebo model-state truth is not used."
        )

        # 保持节点运行，使 tf_static 对后续新启动节点仍然可用。
        rospy.spin()


def main() -> int:
    try:
        node = WorldMapLevelCalibrator()
        node.run()
        return 0
    except rospy.ROSInterruptException:
        return 0
    except Exception as error:
        rospy.logfatal(
            "world_map_level_calibrator failed: %s",
            str(error),
        )
        return 1


if __name__ == "__main__":
    sys.exit(main())
