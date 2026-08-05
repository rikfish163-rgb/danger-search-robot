#!/usr/bin/env python3

import math
import sys

import numpy as np
import rospy
import tf
from gazebo_msgs.msg import ModelStates


def matrix_from_tf(translation, quaternion):
    matrix = tf.transformations.quaternion_matrix(quaternion)
    matrix[0, 3] = translation[0]
    matrix[1, 3] = translation[1]
    matrix[2, 3] = translation[2]
    return matrix


def matrix_from_pose(position, orientation):
    matrix = tf.transformations.quaternion_matrix([
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    ])
    matrix[0, 3] = position.x
    matrix[1, 3] = position.y
    matrix[2, 3] = position.z
    return matrix


def rotation_error_degree(matrix_a, matrix_b):
    relative = np.linalg.inv(matrix_a[:3, :3]) @ matrix_b[:3, :3]

    cosine = (np.trace(relative) - 1.0) / 2.0
    cosine = np.clip(cosine, -1.0, 1.0)

    return math.degrees(math.acos(cosine))


def choose_model(names):
    if "a1_gazebo" in names:
        return "a1_gazebo"

    candidates = [
        name for name in names
        if "a1" in name.lower()
    ]

    if len(candidates) == 1:
        return candidates[0]

    print("无法唯一确定 A1 模型，当前模型：")
    for name in names:
        print(" ", name)

    return None


rospy.init_node("validate_world_map_level", anonymous=True)

listener = tf.TransformListener()
rospy.sleep(2.0)

print("等待 Gazebo 和 TF 数据……")

try:
    states = rospy.wait_for_message(
        "/gazebo/model_states",
        ModelStates,
        timeout=10.0
    )
except Exception as error:
    print("读取 /gazebo/model_states 失败：", error)
    sys.exit(1)

model_name = choose_model(states.name)

if model_name is None:
    sys.exit(1)

required_transforms = [
    ("world", "map_level"),
    ("map_level", "body"),
    ("base", "livox_imu_link"),
]

for parent, child in required_transforms:
    try:
        listener.waitForTransform(
            parent,
            child,
            rospy.Time(0),
            rospy.Duration(15.0)
        )
    except Exception as error:
        print("等待 TF 失败：%s -> %s" % (parent, child))
        print(error)
        sys.exit(1)

print("机器人模型：", model_name)
print("开始验证。让机器人运动约 2～5 米。")
print("按 Ctrl+C 结束并输出统计结果。\n")

position_errors = []
xy_errors = []
rotation_errors = []

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    try:
        states = rospy.wait_for_message(
            "/gazebo/model_states",
            ModelStates,
            timeout=2.0
        )

        model_index = states.name.index(model_name)

        truth_world_base = matrix_from_pose(
            states.pose[model_index].position,
            states.pose[model_index].orientation
        )

        world_map_t, world_map_q = listener.lookupTransform(
            "world",
            "map_level",
            rospy.Time(0)
        )

        map_body_t, map_body_q = listener.lookupTransform(
            "map_level",
            "body",
            rospy.Time(0)
        )

        base_body_t, base_body_q = listener.lookupTransform(
            "base",
            "livox_imu_link",
            rospy.Time(0)
        )

    except Exception as error:
        rospy.logwarn_throttle(
            2.0,
            "读取验证数据失败：%s" % error
        )
        rate.sleep()
        continue

    world_map = matrix_from_tf(world_map_t, world_map_q)
    map_body = matrix_from_tf(map_body_t, map_body_q)
    base_body = matrix_from_tf(base_body_t, base_body_q)

    # SLAM 推算：
    # T_world_base =
    # T_world_map * T_map_body * inverse(T_base_body)
    estimated_world_base = (
        world_map
        @ map_body
        @ np.linalg.inv(base_body)
    )

    difference = (
        estimated_world_base[:3, 3]
        - truth_world_base[:3, 3]
    )

    xy_error = np.linalg.norm(difference[:2])
    position_error = np.linalg.norm(difference)

    angle_error = rotation_error_degree(
        truth_world_base,
        estimated_world_base
    )

    xy_errors.append(xy_error)
    position_errors.append(position_error)
    rotation_errors.append(angle_error)

    if len(xy_errors) % 10 == 0:
        truth = truth_world_base[:3, 3]
        estimated = estimated_world_base[:3, 3]

        print(
            "samples=%4d | "
            "truth=[% .3f % .3f % .3f] | "
            "slam=[% .3f % .3f % .3f] | "
            "XY=%.3f m | 3D=%.3f m | angle=%.2f deg"
            % (
                len(xy_errors),
                truth[0], truth[1], truth[2],
                estimated[0], estimated[1], estimated[2],
                xy_error,
                position_error,
                angle_error,
            )
        )

    try:
        rate.sleep()
    except rospy.ROSInterruptException:
        break


if not xy_errors:
    print("没有获得有效样本。")
    sys.exit(1)

xy_errors = np.asarray(xy_errors)
position_errors = np.asarray(position_errors)
rotation_errors = np.asarray(rotation_errors)

print("\n========== VALIDATION RESULT ==========")
print("Valid samples:", len(xy_errors))

print("\nXY error [m]:")
print("mean =", np.mean(xy_errors))
print("RMSE =", np.sqrt(np.mean(xy_errors ** 2)))
print("max  =", np.max(xy_errors))

print("\n3D error [m]:")
print("mean =", np.mean(position_errors))
print("RMSE =", np.sqrt(np.mean(position_errors ** 2)))
print("max  =", np.max(position_errors))

print("\nRotation error [degree]:")
print("mean =", np.mean(rotation_errors))
print("RMSE =", np.sqrt(np.mean(rotation_errors ** 2)))
print("max  =", np.max(rotation_errors))
