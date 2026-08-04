#!/usr/bin/env python3

import math
import sys

import numpy as np
import rospy
import tf
from gazebo_msgs.msg import ModelStates


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


def matrix_from_tf(translation, quaternion):
    matrix = tf.transformations.quaternion_matrix(quaternion)
    matrix[0, 3] = translation[0]
    matrix[1, 3] = translation[1]
    matrix[2, 3] = translation[2]
    return matrix


def choose_robot_model(names):
    if "a1_gazebo" in names:
        return "a1_gazebo"

    candidates = [
        name for name in names
        if "a1" in name.lower()
    ]

    if len(candidates) == 1:
        return candidates[0]

    print("Could not uniquely identify the A1 model.")
    print("Available models:")

    for name in names:
        print("  ", name)

    return None


def quaternion_average(quaternions):
    reference = quaternions[0]
    aligned = []

    for quaternion in quaternions:
        quaternion = np.asarray(quaternion, dtype=float)

        if np.dot(reference, quaternion) < 0:
            quaternion = -quaternion

        aligned.append(quaternion)

    mean = np.mean(np.asarray(aligned), axis=0)
    norm = np.linalg.norm(mean)

    if norm < 1e-12:
        raise RuntimeError("Quaternion average is invalid.")

    return mean / norm


rospy.init_node("calibrate_world_map_level", anonymous=True)

listener = tf.TransformListener()
rospy.sleep(2.0)

print("Waiting for /gazebo/model_states...")

try:
    first_states = rospy.wait_for_message(
        "/gazebo/model_states",
        ModelStates,
        timeout=10.0
    )
except Exception as error:
    print("Failed to read /gazebo/model_states:", error)
    sys.exit(1)

model_name = choose_robot_model(first_states.name)

if model_name is None:
    sys.exit(1)

model_index = first_states.name.index(model_name)

print("Robot model:", model_name)
print("Waiting for TF transforms...")

try:
    listener.waitForTransform(
        "map_level",
        "body",
        rospy.Time(0),
        rospy.Duration(15.0)
    )

    listener.waitForTransform(
        "base",
        "livox_imu_link",
        rospy.Time(0),
        rospy.Duration(15.0)
    )
except Exception as error:
    print("TF wait failed:", error)
    sys.exit(1)

translations = []
quaternions = []

sample_count = 100
rate = rospy.Rate(10)

print("Collecting %d stationary samples..." % sample_count)

for sample_index in range(sample_count):
    if rospy.is_shutdown():
        break

    try:
        states = rospy.wait_for_message(
            "/gazebo/model_states",
            ModelStates,
            timeout=2.0
        )

        model_index = states.name.index(model_name)
        world_base_pose = states.pose[model_index]

        map_body_translation, map_body_quaternion = (
            listener.lookupTransform(
                "map_level",
                "body",
                rospy.Time(0)
            )
        )

        base_imu_translation, base_imu_quaternion = (
            listener.lookupTransform(
                "base",
                "livox_imu_link",
                rospy.Time(0)
            )
        )

    except Exception as error:
        print(
            "Sample %d failed: %s"
            % (sample_index, error)
        )
        rate.sleep()
        continue

    # T_world_base：Gazebo 模型真值
    world_base = matrix_from_pose(
        world_base_pose.position,
        world_base_pose.orientation
    )

    # T_base_body：body 在 FAST-LIO 中表示 IMU 机体系，
    # 使用机器人固定的 base -> livox_imu_link 外参
    base_body = matrix_from_tf(
        base_imu_translation,
        base_imu_quaternion
    )

    # T_map_body：FAST-LIO / map_level 估计的机体位姿
    map_body = matrix_from_tf(
        map_body_translation,
        map_body_quaternion
    )

    # T_world_map =
    # T_world_base * T_base_body * inverse(T_map_body)
    world_map = (
        world_base
        @ base_body
        @ np.linalg.inv(map_body)
    )

    translation = world_map[:3, 3]
    quaternion = tf.transformations.quaternion_from_matrix(
        world_map
    )

    translations.append(translation)
    quaternions.append(quaternion)

    rate.sleep()

if len(translations) < 20:
    print("Not enough valid samples:", len(translations))
    sys.exit(1)

translations = np.asarray(translations)
quaternions = np.asarray(quaternions)

translation_mean = np.mean(translations, axis=0)
translation_std = np.std(translations, axis=0)
quaternion_mean = quaternion_average(quaternions)

roll, pitch, yaw = tf.transformations.euler_from_quaternion(
    quaternion_mean
)

print()
print("========== CALIBRATION RESULT ==========")
print("Valid samples:", len(translations))

print()
print("world -> map_level translation mean [m]:")
print(translation_mean)

print("translation std [m]:")
print(translation_std)

print()
print("world -> map_level quaternion [x y z w]:")
print(quaternion_mean)

print()
print("world -> map_level RPY [degree]:")
print([
    math.degrees(roll),
    math.degrees(pitch),
    math.degrees(yaw),
])

print()
print("Static transform publisher arguments:")
print(
    "%.9f %.9f %.9f %.9f %.9f %.9f %.9f world map_level"
    % (
        translation_mean[0],
        translation_mean[1],
        translation_mean[2],
        quaternion_mean[0],
        quaternion_mean[1],
        quaternion_mean[2],
        quaternion_mean[3],
    )
)

print()
print("Suggested command:")
print(
    "rosrun tf static_transform_publisher "
    "%.9f %.9f %.9f %.9f %.9f %.9f %.9f "
    "world map_level 100"
    % (
        translation_mean[0],
        translation_mean[1],
        translation_mean[2],
        quaternion_mean[0],
        quaternion_mean[1],
        quaternion_mean[2],
        quaternion_mean[3],
    )
)
