#!/usr/bin/env python3
"""Capture the robot world-frame pose before an elevator transition.

This script reads the existing TF world -> body, averages stationary samples,
adds the configured fixed floor-height offset, and writes an anchor JSON file.
It is intentionally a one-shot tool and does not publish TF.
"""

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Iterable, List

import numpy as np
import rospy
import tf.transformations
import tf2_ros


def average_quaternion(quaternions: Iterable[Iterable[float]]) -> np.ndarray:
    values = [np.asarray(q, dtype=float) for q in quaternions]
    if not values:
        raise RuntimeError("No quaternion samples were collected.")

    reference = values[0]
    aligned: List[np.ndarray] = []
    for quaternion in values:
        if np.dot(reference, quaternion) < 0.0:
            quaternion = -quaternion
        aligned.append(quaternion)

    result = np.mean(np.asarray(aligned), axis=0)
    norm = np.linalg.norm(result)
    if norm < 1e-12:
        raise RuntimeError("Quaternion average is invalid.")
    return result / norm


def transform_to_matrix(transform) -> np.ndarray:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    matrix = tf.transformations.quaternion_matrix(
        [rotation.x, rotation.y, rotation.z, rotation.w]
    )
    matrix[0, 3] = translation.x
    matrix[1, 3] = translation.y
    matrix[2, 3] = translation.z
    return matrix


def matrix_to_dict(matrix: np.ndarray) -> dict:
    quaternion = tf.transformations.quaternion_from_matrix(matrix)
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
    return {
        "translation": {
            "x": float(matrix[0, 3]),
            "y": float(matrix[1, 3]),
            "z": float(matrix[2, 3]),
        },
        "quaternion": {
            "x": float(quaternion[0]),
            "y": float(quaternion[1]),
            "z": float(quaternion[2]),
            "w": float(quaternion[3]),
        },
        "rpy": {
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw": float(yaw),
        },
    }


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Save world->body before an elevator transition."
    )
    parser.add_argument("--source-floor", type=int, required=True)
    parser.add_argument("--target-floor", type=int, required=True)
    parser.add_argument("--floor-height", type=float, default=2.6)
    parser.add_argument(
        "--output",
        default=os.path.expanduser(
            "~/catkin_ws/results/floor_transition_anchor.json"
        ),
    )
    parser.add_argument("--world-frame", default="world")
    parser.add_argument("--body-frame", default="body")
    parser.add_argument("--sample-count", type=int, default=100)
    parser.add_argument("--sample-rate", type=float, default=10.0)
    parser.add_argument("--max-translation-std", type=float, default=0.02)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main() -> None:
    args = parse_arguments()

    if args.source_floor == args.target_floor:
        raise ValueError("source-floor and target-floor must be different.")
    if args.sample_count < 1:
        raise ValueError("sample-count must be positive.")
    if args.floor_height <= 0.0:
        raise ValueError("floor-height must be positive.")

    rospy.init_node("capture_floor_transition_anchor", anonymous=True)

    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
    listener = tf2_ros.TransformListener(buffer)
    del listener

    rospy.loginfo(
        "Waiting for TF %s -> %s...",
        args.world_frame,
        args.body_frame,
    )

    try:
        buffer.can_transform(
            args.world_frame,
            args.body_frame,
            rospy.Time(0),
            rospy.Duration(20.0),
        )
    except Exception as error:
        rospy.logerr("TF wait failed: %s", error)
        raise

    translations = []
    quaternions = []
    rate = rospy.Rate(args.sample_rate)

    rospy.loginfo(
        "Collecting %d stationary samples. Keep the robot still.",
        args.sample_count,
    )

    while not rospy.is_shutdown() and len(translations) < args.sample_count:
        try:
            stamped = buffer.lookup_transform(
                args.world_frame,
                args.body_frame,
                rospy.Time(0),
                rospy.Duration(2.0),
            )
            matrix = transform_to_matrix(stamped)
            translations.append(matrix[:3, 3].copy())
            quaternions.append(
                tf.transformations.quaternion_from_matrix(matrix)
            )
        except Exception as error:
            rospy.logwarn_throttle(2.0, "TF sample failed: %s", error)
        rate.sleep()

    if len(translations) < max(10, args.sample_count // 2):
        raise RuntimeError(
            "Not enough valid TF samples: %d" % len(translations)
        )

    translations_array = np.asarray(translations)
    translation_mean = np.mean(translations_array, axis=0)
    translation_std = np.std(translations_array, axis=0)
    quaternion_mean = average_quaternion(quaternions)

    if np.max(translation_std) > args.max_translation_std:
        raise RuntimeError(
            "Robot was not stationary enough. Translation std=%s m, limit=%.6f m"
            % (translation_std.tolist(), args.max_translation_std)
        )

    world_body_before = tf.transformations.quaternion_matrix(quaternion_mean)
    world_body_before[:3, 3] = translation_mean

    delta_z = (
        args.target_floor - args.source_floor
    ) * args.floor_height

    world_body_after = world_body_before.copy()
    world_body_after[2, 3] += delta_z

    payload = {
        "schema": "floor_transition_anchor_v1",
        "created_at_ros_time": float(rospy.Time.now().to_sec()),
        "source_floor": int(args.source_floor),
        "target_floor": int(args.target_floor),
        "floor_height": float(args.floor_height),
        "delta_z": float(delta_z),
        "frames": {
            "world": args.world_frame,
            "body": args.body_frame,
            "level": "map_level",
        },
        "sample_count": int(len(translations)),
        "translation_std": {
            "x": float(translation_std[0]),
            "y": float(translation_std[1]),
            "z": float(translation_std[2]),
        },
        "world_body_before": matrix_to_dict(world_body_before),
        "world_body_after_expected": matrix_to_dict(world_body_after),
    }

    output_path = Path(os.path.expanduser(args.output))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_suffix(output_path.suffix + ".tmp")
    temporary_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary_path.replace(output_path)

    before = payload["world_body_before"]["translation"]
    after = payload["world_body_after_expected"]["translation"]

    rospy.loginfo("Anchor saved to %s", output_path)
    rospy.loginfo(
        "Before elevator world body: x=%.6f y=%.6f z=%.6f",
        before["x"], before["y"], before["z"],
    )
    rospy.loginfo(
        "Expected after elevator:    x=%.6f y=%.6f z=%.6f",
        after["x"], after["y"], after["z"],
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        rospy.logerr("Failed to capture floor-transition anchor: %s", error)
        sys.exit(1)
