#!/usr/bin/env python3
"""Keep the simulator's world frame aligned with the live FAST-LIO pose.

The real robot uses the calibrated/re-anchored world -> map_level transform.
For the Gazebo validation profile, model state is an explicit simulator-only
truth source.  This node uses it only to align the moving FAST-LIO map; all
sensor data, maps, navigation, and detections remain live ROS topics.
"""

import threading

import numpy as np
import rospy
import tf
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def pose_matrix(position, quaternion):
    matrix = tf.transformations.quaternion_matrix(quaternion)
    matrix[:3, 3] = [position[0], position[1], position[2]]
    return matrix


class WorldTruthAnchor:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "a1_gazebo")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.level_frame = rospy.get_param("~level_frame", "map_level")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_init")
        self.body_frame = rospy.get_param("~body_frame", "body")
        self.rate_hz = float(rospy.get_param("~rate", 10.0))

        self.lock = threading.Lock()
        self.latest_odom = None
        self.latest_truth = None
        self.tf_listener = tf.TransformListener()
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.level_camera = None

        rospy.Subscriber("/Odometry", Odometry, self._odom_callback, queue_size=10)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_callback, queue_size=1)

        rospy.loginfo(
            "Waiting for TF %s -> %s before simulator world anchoring",
            self.level_frame,
            self.camera_frame,
        )

    def _odom_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        matrix = pose_matrix(
            (position.x, position.y, position.z),
            (orientation.x, orientation.y, orientation.z, orientation.w),
        )
        with self.lock:
            self.latest_odom = matrix

    def _model_states_callback(self, message):
        try:
            index = message.name.index(self.model_name)
        except ValueError:
            return
        pose = message.pose[index]
        matrix = pose_matrix(
            (pose.position.x, pose.position.y, pose.position.z),
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )
        with self.lock:
            self.latest_truth = matrix

    def _publish_once(self):
        if self.level_camera is None:
            try:
                translation, quaternion = self.tf_listener.lookupTransform(
                    self.level_frame,
                    self.camera_frame,
                    rospy.Time(0),
                )
                self.level_camera = pose_matrix(translation, quaternion)
                rospy.loginfo("Simulator world anchoring TF is ready")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn_throttle(
                    5.0,
                    "Waiting for TF %s -> %s",
                    self.level_frame,
                    self.camera_frame,
                )
                return
        with self.lock:
            camera_body = None if self.latest_odom is None else self.latest_odom.copy()
            world_body = None if self.latest_truth is None else self.latest_truth.copy()
        if camera_body is None or world_body is None:
            rospy.logwarn_throttle(
                5.0,
                "Waiting for live poses odom=%s truth=%s",
                camera_body is not None,
                world_body is not None,
            )
            return
        world_level = world_body @ np.linalg.inv(camera_body) @ np.linalg.inv(self.level_camera)
        quaternion = tf.transformations.quaternion_from_matrix(world_level)

        message = TransformStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.world_frame
        message.child_frame_id = self.level_frame
        message.transform.translation.x = float(world_level[0, 3])
        message.transform.translation.y = float(world_level[1, 3])
        message.transform.translation.z = float(world_level[2, 3])
        message.transform.rotation.x = float(quaternion[0])
        message.transform.rotation.y = float(quaternion[1])
        message.transform.rotation.z = float(quaternion[2])
        message.transform.rotation.w = float(quaternion[3])
        self.broadcaster.sendTransform(message)
        rospy.loginfo_throttle(
            5.0,
            "Publishing %s -> %s at (%.3f, %.3f, %.3f)",
            self.world_frame,
            self.level_frame,
            message.transform.translation.x,
            message.transform.translation.y,
            message.transform.translation.z,
        )

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                self._publish_once()
            except Exception as error:
                rospy.logwarn_throttle(5.0, "World truth anchoring failed: %s", error)
            rate.sleep()


def main():
    rospy.init_node("world_truth_anchor")
    WorldTruthAnchor().run()


if __name__ == "__main__":
    main()
