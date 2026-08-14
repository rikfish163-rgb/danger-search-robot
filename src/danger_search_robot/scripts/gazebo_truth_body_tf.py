#!/usr/bin/env python3
"""Publish the simulator model pose as the live world -> body TF."""

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped


class TruthBodyTf:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "a1_gazebo")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.body_frame = rospy.get_param("~body_frame", "body")
        self.pose = None
        self.last_stamp = rospy.Time(0)
        self.broadcast = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states, queue_size=1)

    def _model_states(self, message):
        try:
            pose = message.pose[message.name.index(self.model_name)]
        except (ValueError, IndexError):
            return
        stamp = rospy.Time.now()
        if stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.body_frame
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        self.broadcast.sendTransform(transform)


if __name__ == "__main__":
    rospy.init_node("gazebo_truth_body_tf")
    node = TruthBodyTf()
    rospy.spin()
