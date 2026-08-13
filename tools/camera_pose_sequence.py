#!/usr/bin/env python3
import math
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler


POSES = [
    (5.5, 12.753, 5.45, 0.0, 'danger_red_sphere_00'),
    (-4.5, 13.453, 5.45, math.pi, 'danger_red_sphere_01'),
    (-0.5, 11.73, 5.45, 0.0, 'danger_red_sphere_02'),
    (-6.0, 30.77, 5.45, 0.0, 'danger_red_sphere_03'),
]


def main():
    rospy.init_node('camera_pose_sequence')
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    for x, y, z, yaw, label in POSES:
        state = ModelState()
        state.model_name = 'exploration_rgb_camera'
        state.reference_frame = 'world'
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        q = quaternion_from_euler(0.0, 0.0, yaw)
        state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w = q
        result = set_state(state)
        print('%s success=%s message=%s pose=(%.3f, %.3f, %.3f, %.3f)' %
              (label, result.success, result.status_message, x, y, z, yaw), flush=True)
        time.sleep(5.0)


if __name__ == '__main__':
    main()
