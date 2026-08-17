#!/usr/bin/env python3
"""Move the standalone Gazebo RGB camera with the simulated A1 body."""

import math
import os

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node('follow_a1_rgb_camera')
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    camera_model = os.environ.get('POV_CAMERA_MODEL', 'exploration_rgb_camera')
    offset_x = float(os.environ.get('POV_CAMERA_OFFSET_X', '1.0'))
    offset_z = float(os.environ.get('POV_CAMERA_OFFSET_Z', '0.20'))
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            response = get_state('a1_gazebo', 'world')
            p = response.pose.position
            q = response.pose.orientation
            # The standalone sensor follows the A1 body and uses the same
            # forward (+X) convention as the SDF.  The offsets are
            # configurable so a recording can use a head-height POV without
            # changing the detector's real_sense camera.
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
            state = ModelState()
            state.model_name = camera_model
            state.reference_frame = 'world'
            state.pose.position.x = p.x + math.cos(yaw) * offset_x
            state.pose.position.y = p.y + math.sin(yaw) * offset_x
            state.pose.position.z = p.z + offset_z
            state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w = quaternion_from_euler(0.0, 0.0, yaw)
            state.twist = response.twist
            set_state(state)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(5.0, 'camera follow service: %s', exc)
        rate.sleep()


if __name__ == '__main__':
    main()
