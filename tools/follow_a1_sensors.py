#!/usr/bin/env python3
"""Keep the standalone RGB and laser sensors attached to the moving simulated A1."""

import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from tf.transformations import quaternion_from_euler


def sensor_state(name, x, y, z, yaw, twist):
    state = ModelState()
    state.model_name = name
    state.reference_frame = 'world'
    state.pose.position.x = x
    state.pose.position.y = y
    state.pose.position.z = z
    q = quaternion_from_euler(0.0, 0.0, yaw)
    state.pose.orientation.x, state.pose.orientation.y = q[0], q[1]
    state.pose.orientation.z, state.pose.orientation.w = q[2], q[3]
    state.twist = twist
    return state


def main():
    rospy.init_node('follow_a1_sensors')
    rospy.wait_for_service('/gazebo/get_model_state')
    rospy.wait_for_service('/gazebo/set_model_state')
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            response = get_state('a1_gazebo', 'world')
            p = response.pose.position
            q = response.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            forward_x = math.cos(yaw)
            forward_y = math.sin(yaw)
            x = p.x + forward_x * 1.0
            y = p.y + forward_y * 1.0
            z = p.z + 0.20
            set_state(sensor_state('exploration_rgb_camera', x, y, z, yaw, response.twist))
            set_state(sensor_state('exploration_ray_lidar', x, y, z, yaw, response.twist))
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(5.0, 'sensor follow service: %s', exc)
        rate.sleep()


if __name__ == '__main__':
    main()
