#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""动态广播 body -> slam_real_sense_optical_frame。

替代原来的 static_transform_publisher：静态发布会让 body 在 /tf_static 里成为
独立根节点，与 FAST-LIO 的动态 camera_init->body 分属两棵树，导致 move_base 的
C++ tf2 缓冲区报 "two or more unconnected trees"。这里改用 TransformBroadcaster
以 10Hz 动态广播同一个恒定变换，使 body 全程保持动态帧，树保持连通。
"""
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def main():
    rospy.init_node("slam_realsense_optical_tf")
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    t = TransformStamped()
    t.header.frame_id = "body"
    t.child_frame_id = "slam_real_sense_optical_frame"
    t.transform.translation.x = 0.093743593234
    t.transform.translation.y = 0.023290000000
    t.transform.translation.z = -0.013747351471
    t.transform.rotation.x = -0.270728101097
    t.transform.rotation.y = 0.270728101094
    t.transform.rotation.z = -0.653227598373
    t.transform.rotation.w = 0.653227598374

    while not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        br.sendTransform(t)
        rate.sleep()


if __name__ == "__main__":
    main()
