#!/usr/bin/env python3
"""StaticTransformBroadcaster 发布 map_level -> camera_init 到 /tf_static。
解决 ROS1 tf 库对动态 root frame 的识别问题。"""
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def main():
    rospy.init_node("static_level_tf")
    br = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map_level"
    t.child_frame_id = "camera_init"
    t.transform.rotation.y = 0.382683432
    t.transform.rotation.w = 0.923879533
    br.sendTransform(t)
    rospy.spin()


if __name__ == "__main__":
    main()
