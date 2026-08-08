#!/usr/bin/env python3
"""StaticTransformBroadcaster 发布 map_level -> camera_init (0度, IMU补偿方案).
IMU已补偿到base系(水平), camera_init即水平, 无需45度倾斜补偿. """
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def main():
    rospy.init_node("static_level_tf_0")
    br = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map_level"
    t.child_frame_id = "camera_init"
    t.transform.rotation.w = 1.0
    br.sendTransform(t)
    rospy.spin()

if __name__ == "__main__":
    main()
