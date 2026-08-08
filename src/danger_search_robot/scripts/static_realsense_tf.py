#!/usr/bin/env python3
"""StaticTransformBroadcaster 发布 body -> slam_real_sense_optical_frame 到 /tf_static.
替代 vision_stack.launch 里坏掉的 static_transform_publisher (不发布/tf_static). """
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def main():
    rospy.init_node("static_realsense_tf")
    br = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "body"
    t.child_frame_id = "slam_real_sense_optical_frame"
    t.transform.translation.x = 0.093743593234
    t.transform.translation.y = 0.02329
    t.transform.translation.z = -0.013747351471
    t.transform.rotation.x = -0.270728101097
    t.transform.rotation.y = 0.270728101094
    t.transform.rotation.z = -0.653227598373
    t.transform.rotation.w = 0.653227598374
    br.sendTransform(t)
    rospy.spin()

if __name__ == "__main__":
    main()
