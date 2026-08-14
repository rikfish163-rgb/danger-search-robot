#!/usr/bin/env python3
"""Bridge FAST-LIO odometry into the TF tree.

Some FAST-LIO builds publish ``/Odometry`` but omit the matching dynamic TF.
The navigation and exploration nodes consume TF, so this bridge republishes
the exact pose from the odometry message as ``camera_init -> body``.
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class OdometryTfBridge:
    def __init__(self):
        self.parent_frame = rospy.get_param("~parent_frame", "camera_init")
        self.child_frame = rospy.get_param("~child_frame", "body")
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.last_stamp = rospy.Time(0)
        self.subscriber = rospy.Subscriber(
            rospy.get_param("~odometry_topic", "/Odometry"),
            Odometry,
            self._publish_tf,
            queue_size=10,
        )

    def _publish_tf(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        if stamp < self.last_stamp:
            rospy.logwarn_throttle(
                5.0,
                "Ignoring out-of-order odometry stamp %.3f",
                stamp.to_sec(),
            )
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)
        self.last_stamp = stamp


def main():
    rospy.init_node("odometry_tf_bridge")
    OdometryTfBridge()
    rospy.loginfo("Republishing %s -> %s from FAST-LIO odometry", "camera_init", "body")
    rospy.spin()


if __name__ == "__main__":
    main()
