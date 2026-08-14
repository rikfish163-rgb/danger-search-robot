#!/usr/bin/env python3
"""Bridge the stable Gazebo LaserScan into the PointCloud interface used by the Livox adapter."""

import math

import rospy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud


class LaserScanBridge:
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'laser_livox')
        self.pub = rospy.Publisher('/scan', PointCloud, queue_size=5)
        rospy.Subscriber('/stable_scan', LaserScan, self._on_scan, queue_size=5)
        rospy.loginfo('LaserScan bridge: /stable_scan -> /scan (%s)', self.frame_id)

    def _on_scan(self, scan):
        cloud = PointCloud()
        cloud.header = scan.header
        cloud.header.frame_id = self.frame_id
        angle = scan.angle_min
        for distance in scan.ranges:
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                cloud.points.append(Point32(
                    x=distance * math.cos(angle),
                    y=distance * math.sin(angle),
                    z=0.0,
                ))
            angle += scan.angle_increment
        self.pub.publish(cloud)


if __name__ == '__main__':
    rospy.init_node('stable_laser_scan_to_pointcloud')
    LaserScanBridge()
    rospy.spin()
