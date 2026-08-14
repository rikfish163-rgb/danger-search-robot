#!/usr/bin/env python3
"""Restore the Gazebo depth plugin's flattened pixel cloud organization."""

import copy

import rospy
from sensor_msgs.msg import PointCloud2


def main():
    rospy.init_node("depth_flat_to_organized")
    publisher = rospy.Publisher(
        "/real_sense/depth/points_organized", PointCloud2, queue_size=2
    )

    def callback(message):
        organized = copy.copy(message)
        organized.height = 480
        organized.width = 640
        organized.row_step = organized.point_step * organized.width
        publisher.publish(organized)

    rospy.Subscriber(
        "/real_sense/depth/points", PointCloud2, callback, queue_size=2
    )
    rospy.spin()


if __name__ == "__main__":
    main()
