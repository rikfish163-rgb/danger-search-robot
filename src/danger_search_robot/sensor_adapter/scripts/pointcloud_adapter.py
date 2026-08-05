#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2


pub = None


def callback(msg):

    points = []

    for p in pc2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True):

        x, y, z = p

        # 添加默认强度
        intensity = 0.0

        points.append(
            [x, y, z, intensity]
        )


    fields = [
        PointField(
            'x',
            0,
            PointField.FLOAT32,
            1),

        PointField(
            'y',
            4,
            PointField.FLOAT32,
            1),

        PointField(
            'z',
            8,
            PointField.FLOAT32,
            1),

        PointField(
            'intensity',
            12,
            PointField.FLOAT32,
            1)
    ]


    cloud = pc2.create_cloud(
        msg.header,
        fields,
        points
    )


    pub.publish(cloud)



if __name__ == "__main__":

    rospy.init_node(
        "pointcloud_adapter"
    )

    pub = rospy.Publisher(
        "/fastlio/cloud",
        PointCloud2,
        queue_size=10
    )


    rospy.Subscriber(
        "/livox/Pointcloud2",
        PointCloud2,
        callback,
        queue_size=10
    )


    rospy.spin()
