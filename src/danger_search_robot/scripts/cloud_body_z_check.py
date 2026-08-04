#!/usr/bin/env python3

import rospy
import tf
import numpy as np

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


listener = None


def callback(msg):

    try:
        trans, rot = listener.lookupTransform(
            "body",
            "camera_init",
            rospy.Time(0)
        )

    except Exception:
        return


    T = tf.transformations.quaternion_matrix(rot)
    T[0:3,3] = np.array(trans)


    zs = []

    for p in pc2.read_points(
            msg,
            field_names=("x","y","z"),
            skip_nans=True):

        point = np.array(
            [
                p[0],
                p[1],
                p[2],
                1
            ]
        )

        # camera_init -> body
        pb = np.dot(T, point)

        zs.append(pb[2])


    if len(zs)==0:
        return


    print("====================")
    print("points:", len(zs))
    print("min z:", min(zs))
    print("max z:", max(zs))
    print("mean z:", sum(zs)/len(zs))

    bins=[
        -1,
        -0.5,
        -0.2,
        0,
        0.2,
        0.5,
        1,
        1.5,
        2,
        3
    ]

    count=[0]*(len(bins)-1)

    for z in zs:
        for i in range(len(bins)-1):
            if bins[i] <= z < bins[i+1]:
                count[i]+=1


    for i,c in enumerate(count):
        print(
            "{}~{} m : {}".format(
                bins[i],
                bins[i+1],
                c
            )
        )

    print("====================")


rospy.init_node("cloud_body_z_check")

listener=tf.TransformListener()

rospy.Subscriber(
    "/cloud_registered",
    PointCloud2,
    callback,
    queue_size=1
)

rospy.spin()
