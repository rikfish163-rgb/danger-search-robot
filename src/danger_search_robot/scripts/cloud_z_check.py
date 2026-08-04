#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def callback(msg):

    zs=[]

    for p in pc2.read_points(
        msg,
        field_names=("x","y","z"),
        skip_nans=True):

        x,y,z=p
        zs.append(z)


    if len(zs)==0:
        return


    print("====================")
    print("points:",len(zs))
    print("min z:",min(zs))
    print("max z:",max(zs))
    print("mean z:",sum(zs)/len(zs))


    bins=[
        -1,
        0,
        0.2,
        0.5,
        1,
        1.5,
        2,
        3,
        4,
        5
    ]


    count=[0]*(len(bins)-1)


    for z in zs:
        for i in range(len(bins)-1):

            if bins[i]<=z<bins[i+1]:
                count[i]+=1


    for i in range(len(count)):
        print(
            "{}~{} m : {}".format(
                bins[i],
                bins[i+1],
                count[i]
            )
        )


    print("====================")


    rospy.signal_shutdown("done")


rospy.init_node(
    "cloud_z_check"
)


rospy.Subscriber(
    "/cloud_registered",
    PointCloud2,
    callback
)


rospy.spin()
