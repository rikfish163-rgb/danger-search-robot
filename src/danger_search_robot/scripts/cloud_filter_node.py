#!/usr/bin/env python3

import rospy
import tf
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2


class CloudFilter:

    def __init__(self):

        self.listener = tf.TransformListener()

        self.pub = rospy.Publisher(
            "/cloud_filtered",
            PointCloud2,
            queue_size=1
        )


        rospy.Subscriber(
            "/cloud_registered",
            PointCloud2,
            self.callback,
            queue_size=1
        )


        # 参数
        self.z_min = rospy.get_param(
            "~z_min",
            0.1
        )

        self.z_max = rospy.get_param(
            "~z_max",
            2.5
        )


        self.range_limit = rospy.get_param(
            "~range_limit",
            10.0
        )


        rospy.loginfo(
            "Cloud filter started"
        )

        rospy.loginfo(
            "z range: %.2f ~ %.2f",
            self.z_min,
            self.z_max
        )


    def callback(self,msg):


        try:

            trans,rot = self.listener.lookupTransform(
                "body",
                "camera_init",
                rospy.Time(0)
            )


        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException
        ):

            return



        T = tf.transformations.quaternion_matrix(rot)

        T[0:3,3] = np.array(trans)



        points=[]


        for p in pc2.read_points(
                msg,
                field_names=("x","y","z"),
                skip_nans=True):


            point=np.array(
                [
                    p[0],
                    p[1],
                    p[2],
                    1
                ]
            )


            # camera_init -> body

            pb=np.dot(
                T,
                point
            )


            x=pb[0]
            y=pb[1]
            z=pb[2]


            distance=np.sqrt(
                x*x+y*y
            )


            # 距离限制
            if distance > self.range_limit:
                continue


            # 高度限制
            if z < self.z_min:
                continue


            if z > self.z_max:
                continue


            points.append(
                [
                    x,
                    y,
                    z
                ]
            )


        if len(points)==0:
            return



        fields=[

            PointField(
                "x",
                0,
                PointField.FLOAT32,
                1
            ),

            PointField(
                "y",
                4,
                PointField.FLOAT32,
                1
            ),

            PointField(
                "z",
                8,
                PointField.FLOAT32,
                1
            )
        ]



        cloud=pc2.create_cloud(
            msg.header,
            fields,
            points
        )


        # 注意修改frame_id
        cloud.header.frame_id="body"


        self.pub.publish(
            cloud
        )



if __name__=="__main__":


    rospy.init_node(
        "cloud_filter_node"
    )


    node=CloudFilter()


    rospy.spin()
