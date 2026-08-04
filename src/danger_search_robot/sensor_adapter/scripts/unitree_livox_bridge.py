#!/usr/bin/env python3

import rospy

from unitree_guide.msg import CustomMsg as UnitreeCustomMsg
from livox_ros_driver.msg import CustomMsg as LivoxCustomMsg
from livox_ros_driver.msg import CustomPoint as LivoxCustomPoint


class UnitreeLivoxBridge:

    def __init__(self):

        self.input_topic = "/livox/lidar2"
        self.output_topic = "/livox/lidar"

        self.pub = rospy.Publisher(
            self.output_topic,
            LivoxCustomMsg,
            queue_size=10
        )

        self.sub = rospy.Subscriber(
            self.input_topic,
            UnitreeCustomMsg,
            self.callback,
            queue_size=10
        )

        rospy.loginfo(
            "unitree_livox_bridge started"
        )


    def callback(self, msg):

        out = LivoxCustomMsg()

        # header
        out.header = msg.header

        # lidar信息
        out.timebase = msg.timebase
        out.lidar_id = msg.lidar_id
        out.rsvd = msg.rsvd


        points = []

        for p in msg.points:

            q = LivoxCustomPoint()

            q.offset_time = p.offset_time

            q.x = p.x
            q.y = p.y
            q.z = p.z

            q.reflectivity = p.reflectivity

            q.tag = p.tag

            q.line = p.line

            points.append(q)


        out.points = points
        out.point_num = len(points)


        self.pub.publish(out)



if __name__ == "__main__":

    rospy.init_node(
        "unitree_livox_bridge"
    )

    node = UnitreeLivoxBridge()

    rospy.spin()