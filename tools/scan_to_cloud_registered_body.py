#!/usr/bin/env python3
"""Convert the live Gazebo ray scan into the projection cloud interface."""

import math

import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs import point_cloud2


class ScanCloudBridge:
    def __init__(self):
        self.publisher = rospy.Publisher("/cloud_registered_body", PointCloud2, queue_size=2)
        self.body_pose = None
        self.sensor_pose = None
        rospy.Subscriber("/stable_scan", LaserScan, self._scan, queue_size=2)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states, queue_size=1)

    def _model_states(self, message):
        try:
            body = message.pose[message.name.index("a1_gazebo")]
            sensor = message.pose[message.name.index("exploration_ray_lidar")]
        except (ValueError, IndexError):
            return
        self.body_pose = body
        self.sensor_pose = sensor

    def _scan(self, message):
        if self.body_pose is None or self.sensor_pose is None:
            return
        body = self.body_pose
        sensor = self.sensor_pose
        body_yaw = self._yaw(body.orientation)
        sensor_yaw = self._yaw(sensor.orientation)
        dx = sensor.position.x - body.position.x
        dy = sensor.position.y - body.position.y
        relative_x = math.cos(body_yaw) * dx + math.sin(body_yaw) * dy
        relative_y = -math.sin(body_yaw) * dx + math.cos(body_yaw) * dy
        relative_yaw = sensor_yaw - body_yaw
        cos_relative = math.cos(relative_yaw)
        sin_relative = math.sin(relative_yaw)

        points = []
        angle = message.angle_min
        for distance in message.ranges:
            if message.range_min <= distance <= message.range_max:
                sensor_x = distance * math.cos(angle)
                sensor_y = distance * math.sin(angle)
                points.append((
                    relative_x + cos_relative * sensor_x - sin_relative * sensor_y,
                    relative_y + sin_relative * sensor_x + cos_relative * sensor_y,
                    sensor.position.z - body.position.z,
                ))
            angle += message.angle_increment
        cloud = point_cloud2.create_cloud_xyz32(message.header, points)
        cloud.header.frame_id = "body"
        self.publisher.publish(cloud)

    @staticmethod
    def _yaw(quaternion):
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )


if __name__ == "__main__":
    rospy.init_node("scan_to_cloud_registered_body")
    ScanCloudBridge()
    rospy.spin()
