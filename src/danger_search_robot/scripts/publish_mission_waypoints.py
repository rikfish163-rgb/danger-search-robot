#!/usr/bin/env python3

import math
import os
import yaml

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler


class WaypointVisualizer:
    def __init__(self):
        rospy.init_node("mission_waypoint_visualizer")

        self.config_file = rospy.get_param(
            "~waypoint_file",
            os.path.expanduser(
                "~/catkin_ws/src/danger_search_robot/"
                "config/mission_waypoints.yaml"
            ),
        )
        self.floor = int(rospy.get_param("~floor", 0))
        self.frame_id = rospy.get_param("~frame_id", "world")

        self.publisher = rospy.Publisher(
            "/mission_waypoints",
            MarkerArray,
            queue_size=1,
            latch=True,
        )

        self.config = self.load_config()
        rospy.sleep(0.5)
        self.publish_markers()

    def load_config(self):
        with open(self.config_file, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)

        floor_key = "floor_{}".format(self.floor)
        if floor_key not in config:
            raise RuntimeError("Missing configuration: " + floor_key)

        return config[floor_key]

    @staticmethod
    def marker_color(name):
        colors = {
            "entrance_inside": (1.0, 0.3, 0.1),
            "exploration_start": (0.1, 1.0, 0.2),
            "elevator_wait": (0.2, 0.5, 1.0),
            "elevator_inside": (0.8, 0.2, 1.0),
        }
        return colors.get(name, (1.0, 1.0, 1.0))

    def publish_markers(self):
        marker_array = MarkerArray()
        marker_id = 0

        ordered_names = [
            "entrance_inside",
            "exploration_start",
            "elevator_wait",
            "elevator_inside",
        ]

        for name in ordered_names:
            if name not in self.config:
                continue

            point = self.config[name]
            x = float(point["x"])
            y = float(point["y"])
            yaw = float(point["yaw"])

            quaternion = quaternion_from_euler(0.0, 0.0, yaw)
            red, green, blue = self.marker_color(name)

            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "mission_waypoint_points"
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.20
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.35
            sphere.scale.y = 0.35
            sphere.scale.z = 0.35
            sphere.color.r = red
            sphere.color.g = green
            sphere.color.b = blue
            sphere.color.a = 1.0
            marker_array.markers.append(sphere)

            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.header.stamp = rospy.Time.now()
            arrow.ns = "mission_waypoint_directions"
            arrow.id = marker_id
            marker_id += 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.25
            arrow.pose.orientation.x = quaternion[0]
            arrow.pose.orientation.y = quaternion[1]
            arrow.pose.orientation.z = quaternion[2]
            arrow.pose.orientation.w = quaternion[3]
            arrow.scale.x = 0.80
            arrow.scale.y = 0.12
            arrow.scale.z = 0.12
            arrow.color.r = red
            arrow.color.g = green
            arrow.color.b = blue
            arrow.color.a = 1.0
            marker_array.markers.append(arrow)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = rospy.Time.now()
            label.ns = "mission_waypoint_labels"
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.75
            label.pose.orientation.w = 1.0
            label.scale.z = 0.30
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = "{}\n({:.3f}, {:.3f})\nyaw={:.1f} deg".format(
                name,
                x,
                y,
                math.degrees(yaw),
            )
            marker_array.markers.append(label)

        self.publisher.publish(marker_array)

        rospy.loginfo(
            "Published %d waypoint markers for floor %d in frame %s",
            len(marker_array.markers),
            self.floor,
            self.frame_id,
        )


if __name__ == "__main__":
    try:
        WaypointVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
