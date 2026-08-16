#!/usr/bin/env python3

import math
import os
import sys
import time
import yaml

import actionlib
import rospy
import tf2_ros

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MissionManager:
    def __init__(self):
        rospy.init_node("mission_manager")

        self.waypoint_file = rospy.get_param(
            "~waypoint_file",
            os.path.expanduser(
                "~/catkin_ws/src/danger_search_robot/"
                "config/mission_waypoints.yaml"
            ),
        )
        self.floor = int(rospy.get_param("~floor", 0))
        self.goal_frame = rospy.get_param("~goal_frame", "world")
        self.server_timeout = float(
            rospy.get_param("~server_timeout", 30.0)
        )
        self.goal_timeout = float(
            rospy.get_param("~goal_timeout", 180.0)
        )
        self.robot_frame = rospy.get_param(
            "~robot_frame",
            "body",
        )
        self.intermediate_reach_tolerance = float(
            rospy.get_param(
                "~intermediate_reach_tolerance",
                0.35,
            )
        )
        self.entry_odom_topic = rospy.get_param(
            "~entry_odom_topic",
            "/Odometry_gazebo",
        )
        entry_direct_param = rospy.get_param(
            "~entry_direct_control",
            None,
        )
        if entry_direct_param is None:
            published_topics = dict(rospy.get_published_topics())
            self.entry_direct_control = (
                self.entry_odom_topic in published_topics
            )
        else:
            self.entry_direct_control = bool(entry_direct_param)
        self.entry_timeout = float(
            rospy.get_param("~entry_timeout", 180.0)
        )
        self.entry_cmd_topic = rospy.get_param(
            "~entry_cmd_topic", "/cmd_vel_direct"
        )
        self.entry_speed = float(
            rospy.get_param("~entry_speed", 0.20)
        )
        self.entry_turn_speed = float(
            rospy.get_param("~entry_turn_speed", 0.45)
        )
        self.entry_heading_threshold = float(
            rospy.get_param("~entry_heading_threshold", 0.20)
        )
        if self.entry_timeout <= 0.0:
            raise RuntimeError("entry_timeout must be > 0")
        if self.entry_speed <= 0.0 or self.entry_turn_speed <= 0.0:
            raise RuntimeError("entry speeds must be > 0")
        if self.entry_heading_threshold <= 0.0:
            raise RuntimeError("entry_heading_threshold must be > 0")
        self.entry_pose = None
        self.entry_cmd_pub = None
        self.entry_odom_sub = None
        if self.entry_direct_control:
            self.entry_cmd_pub = rospy.Publisher(
                self.entry_cmd_topic,
                Twist,
                queue_size=1,
            )
            self.entry_odom_sub = rospy.Subscriber(
                self.entry_odom_topic,
                Odometry,
                self._entry_odom_callback,
                queue_size=1,
            )
            rospy.loginfo(
                "Gazebo truth entry controller enabled: odom=%s "
                "speed=%.2f m/s timeout=%.1f s",
                self.entry_odom_topic,
                self.entry_speed,
                self.entry_timeout,
            )

        self.status_pub = rospy.Publisher(
            "~status",
            String,
            queue_size=1,
            latch=True,
        )
        self.exploration_ready_pub = rospy.Publisher(
            "/exploration_start",
            Bool,
            queue_size=1,
            latch=True,
        )

        self.exploration_ready_pub.publish(False)
        self.publish_status("INITIALIZING")

        self.config = self.load_config()
        self.route = self.build_route()

        self.move_base_client = actionlib.SimpleActionClient(
            "/move_base",
            MoveBaseAction,
        )

        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rospy.Duration(30.0)
        )
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer
        )

        rospy.on_shutdown(self.cancel_navigation)

    def _entry_odom_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.entry_pose = (
            position.x,
            position.y,
            euler_from_quaternion(
                (
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                )
            )[2],
        )

    @staticmethod
    def _wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _zero_twist():
        return Twist()

    def _release_entry_control(self):
        if self.entry_cmd_pub is not None:
            self.entry_cmd_pub.unregister()
            self.entry_cmd_pub = None
        if self.entry_odom_sub is not None:
            self.entry_odom_sub.unregister()
            self.entry_odom_sub = None

    def navigate_through_waypoint_direct(self, waypoint_name, point):
        """Cross the simulated entrance with a closed-loop forward leg.

        The generated entrance contains a shallow apron.  TEB can choose a
        reverse/turning trajectory around its edge even when a straight route
        is valid.  In the fixed Gazebo profile the truth odometry and the
        learned controller are available, so this short entry leg aligns to
        the waypoint and drives forward until the pass-through tolerance is
        met.  Normal room/elevator navigation remains move_base-controlled.
        """

        if self.entry_cmd_pub is None:
            raise RuntimeError(
                "direct entry requested without an entry command publisher"
            )

        self.publish_status(
            "NAVIGATING_THROUGH_" + waypoint_name
        )
        rospy.loginfo(
            "Sending direct entry leg: odom=%s x=%.3f y=%.3f "
            "reach_tolerance=%.2f m",
            self.entry_odom_topic,
            point["x"],
            point["y"],
            self.intermediate_reach_tolerance,
        )
        self.move_base_client.cancel_all_goals()

        deadline = time.monotonic() + self.entry_timeout
        stable_cycles = 0
        while not rospy.is_shutdown():
            if time.monotonic() >= deadline:
                self.entry_cmd_pub.publish(self._zero_twist())
                self.publish_status("TIMEOUT_" + waypoint_name)
                rospy.logerr(
                    "Direct entry to %s timed out after %.1f seconds "
                    "at pose=%s",
                    waypoint_name,
                    self.entry_timeout,
                    self.entry_pose,
                )
                self._release_entry_control()
                return False

            pose = self.entry_pose
            if pose is None:
                self.entry_cmd_pub.publish(self._zero_twist())
                time.sleep(0.05)
                continue

            current_x, current_y, current_yaw = pose
            distance = math.hypot(
                point["x"] - current_x,
                point["y"] - current_y,
            )
            if distance <= self.intermediate_reach_tolerance:
                self.entry_cmd_pub.publish(self._zero_twist())
                stable_cycles += 1
                if stable_cycles >= 8:
                    self.publish_status("PASSED_" + waypoint_name)
                    rospy.loginfo(
                        "Direct entry passed %s: distance=%.3f pose=%s",
                        waypoint_name,
                        distance,
                        self.entry_pose,
                    )
                    self._release_entry_control()
                    return True
                time.sleep(0.05)
                continue

            stable_cycles = 0
            desired_yaw = math.atan2(
                point["y"] - current_y,
                point["x"] - current_x,
            )
            heading_error = self._wrap_angle(
                desired_yaw - current_yaw
            )
            command = Twist()
            if abs(heading_error) > self.entry_heading_threshold:
                command.angular.z = max(
                    -self.entry_turn_speed,
                    min(self.entry_turn_speed, 1.2 * heading_error),
                )
            else:
                command.linear.x = self.entry_speed
                command.angular.z = max(
                    -0.25,
                    min(0.25, 0.6 * heading_error),
                )
            self.entry_cmd_pub.publish(command)
            time.sleep(0.05)

        self.entry_cmd_pub.publish(self._zero_twist())
        self._release_entry_control()
        return False

    def publish_status(self, status):
        self.status_pub.publish(String(data=status))
        rospy.loginfo("[mission_manager] %s", status)

    def load_config(self):
        if not os.path.isfile(self.waypoint_file):
            raise RuntimeError(
                "Waypoint file does not exist: "
                + self.waypoint_file
            )

        with open(self.waypoint_file, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)

        if not isinstance(config, dict):
            raise RuntimeError(
                "Waypoint YAML root must be a dictionary"
            )

        rospy.loginfo(
            "Loaded waypoint file: %s",
            self.waypoint_file,
        )
        return config

    def read_waypoint(self, floor_config, name):
        try:
            point = floor_config[name]
            return {
                "x": float(point["x"]),
                "y": float(point["y"]),
                "yaw": float(point["yaw"]),
            }
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError(
                "Invalid floor_{}.{}: {}".format(
                    self.floor,
                    name,
                    error,
                )
            )

    def build_route(self):
        floor_key = "floor_{}".format(self.floor)

        if floor_key not in self.config:
            raise RuntimeError(
                "Missing floor configuration: " + floor_key
            )

        floor_config = self.config[floor_key]
        route = []

        # 只有一楼初始出生流程需要经过建筑入口。
        if self.floor == 0:
            route.append(
                (
                    "ENTRANCE_INSIDE",
                    self.read_waypoint(
                        floor_config,
                        "entrance_inside",
                    ),
                )
            )

        route.append(
            (
                "EXPLORATION_START",
                self.read_waypoint(
                    floor_config,
                    "exploration_start",
                ),
            )
        )

        rospy.loginfo(
            "Mission route: %s",
            " -> ".join(name for name, _ in route),
        )
        return route

    def wait_for_move_base(self):
        self.publish_status("WAITING_FOR_MOVE_BASE")

        # 使用系统墙钟时间，避免 /clock 从零跳变造成假超时。
        deadline = time.monotonic() + self.server_timeout

        while not rospy.is_shutdown():
            try:
                topics = dict(rospy.get_published_topics())
            except rospy.ROSException:
                topics = {}

            if "/move_base/status" in topics:
                rospy.loginfo(
                    "Detected /move_base/status"
                )

                self.move_base_client.wait_for_server()

                rospy.loginfo(
                    "/move_base action server is ready"
                )
                return

            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "/move_base action server was not available "
                    "within {:.1f} wall-clock seconds".format(
                        self.server_timeout
                    )
                )

            time.sleep(0.2)

        raise rospy.ROSInterruptException()

    def make_goal(self, point):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.goal_frame
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = point["x"]
        goal.target_pose.pose.position.y = point["y"]
        goal.target_pose.pose.position.z = 0.0

        quaternion = quaternion_from_euler(
            0.0,
            0.0,
            point["yaw"],
        )

        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        return goal

    def navigate_to_waypoint(self, waypoint_name, point):
        self.publish_status(
            "NAVIGATING_TO_" + waypoint_name
        )

        rospy.loginfo(
            "Sending %s goal: frame=%s, "
            "x=%.3f, y=%.3f, yaw=%.3f",
            waypoint_name,
            self.goal_frame,
            point["x"],
            point["y"],
            point["yaw"],
        )

        self.move_base_client.send_goal(
            self.make_goal(point)
        )

        # goal_timeout <= 0 时禁用导航超时。
        deadline = (
            None
            if self.goal_timeout <= 0.0
            else time.monotonic() + self.goal_timeout
        )

        terminal_failures = {
            GoalStatus.PREEMPTED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        }

        while not rospy.is_shutdown():
            state = self.move_base_client.get_state()

            if state == GoalStatus.SUCCEEDED:
                self.publish_status(
                    "REACHED_" + waypoint_name
                )
                rospy.loginfo(
                    "Reached waypoint %s",
                    waypoint_name,
                )
                return True

            if state in terminal_failures:
                self.publish_status(
                    "FAILED_" + waypoint_name
                    + "_STATE_{}".format(state)
                )
                rospy.logerr(
                    "Navigation to %s failed: state=%d, text=%s",
                    waypoint_name,
                    state,
                    self.move_base_client.get_goal_status_text(),
                )
                return False

            if (
                deadline is not None
                and time.monotonic() >= deadline
            ):
                self.move_base_client.cancel_goal()
                self.publish_status(
                    "TIMEOUT_" + waypoint_name
                )
                rospy.logerr(
                    "Navigation to %s timed out after %.1f seconds",
                    waypoint_name,
                    self.goal_timeout,
                )
                return False

            time.sleep(0.1)

        return False

    def distance_to_waypoint(self, point):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.goal_frame,
                self.robot_frame,
                rospy.Time(0),
                rospy.Duration(0.20),
            )
        except Exception as error:
            rospy.logwarn_throttle(
                2.0,
                "Cannot read %s -> %s TF: %s",
                self.goal_frame,
                self.robot_frame,
                error,
            )
            return None

        current_x = transform.transform.translation.x
        current_y = transform.transform.translation.y

        return math.hypot(
            point["x"] - current_x,
            point["y"] - current_y,
        )

    def navigate_through_waypoint(
        self,
        waypoint_name,
        point,
    ):
        """经过中间航点，不要求精确停稳和满足最终朝向。"""

        self.publish_status(
            "NAVIGATING_THROUGH_" + waypoint_name
        )

        rospy.loginfo(
            "Sending pass-through goal %s: "
            "frame=%s, x=%.3f, y=%.3f, yaw=%.3f, "
            "reach_tolerance=%.2f m",
            waypoint_name,
            self.goal_frame,
            point["x"],
            point["y"],
            point["yaw"],
            self.intermediate_reach_tolerance,
        )

        self.move_base_client.send_goal(
            self.make_goal(point)
        )

        # goal_timeout <= 0 时禁用导航超时。
        deadline = (
            None
            if self.goal_timeout <= 0.0
            else time.monotonic() + self.goal_timeout
        )

        terminal_failures = {
            GoalStatus.PREEMPTED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        }

        while not rospy.is_shutdown():
            distance = self.distance_to_waypoint(point)

            if (
                distance is not None
                and distance
                <= self.intermediate_reach_tolerance
            ):
                rospy.loginfo(
                    "Pass-through waypoint %s reached: "
                    "distance=%.3f m",
                    waypoint_name,
                    distance,
                )

                # 不等待 TEB 原地精确调整最终朝向。
                self.move_base_client.cancel_goal()
                time.sleep(0.20)

                self.publish_status(
                    "PASSED_" + waypoint_name
                )
                return True

            state = self.move_base_client.get_state()

            if state == GoalStatus.SUCCEEDED:
                self.publish_status(
                    "PASSED_" + waypoint_name
                )
                return True

            if state in terminal_failures:
                self.publish_status(
                    "FAILED_" + waypoint_name
                    + "_STATE_{}".format(state)
                )
                rospy.logerr(
                    "Navigation through %s failed: "
                    "state=%d, text=%s",
                    waypoint_name,
                    state,
                    self.move_base_client.get_goal_status_text(),
                )
                return False

            if (
                deadline is not None
                and time.monotonic() >= deadline
            ):
                self.move_base_client.cancel_goal()
                self.publish_status(
                    "TIMEOUT_" + waypoint_name
                )
                rospy.logerr(
                    "Navigation through %s timed out",
                    waypoint_name,
                )
                return False

            time.sleep(0.10)

        return False

    def cancel_navigation(self):
        if hasattr(self, "move_base_client"):
            self.move_base_client.cancel_all_goals()
        if self.entry_cmd_pub is not None:
            self.entry_cmd_pub.publish(self._zero_twist())
            self._release_entry_control()

    def run(self):
        self.wait_for_move_base()

        for waypoint_name, point in self.route:
            if waypoint_name == "ENTRANCE_INSIDE":
                if self.entry_direct_control:
                    success = self.navigate_through_waypoint_direct(
                        waypoint_name,
                        point,
                    )
                else:
                    success = self.navigate_through_waypoint(
                        waypoint_name,
                        point,
                    )
            else:
                success = self.navigate_to_waypoint(
                    waypoint_name,
                    point,
                )

            if not success:
                self.exploration_ready_pub.publish(False)
                self.publish_status("MISSION_FAILED")
                return

            # 给机器人和局部规划器留出短暂停稳时间。
            time.sleep(1.0)

        self.exploration_ready_pub.publish(True)
        self.publish_status("READY_FOR_EXPLORATION")
        rospy.loginfo(
            "Initial navigation route completed"
        )

        rospy.spin()


def main():
    try:
        MissionManager().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as error:
        rospy.logfatal(
            "mission_manager failed: %s",
            error,
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
