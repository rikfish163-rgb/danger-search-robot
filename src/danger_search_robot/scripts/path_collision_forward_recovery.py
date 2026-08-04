#!/usr/bin/env python3

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String


class PathCollisionForwardRecovery:
    def __init__(self):
        self.costmap_topic = rospy.get_param(
            "~local_costmap_topic",
            "/move_base/local_costmap/costmap",
        )
        self.plan_topic = rospy.get_param(
            "~local_plan_topic",
            "/move_base/TebLocalPlannerROS/local_plan",
        )
        self.cmd_vel_topic = rospy.get_param(
            "~cmd_vel_topic",
            "/cmd_vel",
        )

        self.collision_hold_time = float(
            rospy.get_param("~collision_hold_time", 5.0)
        )
        self.forward_duration = float(
            rospy.get_param("~forward_duration", 1.0)
        )
        self.forward_speed = float(
            rospy.get_param("~forward_speed", 0.10)
        )
        self.check_path_distance = float(
            rospy.get_param("~check_path_distance", 1.50)
        )
        self.collision_cost_threshold = int(
            rospy.get_param("~collision_cost_threshold", 100)
        )

        self.lock = threading.RLock()

        self.latest_costmap = None
        self.latest_plan = None

        self.collision_since = None
        self.forward_start = None
        self.forward_active = False

        self.cmd_pub = rospy.Publisher(
            self.cmd_vel_topic,
            Twist,
            queue_size=20,
        )
        self.status_pub = rospy.Publisher(
            "~status",
            String,
            queue_size=10,
            latch=True,
        )

        rospy.Subscriber(
            self.costmap_topic,
            OccupancyGrid,
            self.costmap_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.plan_topic,
            Path,
            self.plan_callback,
            queue_size=1,
        )

        # 使用 ROS 仿真时间运行，50 Hz 持续发布直行速度。
        self.timer = rospy.Timer(
            rospy.Duration(0.02),
            self.timer_callback,
        )

        self.publish_status("WAITING_FOR_DATA")

        rospy.loginfo(
            "Path collision recovery started: "
            "collision %.2fs -> straight %.2fs at %.3fm/s",
            self.collision_hold_time,
            self.forward_duration,
            self.forward_speed,
        )

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def costmap_callback(self, message):
        with self.lock:
            self.latest_costmap = message

    def plan_callback(self, message):
        with self.lock:
            self.latest_plan = message

    @staticmethod
    def grid_value(grid, x, y):
        resolution = float(grid.info.resolution)
        origin_x = float(grid.info.origin.position.x)
        origin_y = float(grid.info.origin.position.y)

        grid_x = int(math.floor(
            (x - origin_x) / resolution
        ))
        grid_y = int(math.floor(
            (y - origin_y) / resolution
        ))

        if (
            grid_x < 0
            or grid_y < 0
            or grid_x >= int(grid.info.width)
            or grid_y >= int(grid.info.height)
        ):
            return None

        index = grid_y * int(grid.info.width) + grid_x
        return grid.data[index]

    def segment_collides(
        self,
        grid,
        x1,
        y1,
        x2,
        y2,
    ):
        length = math.hypot(
            x2 - x1,
            y2 - y1,
        )

        # 每半个栅格采样一次，避免路径点间距较大时漏检。
        step = max(
            0.01,
            float(grid.info.resolution) * 0.5,
        )
        sample_count = max(
            1,
            int(math.ceil(length / step)),
        )

        for sample_index in range(sample_count + 1):
            ratio = float(sample_index) / float(sample_count)

            x = x1 + ratio * (x2 - x1)
            y = y1 + ratio * (y2 - y1)

            value = self.grid_value(grid, x, y)

            if (
                value is not None
                and value >= self.collision_cost_threshold
            ):
                return True

        return False

    def path_collides(self, grid, path):
        if not path.poses:
            return False

        path_frame = path.header.frame_id

        if not path_frame:
            path_frame = path.poses[0].header.frame_id

        if path_frame != grid.header.frame_id:
            rospy.logwarn_throttle(
                2.0,
                "Frame mismatch: local_plan=%s costmap=%s",
                path_frame,
                grid.header.frame_id,
            )
            return False

        previous_x = path.poses[0].pose.position.x
        previous_y = path.poses[0].pose.position.y
        travelled_distance = 0.0

        first_value = self.grid_value(
            grid,
            previous_x,
            previous_y,
        )

        if (
            first_value is not None
            and first_value >= self.collision_cost_threshold
        ):
            return True

        for pose_stamped in path.poses[1:]:
            current_x = pose_stamped.pose.position.x
            current_y = pose_stamped.pose.position.y

            segment_length = math.hypot(
                current_x - previous_x,
                current_y - previous_y,
            )

            remaining_distance = (
                self.check_path_distance - travelled_distance
            )

            if remaining_distance <= 0.0:
                break

            if (
                segment_length > remaining_distance
                and segment_length > 1e-9
            ):
                ratio = remaining_distance / segment_length

                end_x = previous_x + ratio * (
                    current_x - previous_x
                )
                end_y = previous_y + ratio * (
                    current_y - previous_y
                )
            else:
                end_x = current_x
                end_y = current_y

            if self.segment_collides(
                grid,
                previous_x,
                previous_y,
                end_x,
                end_y,
            ):
                return True

            travelled_distance += min(
                segment_length,
                remaining_distance,
            )

            if segment_length > remaining_distance:
                break

            previous_x = current_x
            previous_y = current_y

        return False

    def timer_callback(self, _event):
        now = rospy.Time.now()

        if now.to_sec() <= 0.0:
            self.publish_status("WAITING_FOR_CLOCK")
            return

        # 触发后不取消目标、不判断前方，直接持续直行。
        if self.forward_active:
            elapsed = (
                now - self.forward_start
            ).to_sec()

            if elapsed < 0.0:
                self.forward_start = now
                elapsed = 0.0

            if elapsed < self.forward_duration:
                command = Twist()
                command.linear.x = self.forward_speed
                command.angular.z = 0.0

                self.cmd_pub.publish(command)

                self.publish_status(
                    "FORWARD {:.2f}/{:.2f}s".format(
                        elapsed,
                        self.forward_duration,
                    )
                )
                return

            # 直行一秒结束，随后立即恢复路径碰撞判断。
            self.cmd_pub.publish(Twist())

            self.forward_active = False
            self.forward_start = None
            self.collision_since = None

            self.publish_status("RECHECKING")
            rospy.loginfo(
                "Straight motion finished; rechecking path"
            )
            return

        with self.lock:
            grid = self.latest_costmap
            path = self.latest_plan

        if grid is None or path is None:
            self.publish_status("WAITING_FOR_DATA")
            return

        collision = self.path_collides(
            grid,
            path,
        )

        if not collision:
            self.collision_since = None
            self.publish_status("PATH_CLEAR")
            return

        if self.collision_since is None:
            self.collision_since = now

            rospy.logwarn(
                "Displayed local path intersects obstacle; "
                "starting %.2fs simulation-time timer",
                self.collision_hold_time,
            )

        elapsed = (
            now - self.collision_since
        ).to_sec()

        if elapsed < 0.0:
            self.collision_since = now
            elapsed = 0.0

        self.publish_status(
            "COLLISION {:.2f}/{:.2f}s".format(
                elapsed,
                self.collision_hold_time,
            )
        )

        if elapsed >= self.collision_hold_time:
            self.forward_active = True
            self.forward_start = now
            self.collision_since = None

            command = Twist()
            command.linear.x = self.forward_speed
            command.angular.z = 0.0

            self.cmd_pub.publish(command)

            self.publish_status(
                "FORWARD 0.00/{:.2f}s".format(
                    self.forward_duration
                )
            )

            rospy.logwarn(
                "Collision persisted for %.2fs; "
                "forcing straight motion",
                self.collision_hold_time,
            )


def main():
    rospy.init_node(
        "path_collision_forward_recovery"
    )

    PathCollisionForwardRecovery()
    rospy.spin()


if __name__ == "__main__":
    main()
