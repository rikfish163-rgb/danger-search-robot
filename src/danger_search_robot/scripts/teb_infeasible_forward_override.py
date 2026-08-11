#!/usr/bin/env python3
"""Exclusive cmd_vel gateway with a two-stage TEB recovery override.

Normal mode forwards /cmd_vel_nav to /cmd_vel.  The first matching TEB
infeasibility warning forces a straight motion.  If recovery is still needed,
the node compares occupied cells on the robot's left and right in the local
costmap and strafes away from the more obstructed side.  Warnings received
during recovery are retained as pending evidence.  The same alternating
recovery can also be triggered when an active move_base goal has made no actual
motion for a configured duration and either navigation commands are ineffective
or a recent global-planner failure was observed.  The goal is never canceled.
"""

import math
import threading

import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Log
from tf.transformations import euler_from_quaternion


class TebInfeasibleRecoveryOverride:
    FORWARD = "FORWARD"
    LATERAL = "LATERAL"

    def __init__(self):
        self.nav_cmd_topic = rospy.get_param("~nav_cmd_topic", "/cmd_vel_nav")
        self.output_cmd_topic = rospy.get_param("~output_cmd_topic", "/cmd_vel")
        self.log_topic = rospy.get_param("~log_topic", "/rosout")
        self.warning_node = rospy.get_param("~warning_node", "/move_base")
        self.warning_text = rospy.get_param(
            "~warning_text",
            "TebLocalPlannerROS: trajectory is not feasible. Resetting planner...",
        )

        # trigger_duration is retained only for launch-file compatibility.
        # A single matching warning now triggers recovery immediately.
        self.trigger_duration = float(rospy.get_param("~trigger_duration", 0.0))
        self.forward_duration = float(rospy.get_param("~force_duration", 2.0))
        self.forward_speed = float(rospy.get_param("~forward_speed", 0.20))
        self.lateral_duration = float(rospy.get_param("~lateral_duration", 2.0))
        self.lateral_speed = float(rospy.get_param("~lateral_speed", 0.20))
        # Keep exactly-zero commands available for stopping, but raise every
        # non-zero planar command below this magnitude to a speed the A1 can
        # actually execute.  Angular velocity is deliberately not clamped.
        self.min_linear_speed = float(
            rospy.get_param("~min_linear_speed", 0.20)
        )
        self.linear_zero_epsilon = float(
            rospy.get_param("~linear_zero_epsilon", 1.0e-4)
        )
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))

        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.move_base_status_topic = rospy.get_param(
            "~move_base_status_topic", "/move_base/status"
        )
        self.stationary_duration = float(
            rospy.get_param("~stationary_duration", 2.0)
        )
        self.stationary_distance_epsilon = float(
            rospy.get_param("~stationary_distance_epsilon", 0.05)
        )
        self.stationary_yaw_epsilon = float(
            rospy.get_param("~stationary_yaw_epsilon", 0.05)
        )
        self.cmd_linear_threshold = float(
            rospy.get_param("~cmd_linear_threshold", 0.05)
        )
        self.cmd_angular_threshold = float(
            rospy.get_param("~cmd_angular_threshold", 0.10)
        )
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.75))
        self.no_path_event_hold = float(
            rospy.get_param("~no_path_event_hold", 3.0)
        )
        self.no_path_texts = (
            "NO PATH!",
            "Failed to get a plan",
            "Rotate recovery can't rotate in place",
        )

        self.local_costmap_topic = rospy.get_param(
            "~local_costmap_topic", "/move_base/local_costmap/costmap"
        )
        self.base_frame = rospy.get_param("~base_frame", "body")
        self.occupied_threshold = int(rospy.get_param("~occupied_threshold", 50))
        self.obstacle_forward_min = float(
            rospy.get_param("~obstacle_forward_min", -0.30)
        )
        self.obstacle_forward_max = float(
            rospy.get_param("~obstacle_forward_max", 0.90)
        )
        self.obstacle_lateral_min = float(
            rospy.get_param("~obstacle_lateral_min", 0.20)
        )
        self.obstacle_lateral_max = float(
            rospy.get_param("~obstacle_lateral_max", 1.20)
        )

        if self.forward_duration <= 0.0:
            raise ValueError("force_duration must be > 0")
        if self.lateral_duration <= 0.0:
            raise ValueError("lateral_duration must be > 0")
        if self.forward_speed <= 0.0:
            raise ValueError("forward_speed must be > 0")
        if self.lateral_speed <= 0.0:
            raise ValueError("lateral_speed must be > 0")
        if self.min_linear_speed <= 0.0:
            raise ValueError("min_linear_speed must be > 0")
        if not 0.0 <= self.linear_zero_epsilon < self.min_linear_speed:
            raise ValueError(
                "linear_zero_epsilon must be >= 0 and < min_linear_speed"
            )
        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be > 0")
        if self.stationary_duration <= 0.0:
            raise ValueError("stationary_duration must be > 0")
        if self.stationary_distance_epsilon <= 0.0:
            raise ValueError("stationary_distance_epsilon must be > 0")
        if self.stationary_yaw_epsilon <= 0.0:
            raise ValueError("stationary_yaw_epsilon must be > 0")
        if self.cmd_timeout <= 0.0:
            raise ValueError("cmd_timeout must be > 0")
        if self.no_path_event_hold <= 0.0:
            raise ValueError("no_path_event_hold must be > 0")
        if not 0 <= self.occupied_threshold <= 100:
            raise ValueError("occupied_threshold must be in [0, 100]")
        if self.obstacle_forward_min >= self.obstacle_forward_max:
            raise ValueError("obstacle forward bounds are invalid")
        if not 0.0 <= self.obstacle_lateral_min < self.obstacle_lateral_max:
            raise ValueError("obstacle lateral bounds are invalid")

        self._lock = threading.RLock()
        self._costmap = None
        self._active_action = None
        self._action_until = None
        self._lateral_sign = 1.0
        self._next_action = self.FORWARD
        self._fallback_lateral_sign = 1.0
        self._last_timer_time = None
        self._latest_nav_cmd = None
        self._latest_nav_cmd_time = None
        self._latest_pose = None
        self._move_base_active = False
        self._stationary_anchor = None
        self._stationary_since = None
        self._last_no_path_time = None
        self._pending_warning = False

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._cmd_pub = rospy.Publisher(
            self.output_cmd_topic, Twist, queue_size=20
        )
        self._nav_sub = rospy.Subscriber(
            self.nav_cmd_topic,
            Twist,
            self._nav_cmd_callback,
            queue_size=20,
            tcp_nodelay=True,
        )
        self._log_sub = rospy.Subscriber(
            self.log_topic, Log, self._log_callback, queue_size=200
        )
        self._costmap_sub = rospy.Subscriber(
            self.local_costmap_topic,
            OccupancyGrid,
            self._costmap_callback,
            queue_size=1,
        )
        self._odom_sub = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self._odom_callback,
            queue_size=10,
            tcp_nodelay=True,
        )
        self._status_sub = rospy.Subscriber(
            self.move_base_status_topic,
            GoalStatusArray,
            self._status_callback,
            queue_size=10,
        )
        self._timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.publish_rate), self._timer_callback
        )
        rospy.on_shutdown(self._on_shutdown)

        rospy.logwarn(
            "TEB two-stage recovery armed: one warning -> forward %.3fm/s "
            "for %.3fs; next recovery -> lateral %.3fm/s for %.3fs; "
            "stationary trigger=%.3fs with (ineffective cmd OR no-path); "
            "non-zero planar speed floor=%.3fm/s.",
            self.forward_speed,
            self.forward_duration,
            self.lateral_speed,
            self.lateral_duration,
            self.stationary_duration,
            self.min_linear_speed,
        )

    @staticmethod
    def _zero_twist():
        return Twist()

    def _apply_min_linear_speed(self, message):
        """Return a copied Twist with its non-zero planar speed clamped."""
        command = Twist()
        command.linear.x = message.linear.x
        command.linear.y = message.linear.y
        command.linear.z = message.linear.z
        command.angular.x = message.angular.x
        command.angular.y = message.angular.y
        command.angular.z = message.angular.z

        planar_speed = math.hypot(command.linear.x, command.linear.y)
        if (
            self.linear_zero_epsilon < planar_speed < self.min_linear_speed
        ):
            scale = self.min_linear_speed / planar_speed
            command.linear.x *= scale
            command.linear.y *= scale
            return command, planar_speed
        return command, None

    def _publish_command(self, message):
        command, original_speed = self._apply_min_linear_speed(message)
        if original_speed is not None:
            rospy.logwarn_throttle(
                1.0,
                "Raised planar cmd_vel from %.3f to %.3fm/s "
                "(x=%.3f, y=%.3f).",
                original_speed,
                self.min_linear_speed,
                command.linear.x,
                command.linear.y,
            )
        self._cmd_pub.publish(command)

    def _recovery_twist_locked(self):
        command = Twist()
        if self._active_action == self.FORWARD:
            command.linear.x = self.forward_speed
        elif self._active_action == self.LATERAL:
            command.linear.y = self._lateral_sign * self.lateral_speed
        return command

    def _costmap_callback(self, message):
        with self._lock:
            self._costmap = message

    def _nav_cmd_callback(self, message):
        now = rospy.Time.now()
        with self._lock:
            self._latest_nav_cmd = (
                float(message.linear.x),
                float(message.linear.y),
                float(message.angular.z),
            )
            self._latest_nav_cmd_time = now
            if self._active_action is not None:
                return
        self._publish_command(message)

    def _odom_callback(self, message):
        position = message.pose.pose.position
        yaw = self._yaw_from_quaternion(message.pose.pose.orientation)
        with self._lock:
            self._latest_pose = (
                float(position.x),
                float(position.y),
                float(yaw),
            )

    def _status_callback(self, message):
        active_states = (
            GoalStatus.PENDING,
            GoalStatus.ACTIVE,
            GoalStatus.PREEMPTING,
            GoalStatus.RECALLING,
        )
        active = any(
            status.status in active_states
            for status in message.status_list
        )
        with self._lock:
            if active != self._move_base_active:
                self._stationary_anchor = None
                self._stationary_since = None
            self._move_base_active = active

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        return euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )[2]

    def _obstacle_scores(self):
        with self._lock:
            grid = self._costmap

        if grid is None:
            raise RuntimeError("local costmap has not been received")
        if not grid.header.frame_id:
            raise RuntimeError("local costmap frame_id is empty")

        transform = self._tf_buffer.lookup_transform(
            self.base_frame,
            grid.header.frame_id,
            rospy.Time(0),
            rospy.Duration(0.20),
        )

        origin = grid.info.origin
        origin_yaw = self._yaw_from_quaternion(origin.orientation)
        cos_origin = math.cos(origin_yaw)
        sin_origin = math.sin(origin_yaw)

        tf_translation = transform.transform.translation
        tf_yaw = self._yaw_from_quaternion(transform.transform.rotation)
        cos_tf = math.cos(tf_yaw)
        sin_tf = math.sin(tf_yaw)

        width = int(grid.info.width)
        height = int(grid.info.height)
        resolution = float(grid.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            raise RuntimeError("local costmap metadata is invalid")
        if len(grid.data) != width * height:
            raise RuntimeError("local costmap data size is invalid")

        left_score = 0.0
        right_score = 0.0
        left_cells = 0
        right_cells = 0

        for row in range(height):
            grid_y = (row + 0.5) * resolution
            for col in range(width):
                value = int(grid.data[row * width + col])
                if value < self.occupied_threshold:
                    continue

                grid_x = (col + 0.5) * resolution
                source_x = (
                    origin.position.x
                    + cos_origin * grid_x
                    - sin_origin * grid_y
                )
                source_y = (
                    origin.position.y
                    + sin_origin * grid_x
                    + cos_origin * grid_y
                )

                body_x = (
                    tf_translation.x
                    + cos_tf * source_x
                    - sin_tf * source_y
                )
                body_y = (
                    tf_translation.y
                    + sin_tf * source_x
                    + cos_tf * source_y
                )

                if not self.obstacle_forward_min <= body_x <= self.obstacle_forward_max:
                    continue
                lateral_distance = abs(body_y)
                if not self.obstacle_lateral_min <= lateral_distance <= self.obstacle_lateral_max:
                    continue

                distance = max(0.10, math.hypot(body_x, body_y))
                weight = (1.0 + value / 100.0) / distance
                if body_y > 0.0:
                    left_score += weight
                    left_cells += 1
                elif body_y < 0.0:
                    right_score += weight
                    right_cells += 1

        return left_score, right_score, left_cells, right_cells

    def _choose_lateral_sign(self):
        try:
            left_score, right_score, left_cells, right_cells = self._obstacle_scores()
            if left_score > right_score + 1e-6:
                sign = -1.0
                reason = "left obstacle dominant; moving right"
            elif right_score > left_score + 1e-6:
                sign = 1.0
                reason = "right obstacle dominant; moving left"
            else:
                sign = self._fallback_lateral_sign
                self._fallback_lateral_sign *= -1.0
                reason = "left/right scores tied; using alternating fallback"

            rospy.logwarn(
                "TEB lateral decision: left_score=%.3f (%d cells), "
                "right_score=%.3f (%d cells): %s.",
                left_score,
                left_cells,
                right_score,
                right_cells,
                reason,
            )
            return sign
        except Exception as exc:
            sign = self._fallback_lateral_sign
            self._fallback_lateral_sign *= -1.0
            rospy.logerr(
                "Cannot evaluate obstacle side (%s); using alternating "
                "lateral fallback %s.",
                str(exc),
                "left" if sign > 0.0 else "right",
            )
            return sign

    @staticmethod
    def _angle_difference(first, second):
        return abs(math.atan2(
            math.sin(first - second),
            math.cos(first - second),
        ))

    def _stationary_long_enough_locked(self, now):
        if (
            not self._move_base_active
            or self._latest_pose is None
            or self._active_action is not None
        ):
            self._stationary_anchor = None
            self._stationary_since = None
            return False

        if self._stationary_anchor is None:
            self._stationary_anchor = self._latest_pose
            self._stationary_since = now
            return False

        distance = math.hypot(
            self._latest_pose[0] - self._stationary_anchor[0],
            self._latest_pose[1] - self._stationary_anchor[1],
        )
        yaw_change = self._angle_difference(
            self._latest_pose[2],
            self._stationary_anchor[2],
        )
        if (
            distance >= self.stationary_distance_epsilon
            or yaw_change >= self.stationary_yaw_epsilon
        ):
            self._stationary_anchor = self._latest_pose
            self._stationary_since = now
            return False

        return (
            self._stationary_since is not None
            and (now - self._stationary_since).to_sec()
            >= self.stationary_duration
        )

    def _nav_command_ineffective_locked(self, now):
        if (
            self._latest_nav_cmd is None
            or self._latest_nav_cmd_time is None
            or (now - self._latest_nav_cmd_time).to_sec() > self.cmd_timeout
        ):
            return True

        linear_x, linear_y, angular_z = self._latest_nav_cmd
        return (
            abs(linear_x) < self.cmd_linear_threshold
            and abs(linear_y) < self.cmd_linear_threshold
            and abs(angular_z) < self.cmd_angular_threshold
        )

    def _no_path_recent_locked(self, now):
        return (
            self._last_no_path_time is not None
            and 0.0 <= (now - self._last_no_path_time).to_sec()
            <= self.no_path_event_hold
        )

    def _start_next_recovery(self, now, reason):
        with self._lock:
            if self._active_action is not None:
                return False
            action = self._next_action

        lateral_sign = None
        if action == self.LATERAL:
            lateral_sign = self._choose_lateral_sign()

        with self._lock:
            if self._active_action is not None or action != self._next_action:
                return False

            self._active_action = action
            self._stationary_anchor = None
            self._stationary_since = None
            self._pending_warning = False
            self._last_no_path_time = None

            if action == self.FORWARD:
                self._action_until = now + rospy.Duration.from_sec(
                    self.forward_duration
                )
                self._next_action = self.LATERAL
            else:
                self._lateral_sign = lateral_sign
                self._action_until = now + rospy.Duration.from_sec(
                    self.lateral_duration
                )
                self._next_action = self.FORWARD

        if action == self.FORWARD:
            rospy.logerr(
                "Recovery triggered (%s): forcing linear.x=%.3fm/s "
                "for %.3fs.",
                reason,
                self.forward_speed,
                self.forward_duration,
            )
        else:
            rospy.logerr(
                "Recovery triggered (%s): forcing linear.y=%.3fm/s "
                "(%s) for %.3fs.",
                reason,
                lateral_sign * self.lateral_speed,
                "left" if lateral_sign > 0.0 else "right",
                self.lateral_duration,
            )
        return True

    def _log_callback(self, message):
        if message.level < Log.WARN:
            return
        if self.warning_node and message.name != self.warning_node:
            return

        is_teb_warning = self.warning_text in message.msg
        is_no_path = any(
            text in message.msg for text in self.no_path_texts
        )
        if not is_teb_warning and not is_no_path:
            return

        now = rospy.Time.now()
        if now.is_zero():
            return

        with self._lock:
            if is_no_path:
                self._last_no_path_time = now
            if is_teb_warning and self._active_action is not None:
                self._pending_warning = True
                rospy.logwarn_throttle(
                    1.0,
                    "TEB warning received during recovery; saved as pending.",
                )
                return

        if is_teb_warning:
            self._start_next_recovery(now, "TEB trajectory infeasible")

    def _reset_for_time_jump_locked(self, now):
        self._active_action = None
        self._action_until = None
        self._next_action = self.FORWARD
        self._stationary_anchor = None
        self._stationary_since = None
        self._last_no_path_time = None
        self._pending_warning = False
        self._last_timer_time = now
        rospy.logwarn("ROS time moved backwards; TEB recovery state reset.")

    def _timer_callback(self, _event):
        now = rospy.Time.now()
        if now.is_zero():
            return

        command = None
        finished_action = None
        trigger_reason = None
        with self._lock:
            if self._last_timer_time is not None and now < self._last_timer_time:
                self._reset_for_time_jump_locked(now)
                command = self._zero_twist()
            else:
                self._last_timer_time = now

            if self._active_action is not None:
                if now < self._action_until:
                    command = self._recovery_twist_locked()
                else:
                    finished_action = self._active_action
                    self._active_action = None
                    self._action_until = None
                    self._stationary_anchor = self._latest_pose
                    self._stationary_since = now
                    command = self._zero_twist()
            elif self._stationary_long_enough_locked(now):
                ineffective_cmd = self._nav_command_ineffective_locked(now)
                recent_no_path = self._no_path_recent_locked(now)
                pending_warning = self._pending_warning

                # Required logic:
                # ACTIVE goal AND stationary AND
                # (ineffective cmd OR recent no-path).  A TEB warning received
                # during a recovery is retained as an additional valid reason.
                if ineffective_cmd or recent_no_path or pending_warning:
                    reasons = []
                    if ineffective_cmd:
                        reasons.append("ineffective cmd_vel_nav")
                    if recent_no_path:
                        reasons.append("recent no-path")
                    if pending_warning:
                        reasons.append("pending TEB warning")
                    trigger_reason = " OR ".join(reasons)

        if command is not None:
            self._publish_command(command)
        if trigger_reason is not None:
            self._start_next_recovery(
                now,
                "ACTIVE + stationary %.2fs + (%s)"
                % (self.stationary_duration, trigger_reason),
            )
        if finished_action == self.FORWARD:
            rospy.logwarn(
                "Forced-forward interval finished; normal navigation resumed. "
                "The next valid recovery trigger will start lateral recovery."
            )
        elif finished_action == self.LATERAL:
            rospy.logwarn(
                "Forced-lateral interval finished; normal navigation resumed. "
                "The recovery sequence has reset to forward-first."
            )

    def _on_shutdown(self):
        try:
            self._publish_command(self._zero_twist())
        except Exception:
            pass


def main():
    rospy.init_node("teb_infeasible_forward_override")
    TebInfeasibleRecoveryOverride()
    rospy.spin()


if __name__ == "__main__":
    main()
