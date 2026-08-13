#!/usr/bin/env python3
"""Low-speed closed-loop waypoint runner for the Gazebo truth profile.

This is only a deterministic fallback for the broken Livox Gazebo plugin.  It
still drives the real A1 controller through /cmd_vel and closes the loop on
/Odometry_gazebo; it does not change the robot pose directly.
"""

import math
import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class WaypointRunner:
    def __init__(self, waypoints):
        self.pose = None
        self.waypoints = waypoints
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/Odometry_gazebo', Odometry, self.on_odom, queue_size=1)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        self.pose = (p.x, p.y, yaw)

    def stop(self):
        self.pub.publish(Twist())

    def run(self):
        rate = rospy.Rate(20.0)
        for index, (gx, gy, gyaw, label) in enumerate(self.waypoints, 1):
            rospy.loginfo('WAYPOINT %d/%d %s target=(%.3f, %.3f, %.3f)',
                          index, len(self.waypoints), label, gx, gy, gyaw)
            stable = 0
            started = rospy.Time.now()
            while not rospy.is_shutdown():
                if self.pose is None:
                    rate.sleep()
                    continue
                x, y, yaw = self.pose
                distance = math.hypot(gx - x, gy - y)
                heading = math.atan2(gy - y, gx - x)
                heading_error = wrap(heading - yaw)
                final_error = wrap(gyaw - yaw)

                cmd = Twist()
                if distance > 0.22:
                    # The learned gait is more repeatable after an in-place
                    # turn than while receiving negative x commands.  Turn to
                    # the travel heading first, then walk forward.
                    if abs(heading_error) > 0.22:
                        cmd.angular.z = max(-0.75, min(0.75, 1.4 * heading_error))
                    else:
                        cmd.linear.x = min(0.24, 0.65 * distance)
                        cmd.angular.z = max(-0.55, min(0.55, 0.9 * heading_error))
                elif abs(final_error) > 0.10:
                    cmd.angular.z = max(-0.65, min(0.65, 1.2 * final_error))
                else:
                    stable += 1
                    self.stop()
                    if stable >= 12:
                        rospy.loginfo('WAYPOINT_SUCCEEDED %s pose=(%.3f, %.3f, %.3f)',
                                      label, x, y, yaw)
                        break
                self.pub.publish(cmd)
                if (rospy.Time.now() - started).to_sec() > 180.0:
                    self.stop()
                    raise RuntimeError('waypoint timeout: %s pose=(%.3f, %.3f, %.3f)' %
                                       (label, x, y, yaw))
                rate.sleep()
        self.stop()
        return 0


def parse_waypoints(raw):
    result = []
    for item in raw.split(';'):
        x, y, yaw, label = item.split(',', 3)
        result.append((float(x), float(y), float(yaw), label))
    return result


def main():
    if len(sys.argv) != 2:
        print('usage: ground_truth_waypoint_runner.py x,y,yaw,label;...')
        return 2
    rospy.init_node('ground_truth_waypoint_runner')
    runner = WaypointRunner(parse_waypoints(sys.argv[1]))
    try:
        return runner.run()
    except (RuntimeError, rospy.ROSInterruptException) as exc:
        rospy.logerr('%s', exc)
        runner.stop()
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
