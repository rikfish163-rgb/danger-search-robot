#!/usr/bin/env python3
import math
import sys

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


def main():
    if len(sys.argv) != 5:
        print("用法: send_world_goal.py x y yaw label")
        return 2

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3])
    label = sys.argv[4]

    rospy.init_node("send_world_goal", anonymous=True)
    client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

    print("等待 /move_base action server...")
    # Give rospy/TCPROS time to establish the action channels before the
    # simulated-clock timeout starts counting.
    rospy.sleep(2.0)
    if not client.wait_for_server(rospy.Duration(10.0)):
        print("错误：无法连接 /move_base")
        return 1

    q = quaternion_from_euler(0.0, 0.0, yaw)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "world"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    print(f"发送目标：{label}")
    print("  frame = world")
    print(f"  x     = {x:.3f}")
    print(f"  y     = {y:.3f}")
    print(f"  yaw   = {yaw:.4f} rad = {math.degrees(yaw):.2f}°")

    client.send_goal(goal)
    client.wait_for_result()

    state = client.get_state()
    names = {
        GoalStatus.PENDING: "PENDING",
        GoalStatus.ACTIVE: "ACTIVE",
        GoalStatus.PREEMPTED: "PREEMPTED",
        GoalStatus.SUCCEEDED: "SUCCEEDED",
        GoalStatus.ABORTED: "ABORTED",
        GoalStatus.REJECTED: "REJECTED",
        GoalStatus.PREEMPTING: "PREEMPTING",
        GoalStatus.RECALLING: "RECALLING",
        GoalStatus.RECALLED: "RECALLED",
        GoalStatus.LOST: "LOST",
    }

    print(f"导航结果：{names.get(state, str(state))}")
    return 0 if state == GoalStatus.SUCCEEDED else 1


if __name__ == "__main__":
    raise SystemExit(main())
