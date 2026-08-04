#!/usr/bin/env python3

import time
from typing import List

import rosnode
import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist


def find_graph_nbv_nodes(exact_name: str) -> List[str]:
    """查找当前运行的 Graph NBV 节点。"""
    try:
        nodes = rosnode.get_node_names()
    except Exception as exc:
        rospy.logwarn_throttle(
            5.0,
            "[nbv_watchdog] 无法读取节点列表：%s",
            exc,
        )
        return []

    own_name = rospy.get_name()

    if exact_name in nodes:
        return [exact_name]

    return [
        node
        for node in nodes
        if (
            "graph_nbv" in node.lower()
            and "watchdog" not in node.lower()
            and node != own_name
        )
    ]


def publish_stop(cancel_pub, cmd_vel_pub) -> None:
    """取消 move_base 目标并连续发布零速度。"""
    cancel_msg = GoalID()
    zero_cmd = Twist()

    for _ in range(8):
        cancel_pub.publish(cancel_msg)
        cmd_vel_pub.publish(zero_cmd)
        time.sleep(0.05)


def main() -> None:
    rospy.init_node("nbv_sim_time_watchdog")

    duration = float(
        rospy.get_param("~duration", 400.0)
    )
    target_node = str(
        rospy.get_param("~target_node", "/graph_nbv")
    )
    cancel_topic = str(
        rospy.get_param(
            "~cancel_topic",
            "/move_base/cancel",
        )
    )
    cmd_vel_topic = str(
        rospy.get_param("~cmd_vel_topic", "/cmd_vel")
    )

    if duration <= 0.0:
        rospy.logfatal(
            "[nbv_watchdog] duration 必须大于0。"
        )
        return

    if not rospy.get_param("/use_sim_time", False):
        rospy.logfatal(
            "[nbv_watchdog] /use_sim_time 不是 true，"
            "拒绝使用墙钟时间代替仿真时间。"
        )
        return

    cancel_pub = rospy.Publisher(
        cancel_topic,
        GoalID,
        queue_size=1,
    )
    cmd_vel_pub = rospy.Publisher(
        cmd_vel_topic,
        Twist,
        queue_size=1,
    )

    rospy.set_param(
        "/graph_nbv_timeout_reached",
        False,
    )

    rospy.loginfo(
        "[nbv_watchdog] 等待 Gazebo /clock..."
    )

    wait_rate = rospy.Rate(10)

    while (
        not rospy.is_shutdown()
        and rospy.Time.now().to_sec() <= 0.0
    ):
        wait_rate.sleep()

    if rospy.is_shutdown():
        return

    rospy.loginfo(
        "[nbv_watchdog] /clock 已就绪，等待 Graph NBV 节点..."
    )

    graph_nodes: List[str] = []

    while not rospy.is_shutdown():
        graph_nodes = find_graph_nbv_nodes(target_node)

        if graph_nodes:
            break

        wait_rate.sleep()

    if rospy.is_shutdown():
        return

    start_time = rospy.Time.now()
    last_reported_interval = -1

    rospy.logwarn(
        "[nbv_watchdog] Graph NBV 已启动：%s；"
        "开始计算 %.1f 秒仿真时间。",
        ", ".join(graph_nodes),
        duration,
    )

    loop_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        current_nodes = find_graph_nbv_nodes(target_node)

        if not current_nodes:
            rospy.loginfo(
                "[nbv_watchdog] Graph NBV 已自行结束，"
                "看门狗同步退出。"
            )
            return

        now = rospy.Time.now()
        elapsed = (now - start_time).to_sec()

        # 每30秒仿真时间输出一次剩余时间。
        report_interval = int(elapsed // 30.0)

        if report_interval != last_reported_interval:
            remaining = max(0.0, duration - elapsed)

            rospy.loginfo(
                "[nbv_watchdog] 已探索 %.1f 秒，"
                "剩余 %.1f 秒仿真时间。",
                elapsed,
                remaining,
            )

            last_reported_interval = report_interval

        if elapsed >= duration:
            rospy.logwarn(
                "[nbv_watchdog] 已达到 %.1f 秒仿真时间限制，"
                "取消导航目标并结束 Graph NBV。",
                duration,
            )

            rospy.set_param(
                "/graph_nbv_timeout_reached",
                True,
            )

            publish_stop(cancel_pub, cmd_vel_pub)

            targets = find_graph_nbv_nodes(target_node)

            if targets:
                try:
                    killed, failed = rosnode.kill_nodes(
                        targets
                    )

                    rospy.logwarn(
                        "[nbv_watchdog] 已关闭节点：%s",
                        killed,
                    )

                    if failed:
                        rospy.logerr(
                            "[nbv_watchdog] 关闭失败：%s",
                            failed,
                        )

                except Exception as exc:
                    rospy.logerr(
                        "[nbv_watchdog] 关闭 Graph NBV 失败：%s",
                        exc,
                    )

            publish_stop(cancel_pub, cmd_vel_pub)

            rospy.logwarn(
                "[nbv_watchdog] 本次 NBV 探索已超时结束。"
            )
            return

        loop_rate.sleep()


if __name__ == "__main__":
    main()
