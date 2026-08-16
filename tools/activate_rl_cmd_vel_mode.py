#!/usr/bin/env python3
"""Put junior_ctrl into the validated RL /cmd_vel state."""

import argparse
import time

import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Joy


BUTTON_STAND = 1
BUTTON_RL_CMD_VEL = 3
BUTTON_RESET = 10


def joy_message(button):
    message = Joy()
    message.header.stamp = rospy.Time.now()
    message.axes = [0.0] * 6
    message.buttons = [0] * 11
    message.buttons[button] = 1
    return message


def publish_for(publisher, button, seconds, rate):
    deadline = time.monotonic() + seconds
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        publisher.publish(joy_message(button))
        rate.sleep()


def publish_neutral(publisher, rate):
    message = joy_message(0)
    message.buttons = [0] * 11
    for _ in range(3):
        publisher.publish(message)
        rate.sleep()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stand-seconds", type=float, default=4.5)
    parser.add_argument("--rl-seconds", type=float, default=1.5)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument(
        "--reset",
        action="store_true",
        help="reset the Gazebo robot pose before standing and enabling RL",
    )
    parser.add_argument("--reset-seconds", type=float, default=0.5)
    parser.add_argument("--reset-settle-seconds", type=float, default=1.0)
    args = parser.parse_args()

    if (
        args.rate <= 0
        or args.stand_seconds <= 0
        or args.rl_seconds <= 0
        or args.reset_seconds <= 0
        or args.reset_settle_seconds < 0
    ):
        parser.error("durations and rate must be positive")

    rospy.init_node("activate_rl_cmd_vel_mode")
    publisher = rospy.Publisher("/joy", Joy, queue_size=1)
    deadline = time.monotonic() + args.timeout
    while not rospy.is_shutdown() and publisher.get_num_connections() == 0:
        if time.monotonic() >= deadline:
            raise RuntimeError("no /joy subscriber; junior_ctrl is not ready")
        time.sleep(0.1)

    rospy.wait_for_message("/clock", Clock, timeout=args.timeout)
    rate = rospy.Rate(args.rate)
    if args.reset:
        publish_for(publisher, BUTTON_RESET, args.reset_seconds, rate)
        if args.reset_settle_seconds:
            rospy.sleep(args.reset_settle_seconds)
    publish_for(publisher, BUTTON_STAND, args.stand_seconds, rate)
    publish_for(publisher, BUTTON_RL_CMD_VEL, args.rl_seconds, rate)
    publish_neutral(publisher, rate)
    rospy.loginfo(
        "RL /cmd_vel mode activated (reset=%s, stand button=%d, RL button=%d)",
        args.reset,
        BUTTON_STAND,
        BUTTON_RL_CMD_VEL,
    )


if __name__ == "__main__":
    main()
