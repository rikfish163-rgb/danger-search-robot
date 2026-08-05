#!/usr/bin/env python3
"""Keyboard teleop for junior_ctrl through sensor_msgs/Joy."""

import sys
import termios
import tty

import rospy
from sensor_msgs.msg import Joy


BUTTON_PASSIVE = 0
BUTTON_STAND = 1
BUTTON_RL_KEYBOARD = 2
BUTTON_RL = 3
BUTTON_FREE_STAND = 6
BUTTON_BALANCE_TEST = 7
BUTTON_SWING_TEST = 8
BUTTON_STEP_TEST = 9
BUTTON_RESET = 10


def clamp(value, minimum=-1.0, maximum=1.0):
    return max(minimum, min(maximum, value))


class KeyboardTeleop:
    def __init__(self):
        self.axes = [0.0] * 6
        self.buttons = [0] * 11
        self.linear_step = rospy.get_param("~linear_step", 0.05)
        self.angular_step = rospy.get_param("~angular_step", 0.05)
        self.repeat_rate = rospy.get_param("~repeat_rate", 20.0)
        self.pub = rospy.Publisher("/joy", Joy, queue_size=1)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        timer = rospy.Timer(rospy.Duration(1.0 / self.repeat_rate), self.publish)
        try:
            print("Keyboard teleop: 1 passive, 2 stand, 4 RL keyboard walk, 6 RL /cmd_vel, 8 reset, WASD move in mode 4, JL turn, Space stop, q quit")
            while not rospy.is_shutdown():
                key = sys.stdin.read(1)
                if key in ("q", "\x03"):
                    break
                self.handle_key(key)
                self.publish(None)
        finally:
            timer.shutdown()
            self.axes = [0.0] * 6
            self.buttons = [0] * 11
            self.publish(None)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def handle_key(self, key):
        self.buttons = [0] * 11
        if key == "1":
            self.buttons[BUTTON_PASSIVE] = 1
        elif key == "2":
            self.buttons[BUTTON_STAND] = 1
        elif key == "3":
            self.buttons[BUTTON_FREE_STAND] = 1
        elif key == "4":
            self.buttons[BUTTON_RL_KEYBOARD] = 1
        elif key == "6":
            self.buttons[BUTTON_RL] = 1
        elif key == "7":
            self.buttons[BUTTON_BALANCE_TEST] = 1
        elif key == "8":
            self.buttons[BUTTON_RESET] = 1
        elif key == "9":
            self.buttons[BUTTON_SWING_TEST] = 1
        elif key in (" ", "\n"):
            self.axes = [0.0] * 6
        elif key in ("w", "W"):
            self.axes[1] = clamp(self.axes[1] + self.linear_step)
        elif key in ("s", "S"):
            self.axes[1] = clamp(self.axes[1] - self.linear_step)
        elif key in ("d", "D"):
            self.axes[0] = clamp(self.axes[0] + self.linear_step)
        elif key in ("a", "A"):
            self.axes[0] = clamp(self.axes[0] - self.linear_step)
        elif key in ("l", "L"):
            self.axes[3] = clamp(self.axes[3] + self.angular_step)
        elif key in ("j", "J"):
            self.axes[3] = clamp(self.axes[3] - self.angular_step)
        elif key in ("i", "I"):
            self.axes[4] = clamp(self.axes[4] + self.angular_step)
        elif key in ("k", "K"):
            self.axes[4] = clamp(self.axes[4] - self.angular_step)

    def publish(self, _event):
        msg = Joy()
        msg.header.stamp = rospy.Time.now()
        msg.axes = self.axes
        msg.buttons = self.buttons
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("keyboard_teleop")
    KeyboardTeleop().run()
