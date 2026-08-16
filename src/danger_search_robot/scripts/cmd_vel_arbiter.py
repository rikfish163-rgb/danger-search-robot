#!/usr/bin/env python3
"""Single-writer velocity arbiter for the fixed ROS1 runtime.

Navigation, short direct-entry/recovery motions, and the mission timeout
watchdog publish to separate input topics.  This node owns the only final
``/cmd_vel`` publisher and expires every input using wall time, so a paused
simulation cannot preserve an old non-zero command indefinitely.
"""

from __future__ import annotations

import threading
import time
from typing import Dict, Optional

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    from cmd_vel_arbiter_policy import choose_source
except ImportError:
    from danger_search_robot.scripts.cmd_vel_arbiter_policy import choose_source


class CmdVelArbiter:
    def __init__(self) -> None:
        self.output_topic = str(
            rospy.get_param("~output_topic", "/cmd_vel")
        )
        self.publish_rate = float(
            rospy.get_param("~publish_rate", 30.0)
        )
        self.default_timeout = float(
            rospy.get_param("~default_timeout", 0.75)
        )
        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        if self.default_timeout <= 0.0:
            raise ValueError("default_timeout must be positive")

        self.sources = self._load_sources()
        self._lock = threading.RLock()
        self._commands: Dict[str, Twist] = {}
        self._received_wall: Dict[str, float] = {}
        self._last_selected = "NONE"

        self.output_pub = rospy.Publisher(
            self.output_topic, Twist, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            "~status", String, queue_size=1, latch=True
        )
        for source in self.sources:
            rospy.Subscriber(
                source["topic"],
                Twist,
                self._callback,
                callback_args=source["name"],
                queue_size=1,
                tcp_nodelay=True,
            )

        self.timer = rospy.Timer(
            rospy.Duration.from_sec(1.0 / self.publish_rate),
            self._timer_callback,
        )
        rospy.on_shutdown(self._on_shutdown)
        self._publish_status("NONE")
        rospy.loginfo(
            "cmd_vel arbiter ready: output=%s sources=%s",
            self.output_topic,
            ",".join(
                "%s(priority=%d,timeout=%.2f)" % (
                    item["name"], item["priority"], item["timeout"]
                )
                for item in self.sources
            ),
        )

    def _load_sources(self):
        defaults = (
            ("safety", "/cmd_vel_safety", 100, 1.50),
            ("direct", "/cmd_vel_direct", 80, 0.75),
            ("exploration", "/cmd_vel_exploration", 60, 2.50),
            ("navigation", "/cmd_vel_nav", 10, 0.90),
        )
        sources = []
        for name, topic, priority, timeout in defaults:
            prefix = "~%s_" % name
            configured_topic = str(
                rospy.get_param(prefix + "topic", topic)
            )
            configured_priority = int(
                rospy.get_param(prefix + "priority", priority)
            )
            configured_timeout = float(
                rospy.get_param(
                    prefix + "timeout", timeout or self.default_timeout
                )
            )
            if not configured_topic:
                raise ValueError("%s topic must not be empty" % name)
            if configured_timeout <= 0.0:
                raise ValueError("%s timeout must be positive" % name)
            sources.append(
                {
                    "name": name,
                    "topic": configured_topic,
                    "priority": configured_priority,
                    "timeout": configured_timeout,
                }
            )
        return tuple(sources)

    def _callback(self, message: Twist, source: str) -> None:
        with self._lock:
            self._commands[source] = message
            self._received_wall[source] = time.monotonic()

    def _lease_snapshot(self, now: float):
        return {
            item["name"]: (
                item["priority"],
                self._received_wall.get(item["name"], float("nan")),
                item["timeout"],
            )
            for item in self.sources
        }

    def _timer_callback(self, _event) -> None:
        now = time.monotonic()
        with self._lock:
            leases = self._lease_snapshot(now)
            selected = choose_source(now, leases)
            command = (
                self._commands.get(selected, Twist())
                if selected is not None
                else Twist()
            )
        self.output_pub.publish(command)
        if selected is None:
            selected = "NONE"
        if selected != self._last_selected:
            self._publish_status(selected)

    def _publish_status(self, selected: str) -> None:
        self._last_selected = selected
        self.status_pub.publish(String(data=selected))

    def _on_shutdown(self) -> None:
        try:
            self.output_pub.publish(Twist())
        except Exception:
            pass


if __name__ == "__main__":
    try:
        rospy.init_node("cmd_vel_arbiter")
        CmdVelArbiter()
        rospy.spin()
    except (rospy.ROSInterruptException, ValueError) as error:
        rospy.logfatal("cmd_vel arbiter stopped: %s", error)
