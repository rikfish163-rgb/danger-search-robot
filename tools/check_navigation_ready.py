#!/usr/bin/env python3
"""Wait for the complete move_base runtime contract.

The move_base node registers before its action endpoints, TF lookup, and
costmaps are usable.  This probe deliberately checks the client-facing
action connection instead of treating node/service registration as ready.
"""

import argparse
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener


class NavigationReadiness:
    def __init__(self):
        self.received = {}
        self.costmaps = {}
        self.tf_buffer = Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = TransformListener(self.tf_buffer)
        rospy.Subscriber(
            "/move_base/status", GoalStatusArray, self._status_cb, queue_size=1
        )
        for topic in ("/move_base/global_costmap/costmap", "/move_base/local_costmap/costmap"):
            rospy.Subscriber(topic, OccupancyGrid, self._costmap_cb, topic, queue_size=1)
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

    def _status_cb(self, _msg):
        self.received["status"] = time.monotonic()

    def _costmap_cb(self, msg, topic):
        self.costmaps[topic] = (time.monotonic(), msg)

    def _tf_ready(self):
        try:
            return self.tf_buffer.can_transform(
                "world", "body", rospy.Time(0), rospy.Duration(0.2)
            )
        except Exception:
            return False

    def _costmap_ready(self, topic):
        item = self.costmaps.get(topic)
        if item is None:
            return False
        received_at, msg = item
        return (
            time.monotonic() - received_at < 2.0
            and msg.header.frame_id == "world"
            and msg.header.stamp.to_sec() > 0.0
            and msg.info.width > 0
            and msg.info.height > 0
            and len(msg.data) == msg.info.width * msg.info.height
        )

    def wait(self, timeout):
        deadline = time.monotonic() + timeout
        action_ready = False
        last_report = 0.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if not action_ready:
                remaining = max(0.1, min(1.0, deadline - time.monotonic()))
                try:
                    action_ready = self.client.wait_for_server(rospy.Duration(remaining))
                except Exception:
                    action_ready = False

            tf_ready = self._tf_ready()
            global_ready = self._costmap_ready("/move_base/global_costmap/costmap")
            local_ready = self._costmap_ready("/move_base/local_costmap/costmap")
            status_ready = "status" in self.received and time.monotonic() - self.received["status"] < 2.0
            if action_ready and tf_ready and global_ready and local_ready and status_ready:
                print(
                    "navigation readiness PASS: action_server=1 tf=world<-body "
                    "global_costmap=1 local_costmap=1 status=1",
                    flush=True,
                )
                return True

            now = time.monotonic()
            if now - last_report >= 3.0:
                print(
                    "navigation readiness waiting: action_server=%d tf=%d "
                    "global_costmap=%d local_costmap=%d status=%d"
                    % (
                        int(action_ready),
                        int(tf_ready),
                        int(global_ready),
                        int(local_ready),
                        int(status_ready),
                    ),
                    flush=True,
                )
                last_report = now
            time.sleep(0.1)

        print("navigation readiness FAILED before timeout", flush=True)
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    rospy.init_node("navigation_readiness_probe", anonymous=True)
    return 0 if NavigationReadiness().wait(args.timeout) else 1


if __name__ == "__main__":
    raise SystemExit(main())
