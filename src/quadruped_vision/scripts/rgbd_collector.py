#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Synchronized RGB-D collector for dataset recording and labeling."""

from __future__ import annotations

import os
import time

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class RgbdCollector(object):
    def __init__(self):
        rospy.init_node("rgbd_data_collector", anonymous=True)
        self.bridge = CvBridge()

        configured_dir = str(rospy.get_param("~dataset_dir", "")).strip()
        base_dir = os.path.abspath(
            os.path.expanduser(
                configured_dir
                or os.path.join("~", "yolo_workspace", "dataset")
            )
        )
        self.save_dir_rgb = os.path.join(base_dir, "images", "raw_rgb")
        self.save_dir_depth = os.path.join(base_dir, "depth_data")
        os.makedirs(self.save_dir_rgb, exist_ok=True)
        os.makedirs(self.save_dir_depth, exist_ok=True)

        # 话题可通过私有参数覆盖。
        self.rgb_topic = rospy.get_param("~rgb_topic", "/real_sense/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/real_sense/depth/image_raw")

        sub_rgb = message_filters.Subscriber(self.rgb_topic, Image)
        sub_depth = message_filters.Subscriber(self.depth_topic, Image)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [sub_rgb, sub_depth], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

        self.latest_rgb = None
        self.latest_depth = None
        self.image_count = 0

        rospy.loginfo(
            "RGB-D collector ready | rgb=%s | depth=%s | dir=%s",
            self.rgb_topic,
            self.depth_topic,
            base_dir,
        )

    def sync_callback(self, rgb_msg, depth_msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr("Image convert failed: %s", e)

    def run(self):
        rospy.loginfo("Focus image window: press 's' to save, 'q' to quit.")
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.latest_rgb is not None:
                cv2.imshow("RGB-D Collector", self.latest_rgb)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("s"):
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    rgb_filename = "frame_{}_{}.jpg".format(timestamp, self.image_count)
                    rgb_path = os.path.join(self.save_dir_rgb, rgb_filename)
                    cv2.imwrite(rgb_path, self.latest_rgb)
                    depth_filename = "depth_{}_{}.npy".format(timestamp, self.image_count)
                    depth_path = os.path.join(self.save_dir_depth, depth_filename)
                    np.save(depth_path, self.latest_depth)
                    self.image_count += 1
                    rospy.loginfo("Saved pair #%d", self.image_count)
                elif key == ord("q"):
                    rospy.signal_shutdown("user quit")
                    break
            rate.sleep()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        RgbdCollector().run()
    except rospy.ROSInterruptException:
        pass
