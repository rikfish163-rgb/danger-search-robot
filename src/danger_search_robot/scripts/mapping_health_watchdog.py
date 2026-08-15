#!/usr/bin/env python3
"""Fail-closed health gate for the simulated point-cloud mapping chain.

The mapping node can be alive while its input cloud is stale, its TF lookup is
waiting, or its latched OccupancyGrid is no longer being refreshed. This node
turns those conditions into a small, machine-readable contract for mission
launchers and exploration controllers. It never clears a map or changes the
active floor; recovery remains an explicit operator/startup action.
"""

from __future__ import annotations

import json
import math
import time
from typing import Dict, Optional

import rospy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String


class MappingHealthWatchdog:
    """Publish a fail-closed health snapshot for pointcloud -> 2D map."""

    def __init__(self) -> None:
        self.cloud_topic = rospy.get_param(
            "~cloud_topic", "/livox/Pointcloud2"
        )
        self.map_topic = rospy.get_param(
            "~map_topic", "/map_confirmed"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/fastlio_2d_projection/status"
        )
        self.max_cloud_age = float(
            rospy.get_param("~max_cloud_age", 0.75)
        )
        self.max_map_age = float(
            rospy.get_param("~max_map_age", 1.25)
        )
        self.max_wall_silence = float(
            rospy.get_param("~max_wall_silence", 2.5)
        )
        self.future_tolerance = float(
            rospy.get_param("~future_tolerance", 0.25)
        )
        self.min_observed_cells = int(
            rospy.get_param("~min_observed_cells", 1)
        )
        self.publish_period = float(
            rospy.get_param("~publish_period", 0.5)
        )
        self.expected_width = int(
            rospy.get_param("~expected_width", 0)
        )
        self.expected_height = int(
            rospy.get_param("~expected_height", 0)
        )
        self.expected_resolution = float(
            rospy.get_param("~expected_resolution", 0.0)
        )
        self._validate_parameters()

        self._last_cloud_wall: Optional[float] = None
        self._last_cloud_stamp = rospy.Time(0)
        self._last_cloud_points = 0
        self._last_map_wall: Optional[float] = None
        self._last_map_stamp = rospy.Time(0)
        self._last_map_geometry: Optional[Dict[str, object]] = None
        self._last_map_observed = 0
        self._last_status_wall: Optional[float] = None
        self._last_status = ""

        self.health_pub = rospy.Publisher(
            "~health", String, queue_size=1, latch=True
        )
        self.healthy_pub = rospy.Publisher(
            "~healthy", Bool, queue_size=1, latch=True
        )
        # Stable global aliases for shell preflight and external runners.
        self.global_health_pub = rospy.Publisher(
            "/mapping_health", String, queue_size=1, latch=True
        )
        self.global_healthy_pub = rospy.Publisher(
            "/mapping_healthy", Bool, queue_size=1, latch=True
        )

        rospy.Subscriber(
            self.cloud_topic,
            PointCloud2,
            self._cloud_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.map_topic,
            OccupancyGrid,
            self._map_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.status_topic,
            String,
            self._status_callback,
            queue_size=1,
        )
        self.timer = rospy.Timer(
            rospy.Duration(self.publish_period),
            self._timer_callback,
        )
        rospy.loginfo(
            "mapping health watchdog ready: cloud=%s map=%s status=%s",
            self.cloud_topic,
            self.map_topic,
            self.status_topic,
        )

    def _validate_parameters(self) -> None:
        values = (
            self.max_cloud_age,
            self.max_map_age,
            self.max_wall_silence,
            self.future_tolerance,
            self.publish_period,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("health timing parameters must be finite and > 0")
        if self.min_observed_cells < 0:
            raise ValueError("min_observed_cells must be >= 0")
        if self.expected_width < 0 or self.expected_height < 0:
            raise ValueError("expected map dimensions must be >= 0")
        if not math.isfinite(self.expected_resolution) or self.expected_resolution < 0.0:
            raise ValueError("expected_resolution must be finite and >= 0")

    @staticmethod
    def _stamp_age(stamp: rospy.Time, now: rospy.Time) -> Optional[float]:
        if stamp.is_zero() or now.is_zero():
            return None
        age = (now - stamp).to_sec()
        return age if math.isfinite(age) else None

    def _cloud_callback(self, message: PointCloud2) -> None:
        self._last_cloud_wall = time.monotonic()
        self._last_cloud_stamp = message.header.stamp
        self._last_cloud_points = int(message.width * message.height)

    def _map_callback(self, message: OccupancyGrid) -> None:
        self._last_map_wall = time.monotonic()
        self._last_map_stamp = message.header.stamp
        self._last_map_geometry = {
            "width": int(message.info.width),
            "height": int(message.info.height),
            "resolution": float(message.info.resolution),
            "frame": str(message.header.frame_id),
            "origin_x": float(message.info.origin.position.x),
            "origin_y": float(message.info.origin.position.y),
        }
        self._last_map_observed = sum(
            1 for value in message.data if value >= 0
        )

    def _status_callback(self, message: String) -> None:
        self._last_status_wall = time.monotonic()
        self._last_status = str(message.data)

    def _timer_callback(self, _event: rospy.timer.TimerEvent) -> None:
        snapshot = self._build_snapshot()
        encoded = json.dumps(
            snapshot, sort_keys=True, separators=(",", ":")
        )
        self.health_pub.publish(String(data=encoded))
        self.global_health_pub.publish(String(data=encoded))
        self.healthy_pub.publish(Bool(data=bool(snapshot["healthy"])))
        self.global_healthy_pub.publish(Bool(data=bool(snapshot["healthy"])))
        if snapshot["healthy"]:
            rospy.loginfo_throttle(
                10.0,
                "mapping health PASS: cloud_age=%.3f map_age=%.3f "
                "status=%s observed=%d",
                float(snapshot["cloud_age"]),
                float(snapshot["map_age"]),
                self._last_status,
                self._last_map_observed,
            )
        else:
            rospy.logwarn_throttle(
                5.0,
                "mapping health FAIL: %s",
                ",".join(snapshot["reasons"]),
            )

    def _build_snapshot(self) -> Dict[str, object]:
        now = rospy.Time.now()
        now_wall = time.monotonic()
        cloud_age = self._stamp_age(self._last_cloud_stamp, now)
        map_age = self._stamp_age(self._last_map_stamp, now)
        reasons = []

        if self._last_cloud_wall is None:
            reasons.append("NO_CLOUD")
        elif now_wall - self._last_cloud_wall > self.max_wall_silence:
            reasons.append("CLOUD_SILENT")
        if cloud_age is None:
            reasons.append("ZERO_CLOUD_STAMP")
        elif cloud_age > self.max_cloud_age:
            reasons.append("STALE_CLOUD")
        elif cloud_age < -self.future_tolerance:
            reasons.append("FUTURE_CLOUD")
        if self._last_cloud_points <= 0:
            reasons.append("EMPTY_CLOUD")

        if self._last_map_wall is None:
            reasons.append("NO_MAP")
        elif now_wall - self._last_map_wall > self.max_wall_silence:
            reasons.append("MAP_SILENT")
        if map_age is None:
            reasons.append("ZERO_MAP_STAMP")
        elif map_age > self.max_map_age:
            reasons.append("STALE_MAP")
        elif map_age < -self.future_tolerance:
            reasons.append("FUTURE_MAP")

        geometry = self._last_map_geometry
        if geometry is None:
            reasons.append("NO_MAP_GEOMETRY")
        else:
            width = int(geometry["width"])
            height = int(geometry["height"])
            resolution = float(geometry["resolution"])
            if width <= 0 or height <= 0 or resolution <= 0.0:
                reasons.append("INVALID_GEOMETRY")
            if self.expected_width and width != self.expected_width:
                reasons.append("UNEXPECTED_WIDTH")
            if self.expected_height and height != self.expected_height:
                reasons.append("UNEXPECTED_HEIGHT")
            if self.expected_resolution and abs(
                resolution - self.expected_resolution
            ) > 1e-6:
                reasons.append("UNEXPECTED_RESOLUTION")
            if not geometry["frame"]:
                reasons.append("EMPTY_MAP_FRAME")
        if self._last_map_observed < self.min_observed_cells:
            reasons.append("INSUFFICIENT_OBSERVATIONS")

        if self._last_status_wall is None:
            reasons.append("NO_PROJECTION_STATUS")
        elif now_wall - self._last_status_wall > self.max_wall_silence:
            reasons.append("STATUS_SILENT")
        if self._last_status != "RUNNING":
            reasons.append("PROJECTION_" + (self._last_status or "UNKNOWN"))

        return {
            "healthy": not reasons,
            "reasons": reasons,
            "cloud_age": round(cloud_age, 3) if cloud_age is not None else None,
            "map_age": round(map_age, 3) if map_age is not None else None,
            "cloud_points": self._last_cloud_points,
            "map_observed_cells": self._last_map_observed,
            "projection_status": self._last_status or "UNKNOWN",
            "map_geometry": geometry,
            "stamp": now.to_sec() if not now.is_zero() else 0.0,
        }


if __name__ == "__main__":
    rospy.init_node("mapping_health_watchdog")
    try:
        MappingHealthWatchdog()
        rospy.spin()
    except (rospy.ROSInterruptException, ValueError) as error:
        rospy.logfatal("mapping health watchdog stopped: %s", error)
