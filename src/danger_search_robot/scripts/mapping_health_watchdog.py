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
import threading
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
        # The map used for navigation is the map that must gate a mission.
        # ``/map_confirmed`` is intentionally kept as a secondary diagnostic
        # stream: it is sparse by design during early exploration and must not
        # make a healthy raw planning map look unhealthy (or vice versa).
        self.map_topic = rospy.get_param(
            "~map_topic", "/map_raw"
        )
        self.confirmed_map_topic = rospy.get_param(
            "~confirmed_map_topic", "/map_confirmed"
        )
        self.expected_map_frame = str(
            rospy.get_param("~expected_map_frame", "world")
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
            # Map/projection rates are expressed in simulation time.  A
            # loaded Gazebo can therefore leave a valid stamped map silent on
            # the wall clock for several seconds; stamp age remains the
            # authoritative freshness gate below.
            rospy.get_param("~max_wall_silence", 8.0)
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
        self._last_confirmed_map_wall: Optional[float] = None
        self._last_confirmed_map_stamp = rospy.Time(0)
        self._last_confirmed_map_geometry: Optional[Dict[str, object]] = None
        self._last_confirmed_map_observed = 0
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
        if self.confirmed_map_topic != self.map_topic:
            rospy.Subscriber(
                self.confirmed_map_topic,
                OccupancyGrid,
                self._confirmed_map_callback,
                queue_size=1,
            )
        rospy.Subscriber(
            self.status_topic,
            String,
            self._status_callback,
            queue_size=1,
        )
        # The mission controller measures this lease with wall time because
        # a slow Gazebo real-time factor must not make a live map look stale.
        # rospy.Timer follows /clock, so it can publish more slowly in wall
        # time exactly when Gazebo is under load.  Use a wall-clock heartbeat
        # for the health contract and keep ROS time only for sensor ages.
        self._stop_event = threading.Event()
        self._health_thread = threading.Thread(
            target=self._wall_publish_loop,
            name="mapping-health-heartbeat",
            daemon=True,
        )
        self._health_thread.start()
        rospy.on_shutdown(self._stop_health_thread)
        rospy.loginfo(
            "mapping health watchdog ready: cloud=%s planning_map=%s "
            "confirmed_map=%s expected_frame=%s status=%s",
            self.cloud_topic,
            self.map_topic,
            self.confirmed_map_topic,
            self.expected_map_frame,
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

    @staticmethod
    def _map_geometry(message: OccupancyGrid) -> Dict[str, object]:
        return {
            "width": int(message.info.width),
            "height": int(message.info.height),
            "resolution": float(message.info.resolution),
            "frame": str(message.header.frame_id),
            "origin_x": float(message.info.origin.position.x),
            "origin_y": float(message.info.origin.position.y),
        }

    @staticmethod
    def _observed_cells(message: OccupancyGrid) -> int:
        return sum(1 for value in message.data if value >= 0)

    def _map_callback(self, message: OccupancyGrid) -> None:
        self._last_map_wall = time.monotonic()
        self._last_map_stamp = message.header.stamp
        self._last_map_geometry = self._map_geometry(message)
        self._last_map_observed = self._observed_cells(message)

    def _confirmed_map_callback(self, message: OccupancyGrid) -> None:
        self._last_confirmed_map_wall = time.monotonic()
        self._last_confirmed_map_stamp = message.header.stamp
        self._last_confirmed_map_geometry = self._map_geometry(message)
        self._last_confirmed_map_observed = self._observed_cells(message)

    def _status_callback(self, message: String) -> None:
        self._last_status_wall = time.monotonic()
        self._last_status = str(message.data)

    def _publish_snapshot(self) -> None:
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

    def _wall_publish_loop(self) -> None:
        """Publish the lease at a wall-clock cadence independent of /clock."""

        while not rospy.is_shutdown() and not self._stop_event.is_set():
            self._publish_snapshot()
            self._stop_event.wait(self.publish_period)

    def _stop_health_thread(self) -> None:
        self._stop_event.set()

    def _map_snapshot(
        self,
        now: rospy.Time,
        now_wall: float,
        *,
        confirmed: bool = False,
        require_observations: bool = True,
    ) -> Dict[str, object]:
        if confirmed:
            last_wall = self._last_confirmed_map_wall
            stamp = self._last_confirmed_map_stamp
            geometry = self._last_confirmed_map_geometry
            observed = self._last_confirmed_map_observed
        else:
            last_wall = self._last_map_wall
            stamp = self._last_map_stamp
            geometry = self._last_map_geometry
            observed = self._last_map_observed

        age = self._stamp_age(stamp, now)
        reasons = []
        if last_wall is None:
            reasons.append("NO_MAP")
        elif now_wall - last_wall > self.max_wall_silence:
            reasons.append("MAP_SILENT")
        if age is None:
            reasons.append("ZERO_MAP_STAMP")
        elif age > self.max_map_age:
            reasons.append("STALE_MAP")
        elif age < -self.future_tolerance:
            reasons.append("FUTURE_MAP")

        if geometry is None:
            reasons.append("NO_MAP_GEOMETRY")
        else:
            width = int(geometry["width"])
            height = int(geometry["height"])
            resolution = float(geometry["resolution"])
            frame = str(geometry["frame"])
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
            if not frame:
                reasons.append("EMPTY_MAP_FRAME")
            elif self.expected_map_frame and frame != self.expected_map_frame:
                reasons.append("UNEXPECTED_MAP_FRAME")
        if require_observations and observed < self.min_observed_cells:
            reasons.append("INSUFFICIENT_OBSERVATIONS")

        return {
            "healthy": not reasons,
            "reasons": reasons,
            "age": round(age, 3) if age is not None else None,
            "observed_cells": observed,
            "geometry": geometry,
        }

    def _build_snapshot(self) -> Dict[str, object]:
        now = rospy.Time.now()
        now_wall = time.monotonic()
        cloud_age = self._stamp_age(self._last_cloud_stamp, now)
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

        planning_map = self._map_snapshot(now, now_wall)
        reasons.extend(planning_map["reasons"])
        confirmed_map = self._map_snapshot(
            now, now_wall, confirmed=True, require_observations=False
        )

        if self._last_status_wall is None:
            reasons.append("NO_PROJECTION_STATUS")
        elif now_wall - self._last_status_wall > self.max_wall_silence:
            reasons.append("STATUS_SILENT")
        if self._last_status != "RUNNING":
            reasons.append("PROJECTION_" + (self._last_status or "UNKNOWN"))

        snapshot = {
            "healthy": not reasons,
            "reasons": reasons,
            "planning_map_topic": self.map_topic,
            "confirmed_map_topic": self.confirmed_map_topic,
            "expected_map_frame": self.expected_map_frame,
            "cloud_age": round(cloud_age, 3) if cloud_age is not None else None,
            "map_age": planning_map["age"],
            "cloud_points": self._last_cloud_points,
            "map_observed_cells": planning_map["observed_cells"],
            "planning_map": planning_map,
            "confirmed_map": confirmed_map,
            "projection_status": self._last_status or "UNKNOWN",
            "map_geometry": planning_map["geometry"],
            "stamp": now.to_sec() if not now.is_zero() else 0.0,
        }
        return snapshot


if __name__ == "__main__":
    rospy.init_node("mapping_health_watchdog")
    try:
        MappingHealthWatchdog()
        rospy.spin()
    except (rospy.ROSInterruptException, ValueError) as error:
        rospy.logfatal("mapping health watchdog stopped: %s", error)
