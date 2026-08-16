#!/usr/bin/env python3
"""Pure policy helpers shared by the Graph-NBV runtime and its tests."""

import math
from typing import Iterable, Optional, Sequence, Tuple


Pose = Tuple[float, float, float]
Point2D = Tuple[float, float]


def point_is_blacklisted(
    point: Point2D,
    blacklist: Iterable[Point2D],
    radius: float,
) -> bool:
    """Return whether a point is inside the configured blacklist radius."""

    if not math.isfinite(radius) or radius < 0.0:
        raise ValueError("blacklist radius must be finite and non-negative")
    x, y = float(point[0]), float(point[1])
    return any(
        math.hypot(x - float(bx), y - float(by)) < radius
        for bx, by in blacklist
    )


def path_point_at_distance(
    points: Sequence[Point2D],
    max_distance: float,
) -> Optional[Point2D]:
    """Return a point no farther than ``max_distance`` along a path.

    A global frontier can be reachable according to ``make_plan`` while its
    full path is still too long for the local controller to execute reliably
    in one action.  This helper keeps the next action on the already checked
    path and is deliberately pure so it can be regression-tested without ROS.
    """

    if not math.isfinite(max_distance) or max_distance <= 0.0:
        raise ValueError("max_distance must be finite and positive")
    if len(points) < 2:
        return None

    remaining = float(max_distance)
    previous = (float(points[0][0]), float(points[0][1]))
    for point in points[1:]:
        current = (float(point[0]), float(point[1]))
        segment = math.hypot(
            current[0] - previous[0],
            current[1] - previous[1],
        )
        if not math.isfinite(segment):
            return None
        if segment <= 1e-9:
            previous = current
            continue
        if remaining <= segment:
            ratio = remaining / segment
            return (
                previous[0] + ratio * (current[0] - previous[0]),
                previous[1] + ratio * (current[1] - previous[1]),
            )
        remaining -= segment
        previous = current

    return previous


def pose_is_reasonable(
    pose: Optional[Pose],
    max_abs_position: float,
) -> bool:
    """Reject non-finite or numerically divergent planar TF poses."""

    if pose is None or len(pose) != 3:
        return False
    if max_abs_position <= 0.0 or not math.isfinite(max_abs_position):
        raise ValueError("max_abs_position must be finite and positive")
    if not all(math.isfinite(float(value)) for value in pose):
        return False
    return (
        abs(float(pose[0])) <= max_abs_position
        and abs(float(pose[1])) <= max_abs_position
    )


def freshness_age(
    stamped_at: float,
    received_at: float,
    now: float,
) -> Optional[float]:
    """Return freshness based on receipt time, falling back to header time.

    Sensor pipelines can legitimately deliver a message whose header stamp is
    older than the current clock because it spent time in a processing queue.
    Receipt time answers whether the pipeline is alive; the stamped age is
    still useful as a separate diagnostic metric.
    """

    if now <= 0.0 or not math.isfinite(now):
        return None
    for candidate in (received_at, stamped_at):
        if candidate > 0.0 and math.isfinite(candidate):
            return max(0.0, now - candidate)
    return None


def mapping_health_ready(
    received: bool,
    healthy: bool,
    age: Optional[float],
    timeout: float,
) -> bool:
    """Return whether the external mapping-health lease is still valid.

    The lease is deliberately fail-closed: a missing, unhealthy, stale, or
    non-finite watchdog update must pause exploration instead of allowing a
    stale occupancy grid to generate another navigation goal.
    """

    if not received or not healthy:
        return False
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("mapping health timeout must be finite and positive")
    if age is None or not math.isfinite(float(age)) or float(age) < 0.0:
        return False
    return float(age) <= timeout


def _angle_difference(first: float, second: float) -> float:
    return math.atan2(math.sin(first - second), math.cos(first - second))


def pose_is_close(
    first: Pose,
    second: Pose,
    position_tolerance: float,
    angle_tolerance: float,
) -> bool:
    """Return whether two planar poses are close enough for gate locking."""

    if position_tolerance < 0.0 or angle_tolerance < 0.0:
        raise ValueError("pose tolerances must be non-negative")

    distance = math.hypot(first[0] - second[0], first[1] - second[1])
    angle_error = abs(_angle_difference(first[2], second[2]))
    return distance <= position_tolerance and angle_error <= angle_tolerance


def pose_samples_stable(
    samples: Iterable[Pose],
    position_tolerance: float,
    angle_tolerance: float,
) -> bool:
    """Return whether every pose sample agrees with the first sample."""

    values: Sequence[Pose] = tuple(samples)
    if not values:
        return False
    return all(
        pose_is_close(values[0], pose, position_tolerance, angle_tolerance)
        for pose in values[1:]
    )


def gate_exhaustion_reached(empty_cycles: int, stable_cycles: int) -> bool:
    """Return whether the locked forward region has been stably exhausted."""

    return max(0, int(empty_cycles)) >= max(1, int(stable_cycles))


def failure_budget_exhausted(
    consecutive_failures: int,
    maximum_failures: int,
) -> bool:
    """Return whether repeated goal failures require a safe abort."""

    return max(0, int(consecutive_failures)) >= max(1, int(maximum_failures))


def path_cost_allowed(path_cost: float, maximum_path_cost: float) -> bool:
    """Return whether a local target is within the navigation path budget."""

    if not math.isfinite(maximum_path_cost) or maximum_path_cost <= 0.0:
        raise ValueError("maximum path cost must be finite and positive")
    return (
        math.isfinite(float(path_cost))
        and float(path_cost) >= 0.0
        and float(path_cost) <= maximum_path_cost
    )


def return_retry_available(
    retry_count: int,
    retry_limit: int,
) -> bool:
    """Return whether a failed return leg may be replanned."""

    return max(0, int(retry_count)) < max(0, int(retry_limit))
