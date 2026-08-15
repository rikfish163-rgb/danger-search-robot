#!/usr/bin/env python3
"""Pure policy helpers shared by the Graph-NBV runtime and its tests."""

import math
from typing import Iterable, Optional, Sequence, Tuple


Pose = Tuple[float, float, float]


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
