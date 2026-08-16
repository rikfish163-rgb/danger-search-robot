#!/usr/bin/env python3
"""Pure source-selection policy for the ROS1 velocity arbiter."""

import math
from typing import Mapping, Optional, Tuple


# source -> (priority, last_received_wall_time, lease_timeout_seconds)
SourceLease = Tuple[int, float, float]


def choose_source(
    now: float,
    leases: Mapping[str, SourceLease],
) -> Optional[str]:
    """Choose the highest-priority non-expired command source.

    A source lease is wall-clock based because a stopped Gazebo clock must not
    keep a stale velocity command alive.  Ties are resolved by the newest
    command so a source cannot lose merely because another source has the
    same configured priority.
    """

    if not math.isfinite(float(now)):
        raise ValueError("now must be finite")

    active = []
    for name, (priority, received_at, timeout) in leases.items():
        if not math.isfinite(float(received_at)):
            continue
        if not math.isfinite(float(timeout)) or timeout <= 0.0:
            raise ValueError("source lease timeout must be finite and positive")
        age = float(now) - float(received_at)
        if age < 0.0 or age > float(timeout):
            continue
        active.append((int(priority), float(received_at), str(name)))

    if not active:
        return None
    return max(active)[2]
