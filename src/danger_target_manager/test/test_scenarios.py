"""Additional requirement-focused frame scenarios."""

import os
import sys
from typing import Optional, Sequence, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from danger_target_manager.core import Observation, TargetManagerConfig, TargetManagerCore

Point = Tuple[float, float, float]
P: Point = (1.0, 2.0, 0.5)


def manager(**kwargs: object) -> TargetManagerCore:
    """Create an isolated manager with no incidental frame-count timeout."""
    kwargs.setdefault("observation_timeout", 100.0)
    return TargetManagerCore(TargetManagerConfig(**kwargs))


def frame(
    core: TargetManagerCore, stamp: float, observations: Sequence[Observation]
) -> list:
    """Process one deterministic frame."""
    return core.process_frame(float(stamp), observations)


def obs(point: Optional[Point] = P, **kwargs: object) -> Observation:
    """Create one test observation without depending on another test module."""
    return Observation(point, **kwargs)

def test_same_stamp_multiple_observations_form_one_frame():
    core=manager(); events=[]
    for stamp in range(1,6): events += frame(core, stamp, [obs(P), obs((3., 0., .5))])
    assert len(events)==2 and {event.track_id for event in events}=={1,2}

def test_non_world_never_creates_candidate():
    core=manager()
    for stamp in range(1,6): assert not frame(core,stamp,[obs(P,frame_id="camera")])
    assert core.get_diagnostics()["active_count"]==0

def test_association_one_to_one():
    core=manager(association_radius=1.0)
    frame(core,1,[obs((0.,0.,0.)),obs((.8,0.,0.))])
    frame(core,2,[obs((.4,0.,0.))])
    tracks=core.get_diagnostics()["tracks"]
    assert sorted(track["consecutive_misses"] for track in tracks)==[0,1]
