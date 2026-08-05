"""Unit tests for deterministic target manager core."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from danger_target_manager.core import Observation, TargetManagerConfig, TargetManagerCore

P = (1.0, 2.0, 0.5)
def manager(**kwargs):
    """Use a long timeout for frame-count tests; timeout has its own test."""
    kwargs.setdefault("observation_timeout", 100.0)
    return TargetManagerCore(TargetManagerConfig(**kwargs))
def frame(core, stamp, observations): return core.process_frame(float(stamp), observations)
def obs(point=P, **kwargs): return Observation(point, **kwargs)

def test_single_confirms_once_and_ids_are_unique():
    core = manager(); events = []
    for stamp in range(1, 7): events += frame(core, stamp, [obs()])
    assert len(events) == 1 and events[0].track_id == 1
    for stamp in range(7, 12): events += frame(core, stamp, [obs()])
    assert len(events) == 1

def test_dropout_and_outlier_are_robust():
    core = manager(); events = []
    for stamp, point in enumerate([P, None, P, (8, 8, 8), P, P, P], 1): events += frame(core, stamp, [obs(point, valid=point is not None)])
    assert len(events) == 1
    assert max(abs(a-b) for a,b in zip(events[0].position, P)) < 1e-6

def test_insufficient_and_miss_expiry():
    core = manager(); events = []
    for stamp, point in enumerate([P, None, P, None, P, None, P], 1): events += frame(core, stamp, [obs(point, valid=point is not None)])
    assert not events
    for stamp in range(8, 11): frame(core, stamp, [obs(None, valid=False)])
    assert core.get_diagnostics()["active_count"] == 0

def test_multi_one_to_one_history_and_new_target():
    core = manager(); a, b = P, (3., -1., .5); events=[]
    for stamp in range(1, 6): events += frame(core, stamp, [obs(a), obs(b)])
    assert [event.track_id for event in events] == [1, 2]
    for stamp in range(6, 11): events += frame(core, stamp, [obs(a), obs((5., 5., .5))])
    assert [event.track_id for event in events] == [1, 2, 3]

def test_timeout_invalid_frame_and_reset_determinism():
    core = manager(observation_timeout=1.0); frame(core, 1, [obs(P)]); core.expire(2.0); assert core.get_diagnostics()["active_count"] == 0
    assert not frame(core, 3, [obs((float("nan"), 0, 0))]); assert not frame(core, 2, [obs(P)])
    core.reset(); assert core.get_diagnostics()["active_count"] == 0
    def run():
        c=manager(); return [e.track_id for t in range(1,6) for e in frame(c,t,[obs(P)])]
    assert run() == run() == [1]
