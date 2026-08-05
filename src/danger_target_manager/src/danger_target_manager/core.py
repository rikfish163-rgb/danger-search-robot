"""Deterministic, ROS-independent multi-target confirmation state machine."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

Point3 = Tuple[float, float, float]


class TrackState(str, Enum):
    """Lifecycle state of a target track."""

    DETECTED = "DETECTED"
    CONFIRMED = "CONFIRMED"
    REPORTED = "REPORTED"


@dataclass(frozen=True)
class Observation:
    """One upstream observation, represented without ROS message dependencies."""

    position: Optional[Point3]
    valid: bool = True
    frame_id: str = "world"
    detector_confidence: Optional[float] = None


@dataclass(frozen=True)
class WindowEntry:
    """One candidate result for a processed frame."""

    stamp: float
    valid: bool
    position: Optional[Point3]
    detector_confidence: Optional[float]


@dataclass(frozen=True)
class ConfirmedTarget:
    """A single publication event produced exactly once per real target."""

    track_id: int
    position: Point3
    confidence: float
    stamp: float


@dataclass
class TargetManagerConfig:
    """All association and confirmation thresholds used by TargetManagerCore."""

    expected_frame: str = "world"
    window_size: int = 7
    min_valid_count: int = 5
    candidate_radius: float = 0.20
    association_radius: float = 0.40
    history_dedup_radius: float = 0.40
    max_consecutive_misses: int = 3
    observation_timeout: float = 1.0
    update_reported_position: bool = True
    reported_position_alpha: float = 0.10

    def validate(self) -> None:
        """Raise ValueError if the configuration would make state transitions ambiguous."""
        if self.window_size < 1:
            raise ValueError("window_size must be >= 1")
        if not 1 <= self.min_valid_count <= self.window_size:
            raise ValueError("min_valid_count must be in [1, window_size]")
        for name in ("candidate_radius", "association_radius", "history_dedup_radius"):
            if getattr(self, name) <= 0:
                raise ValueError("%s must be > 0" % name)
        if self.observation_timeout <= 0:
            raise ValueError("observation_timeout must be > 0")
        if self.max_consecutive_misses < 1:
            raise ValueError("max_consecutive_misses must be >= 1")
        if not 0.0 <= self.reported_position_alpha <= 1.0:
            raise ValueError("reported_position_alpha must be in [0, 1]")


@dataclass
class _Candidate:
    track_id: int
    state: TrackState
    window: Deque[WindowEntry]
    last_valid_stamp: float
    last_frame_stamp: float
    consecutive_misses: int
    created_stamp: float
    confirmation_stamp: Optional[float] = None
    confirmed_position: Optional[Point3] = None
    reported: bool = False
    last_association: str = "created"

    def reference_position(self) -> Point3:
        """Use the latest valid observation as deterministic association reference."""
        for entry in reversed(self.window):
            if entry.valid and entry.position is not None:
                return entry.position
        raise RuntimeError("candidate without a valid position")


@dataclass
class _HistoryTarget:
    track_id: int
    position: Point3
    confirmation_stamp: float


def _finite_point(point: Optional[Point3]) -> bool:
    return point is not None and len(point) == 3 and all(math.isfinite(float(v)) for v in point)


def _distance(a: Point3, b: Point3) -> float:
    return float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))


def _clamp(value: Optional[float]) -> Optional[float]:
    if value is None or not math.isfinite(float(value)):
        return None
    return max(0.0, min(1.0, float(value)))


class TargetManagerCore:
    """Frame-oriented candidate association, confirmation, expiry, and deduplication."""

    def __init__(self, config: TargetManagerConfig) -> None:
        config.validate()
        self.config = config
        self.reset()

    def reset(self) -> None:
        """Clear all task-period state and restart IDs at one."""
        self._active: Dict[int, _Candidate] = {}
        self._history: List[_HistoryTarget] = []
        self._next_track_id = 1
        self._last_processed_stamp: Optional[float] = None
        self._events: List[str] = ["reset"]
        self._rejected_frames = 0
        self._rejected_observations = 0

    def process_frame(self, frame_stamp: float, observations: Sequence[Observation]) -> List[ConfirmedTarget]:
        """Process exactly one completed frame and return newly confirmed targets.

        A frame can contain zero valid spatial observations.  Invalid observations
        advance existing tracks as misses but never create a candidate.
        """
        if not math.isfinite(frame_stamp) or frame_stamp <= 0.0:
            self._rejected_frames += 1
            self._events.append("rejected invalid frame stamp")
            return []
        if self._last_processed_stamp is not None and frame_stamp < self._last_processed_stamp:
            self._rejected_frames += 1
            self._events.append("rejected time reversal %.9f" % frame_stamp)
            return []
        if self._last_processed_stamp is not None and frame_stamp == self._last_processed_stamp:
            self._rejected_frames += 1
            self._events.append("rejected duplicate completed frame %.9f" % frame_stamp)
            return []
        self._last_processed_stamp = frame_stamp
        valid: List[Observation] = []
        for observation in observations:
            if observation.valid and observation.frame_id == self.config.expected_frame and _finite_point(observation.position):
                valid.append(observation)
            else:
                self._rejected_observations += 1
        candidate_ids = sorted(self._active)
        pairs: List[Tuple[float, int, int]] = []
        for track_id in candidate_ids:
            reference = self._active[track_id].reference_position()
            for index, observation in enumerate(valid):
                assert observation.position is not None
                distance = _distance(reference, observation.position)
                if distance <= self.config.association_radius:
                    pairs.append((distance, track_id, index))
        pairs.sort(key=lambda pair: (pair[0], pair[1], pair[2]))
        used_tracks, used_observations = set(), set()
        matches: Dict[int, int] = {}
        for _, track_id, index in pairs:
            if track_id not in used_tracks and index not in used_observations:
                used_tracks.add(track_id)
                used_observations.add(index)
                matches[track_id] = index
        for track_id in candidate_ids:
            candidate = self._active[track_id]
            candidate.last_frame_stamp = frame_stamp
            if track_id in matches:
                observation = valid[matches[track_id]]
                candidate.window.append(WindowEntry(frame_stamp, True, observation.position, _clamp(observation.detector_confidence)))
                candidate.last_valid_stamp = frame_stamp
                candidate.consecutive_misses = 0
                candidate.last_association = "matched observation %d" % matches[track_id]
                self._events.append("associated track %d" % track_id)
            else:
                candidate.window.append(WindowEntry(frame_stamp, False, None, None))
                candidate.consecutive_misses += 1
                candidate.last_association = "miss"
                self._events.append("unmatched track %d" % track_id)
        for index, observation in enumerate(valid):
            if index in used_observations:
                continue
            assert observation.position is not None
            history = self._nearest_history(observation.position)
            if history is not None:
                self._events.append("history dedup track %d" % history.track_id)
                if self.config.update_reported_position:
                    alpha = self.config.reported_position_alpha
                    history.position = tuple((1.0 - alpha) * old + alpha * new for old, new in zip(history.position, observation.position))  # type: ignore[assignment]
                continue
            self._create_candidate(frame_stamp, observation)
        confirmed: List[ConfirmedTarget] = []
        for track_id in sorted(list(self._active)):
            candidate = self._active.get(track_id)
            if candidate is not None:
                event = self._confirm_if_ready(candidate, frame_stamp)
                if event is not None:
                    confirmed.append(event)
        self.expire(frame_stamp)
        return confirmed

    def expire(self, logical_time: float) -> None:
        """Remove only unconfirmed candidates that exceeded miss or time limits."""
        if not math.isfinite(logical_time):
            return
        for track_id, candidate in list(self._active.items()):
            timeout = logical_time - candidate.last_valid_stamp >= self.config.observation_timeout
            misses = candidate.consecutive_misses >= self.config.max_consecutive_misses
            if timeout or misses:
                reason = "timeout" if timeout else "miss limit"
                del self._active[track_id]
                self._events.append("deleted track %d: %s" % (track_id, reason))

    def _create_candidate(self, stamp: float, observation: Observation) -> None:
        assert observation.position is not None
        track_id = self._next_track_id
        self._next_track_id += 1
        entry = WindowEntry(stamp, True, observation.position, _clamp(observation.detector_confidence))
        self._active[track_id] = _Candidate(track_id, TrackState.DETECTED, deque([entry], maxlen=self.config.window_size), stamp, stamp, 0, stamp)
        self._events.append("created track %d" % track_id)

    def _nearest_history(self, position: Point3) -> Optional[_HistoryTarget]:
        pairs = [( _distance(position, item.position), item.track_id, item) for item in self._history]
        eligible = [pair for pair in pairs if pair[0] <= self.config.history_dedup_radius]
        return min(eligible, key=lambda pair: (pair[0], pair[1]))[2] if eligible else None

    def _confirm_if_ready(self, candidate: _Candidate, stamp: float) -> Optional[ConfirmedTarget]:
        entries = [entry for entry in candidate.window if entry.valid and entry.position is not None]
        if len(entries) < self.config.min_valid_count:
            return None
        values = np.asarray([entry.position for entry in entries], dtype=float)
        median = np.median(values, axis=0)
        distances = np.linalg.norm(values - median, axis=1)
        inlier_indexes = [index for index, distance in enumerate(distances) if distance <= self.config.candidate_radius]
        if len(inlier_indexes) < self.config.min_valid_count:
            return None
        inlier_values = values[inlier_indexes]
        position = tuple(float(value) for value in np.median(inlier_values, axis=0))
        confidences = [entries[index].detector_confidence for index in inlier_indexes]
        known = [confidence for confidence in confidences if confidence is not None]
        ratio = float(len(inlier_indexes)) / float(self.config.window_size)
        confidence = ratio * (float(np.mean(known)) if known else 1.0)
        event = ConfirmedTarget(candidate.track_id, position, max(0.0, min(1.0, confidence)), stamp)
        candidate.state = TrackState.CONFIRMED
        candidate.confirmation_stamp = stamp
        candidate.confirmed_position = position
        candidate.reported = True
        candidate.state = TrackState.REPORTED
        del self._active[candidate.track_id]
        self._history.append(_HistoryTarget(candidate.track_id, position, stamp))
        self._events.append("confirmed track %d" % candidate.track_id)
        return event

    def get_diagnostics(self) -> Dict[str, object]:
        """Return serializable state suitable for throttled ROS diagnostics and tests."""
        tracks = []
        for candidate in (self._active[key] for key in sorted(self._active)):
            entries = [entry for entry in candidate.window if entry.valid and entry.position is not None]
            inliers = 0
            if entries:
                points = np.asarray([entry.position for entry in entries], dtype=float)
                median = np.median(points, axis=0)
                inliers = int(np.sum(np.linalg.norm(points - median, axis=1) <= self.config.candidate_radius))
            tracks.append({"track_id": candidate.track_id, "state": candidate.state.value, "window_length": len(candidate.window), "valid_count": len(entries), "inlier_count": inliers, "consecutive_misses": candidate.consecutive_misses, "last_valid_stamp": candidate.last_valid_stamp, "last_association": candidate.last_association, "confirmation_stamp": candidate.confirmation_stamp})
        return {"active_count": len(self._active), "history_count": len(self._history), "tracks": tracks, "events": list(self._events[-50:]), "rejected_frames": self._rejected_frames, "rejected_observations": self._rejected_observations, "last_processed_stamp": self._last_processed_stamp}
