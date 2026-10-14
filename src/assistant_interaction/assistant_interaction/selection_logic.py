"""Pure gaze-to-detection selection and temporal confirmation logic."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class GazeObservation:
    """Normalized gaze position and confidence."""

    x: float
    y: float
    confidence: float
    valid: bool = True


@dataclass(frozen=True)
class DetectionObservation:
    """Detection data needed by selection, independent of ROS message types."""

    object_id: str
    class_name: str
    confidence: float
    image_x: float
    image_y: float
    image_width: float
    image_height: float
    workspace_xy: tuple[float, float] | None = None
    robot_pose: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class SelectionConfig:
    """Thresholds used for one-frame filtering."""

    minimum_gaze_confidence: float = 0.8
    minimum_detection_confidence: float = 0.55
    maximum_gaze_object_distance: float = 0.2


@dataclass(frozen=True)
class SelectionCandidate:
    """Nearest detection that passed the configured filters."""

    object_id: str
    distance: float
    detection: DetectionObservation


def nearest_valid_detection(
    gaze: GazeObservation,
    detections: Iterable[DetectionObservation],
    config: SelectionConfig,
) -> SelectionCandidate | None:
    """Return the closest valid object to the gaze point."""
    if (
        not gaze.valid
        or not all(math.isfinite(value) for value in (gaze.x, gaze.y, gaze.confidence))
        or gaze.confidence < config.minimum_gaze_confidence
        or not 0.0 <= gaze.x <= 1.0
        or not 0.0 <= gaze.y <= 1.0
    ):
        return None

    nearest: SelectionCandidate | None = None
    for detection in detections:
        if (
            not detection.object_id
            or not math.isfinite(detection.confidence)
            or detection.confidence < config.minimum_detection_confidence
            or detection.image_width <= 0.0
            or detection.image_height <= 0.0
        ):
            continue
        object_x = detection.image_x / detection.image_width
        object_y = detection.image_y / detection.image_height
        if not 0.0 <= object_x <= 1.0 or not 0.0 <= object_y <= 1.0:
            continue
        distance = math.hypot(gaze.x - object_x, gaze.y - object_y)
        if distance > config.maximum_gaze_object_distance:
            continue
        candidate = SelectionCandidate(detection.object_id, distance, detection)
        if nearest is None or distance < nearest.distance:
            nearest = candidate
    return nearest


class TemporalSelector:
    """Confirm one candidate after it stays stable for a hold duration.

    Gaze is noisy, so a candidate may drop out for up to ``grace_seconds``
    (a jittery frame, a missed detection) without restarting the hold.
    """

    def __init__(self, hold_seconds: float, grace_seconds: float = 0.5) -> None:
        """Create a selector with non-negative hold and grace durations."""
        self._hold_seconds = max(0.0, float(hold_seconds))
        self._grace_seconds = max(0.0, float(grace_seconds))
        self._candidate_id: str | None = None
        self._candidate_started = 0.0
        self._last_seen = 0.0
        self._published = False

    def _restart(self, candidate_id: str | None, now: float) -> None:
        self._candidate_id = candidate_id
        self._candidate_started = now
        self._last_seen = now
        self._published = False

    def update(
        self, candidate: SelectionCandidate | DetectionObservation | None, now: float
    ) -> SelectionCandidate | DetectionObservation | None:
        """Update the current candidate and return a newly confirmed target."""
        candidate_id = getattr(candidate, "object_id", None)
        if candidate_id is None:
            if (
                self._candidate_id is not None
                and now - self._last_seen > self._grace_seconds
            ):
                self._restart(None, now)
            return None
        if (
            candidate_id != self._candidate_id
            or now - self._last_seen > self._grace_seconds
        ):
            self._restart(candidate_id, now)
        self._last_seen = now
        if self._published or now - self._candidate_started < self._hold_seconds:
            return None
        self._published = True
        return candidate
