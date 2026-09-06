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
    minimum_detection_confidence: float = 0.7
    maximum_gaze_object_distance: float = 0.15


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
    """Confirm one candidate after it remains stable for a hold duration."""

    def __init__(self, hold_seconds: float) -> None:
        """Create a selector with a non-negative hold duration."""
        self._hold_seconds = max(0.0, float(hold_seconds))
        self._candidate_id: str | None = None
        self._candidate_started: float | None = None
        self._published = False

    def update(
        self, candidate: SelectionCandidate | DetectionObservation | None, now: float
    ) -> SelectionCandidate | DetectionObservation | None:
        """Update the current candidate and return a newly confirmed target."""
        candidate_id = getattr(candidate, "object_id", None)
        if candidate_id != self._candidate_id:
            self._candidate_id = candidate_id
            self._candidate_started = now if candidate_id is not None else None
            self._published = False
            return None
        if candidate is None or self._candidate_started is None:
            return None
        if now - self._candidate_started < self._hold_seconds or self._published:
            return None
        self._published = True
        return candidate
