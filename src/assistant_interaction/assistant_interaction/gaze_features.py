"""Face-landmark features shared by gaze training and the gaze node."""

from __future__ import annotations

import math

# MediaPipe FaceMesh indices (refine_landmarks=True): eye corners and lids,
# iris centres, chin, nose tip, forehead, and mouth corners.
_POINTS = (33, 133, 159, 145, 263, 362, 386, 374, 468, 473, 152, 1, 10, 61, 291)
# (outer corner, inner corner, iris centre) for each eye.
_EYES = ((33, 133, 468), (263, 362, 473))
FEATURE_COUNT = 3 * len(_POINTS) + 2 * len(_EYES) + 3


def landmark_features(landmarks) -> list[float]:
    """Return head-relative landmark features scaled by inter-ocular distance.

    Dividing by the eye-corner distance cancels how far the user sits from the
    camera; the iris offset inside each eye is the direct gaze cue.
    """
    nose = landmarks[1]
    scale = math.hypot(
        landmarks[33].x - landmarks[263].x, landmarks[33].y - landmarks[263].y
    )
    if scale <= 0.0:
        raise ValueError("degenerate face landmarks")
    features = []
    for index in _POINTS:
        point = landmarks[index]
        features.extend(
            (
                (point.x - nose.x) / scale,
                (point.y - nose.y) / scale,
                (point.z - nose.z) / scale,
            )
        )
    for outer, inner, iris in _EYES:
        centre_x = (landmarks[outer].x + landmarks[inner].x) / 2.0
        centre_y = (landmarks[outer].y + landmarks[inner].y) / 2.0
        features.extend(
            (
                (landmarks[iris].x - centre_x) / scale,
                (landmarks[iris].y - centre_y) / scale,
            )
        )
    features.extend((nose.x, nose.y, scale))
    return features
