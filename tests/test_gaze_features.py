from types import SimpleNamespace

import pytest

from assistant_interaction.gaze_features import FEATURE_COUNT, landmark_features
from training.gaze.modal_train import DEFAULT_GPU


def _face(shift: float = 0.0, size: float = 1.0):
    base = [(0.5, 0.5)] * 478
    base[33], base[263], base[468] = (0.4, 0.4), (0.6, 0.4), (0.42, 0.41)
    return [
        SimpleNamespace(x=0.5 + shift + (x - 0.5) * size, y=0.5 + (y - 0.5) * size, z=0.0)
        for x, y in base
    ]


def test_feature_count_matches_output() -> None:
    assert len(landmark_features(_face())) == FEATURE_COUNT


def test_relative_features_ignore_head_position_and_distance() -> None:
    near = landmark_features(_face())
    moved = landmark_features(_face(shift=0.2, size=0.5))
    assert near[:-3] == pytest.approx(moved[:-3])


def test_degenerate_face_is_rejected() -> None:
    with pytest.raises(ValueError):
        landmark_features([SimpleNamespace(x=0.5, y=0.5, z=0.0)] * 478)


def test_gaze_training_uses_cheapest_gpu() -> None:
    assert DEFAULT_GPU == "T4"
