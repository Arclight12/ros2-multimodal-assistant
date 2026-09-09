"""Small loader for preprocessed gaze NPZ files."""

from __future__ import annotations

from pathlib import Path

import numpy as np


def load_dataset(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """Load features and normalized `(x, y)` targets from an NPZ file."""
    with np.load(path) as data:
        features = np.asarray(data["features"], dtype=np.float32)
        targets = np.asarray(data["targets"], dtype=np.float32)
    if features.ndim != 2 or targets.ndim != 2 or targets.shape[1] != 2:
        raise ValueError("gaze dataset must contain 2-D features and two targets")
    if len(features) != len(targets) or len(features) < 2:
        raise ValueError("gaze dataset needs at least two matching samples")
    return features, targets
