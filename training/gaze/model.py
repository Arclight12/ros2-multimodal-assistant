"""Gaze regression model over the shared face-landmark features."""

from __future__ import annotations

try:
    import torch
    import torch.nn as nn
except ImportError:  # pragma: no cover - training dependency
    torch = None
    nn = None


if nn is not None:

    class GazeRegressor(nn.Module):
        """Standardize features inside the model, then regress `(x, y)` in 0-1."""

        def __init__(self, mean, std, hidden=(256, 128, 64), dropout=0.1):
            super().__init__()
            self.register_buffer("mean", mean)
            self.register_buffer("std", std)
            layers, width = [], mean.numel()
            for size in hidden:
                layers += [nn.Linear(width, size), nn.ReLU(), nn.Dropout(dropout)]
                width = size
            self.net = nn.Sequential(*layers, nn.Linear(width, 2), nn.Sigmoid())

        def forward(self, values):
            return self.net((values - self.mean) / self.std)

else:

    class GazeRegressor:  # pragma: no cover - only used for a clear error
        """Placeholder type when torch is not installed."""


def build_model(mean, std):
    """Build the regressor; `mean` and `std` are per-feature training stats."""
    if torch is None or nn is None:
        raise RuntimeError("Install torch to build the gaze model.")
    return GazeRegressor(mean, std)
