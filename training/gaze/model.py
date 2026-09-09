"""Custom gaze feature-fusion regression model."""

from __future__ import annotations

try:
    import torch
    import torch.nn as nn
except ImportError:  # pragma: no cover - training dependency
    torch = None
    nn = None


if nn is not None:

    class FusionRegressor(nn.Module):
        """Fuse left-eye, right-eye, and head landmark features."""

        def __init__(self):
            super().__init__()
            self.left_eye = nn.Sequential(nn.Linear(15, 32), nn.ReLU())
            self.right_eye = nn.Sequential(nn.Linear(12, 32), nn.ReLU())
            self.head = nn.Sequential(nn.Linear(3, 16), nn.ReLU())
            self.regression = nn.Sequential(
                nn.Linear(80, 32), nn.ReLU(), nn.Linear(32, 2), nn.Sigmoid()
            )

        def forward(self, values):
            left = self.left_eye(values[:, :15])
            right = self.right_eye(values[:, 15:27])
            head = self.head(values[:, 27:30])
            return self.regression(torch.cat((left, right, head), dim=1))

else:

    class FusionRegressor:  # pragma: no cover - only used for a clear error
        """Placeholder type when torch is not installed."""


def build_model(input_size: int = 30):
    """Build the custom regressor for ten 3-D face landmarks."""
    if torch is None or nn is None:
        raise RuntimeError("Install torch to build the gaze model.")
    if input_size != 30:
        raise ValueError("the first model expects ten 3-D face landmarks")
    return FusionRegressor()
