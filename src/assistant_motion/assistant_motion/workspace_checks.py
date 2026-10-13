"""Small pose checks for the motion boundary."""

from __future__ import annotations

import math


def valid_pose(pose: tuple[float, float, float] | None) -> bool:
    """Return whether a Cartesian target contains finite coordinates."""
    return (
        pose is not None
        and len(pose) == 3
        and all(math.isfinite(value) for value in pose)
    )
