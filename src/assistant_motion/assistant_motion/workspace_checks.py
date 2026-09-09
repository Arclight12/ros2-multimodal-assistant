"""Small pose and reachability checks for the motion boundary."""

from __future__ import annotations

import math


def valid_pose(pose: tuple[float, float, float] | None) -> bool:
    """Return whether a Cartesian target contains finite coordinates."""
    return (
        pose is not None
        and len(pose) == 3
        and all(math.isfinite(value) for value in pose)
    )


def reachable(
    pose: tuple[float, float, float] | None,
    minimum_radius_m: float,
    maximum_radius_m: float,
) -> bool:
    """Check the configured planar radial reach envelope."""
    if (
        not valid_pose(pose)
        or minimum_radius_m < 0.0
        or maximum_radius_m < minimum_radius_m
    ):
        return False
    radius = math.hypot(pose[0], pose[1])
    return minimum_radius_m <= radius <= maximum_radius_m
