"""Analytic inverse kinematics for the 4-DOF pick arm.

Angles are radians and lengths are metres. The base is at the origin on
the table plane, x forward, y left, z up.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ArmGeometry:
    """Link lengths in metres and shoulder pivot height above the table."""

    shoulder_height_m: float = 0.035
    upper_arm_m: float = 0.10
    forearm_m: float = 0.08
    gripper_m: float = 0.11


@dataclass(frozen=True)
class JointLimits:
    """Allowed (min, max) joint angles in radians."""

    base: tuple[float, float] = (-1.5708, 1.5708)
    shoulder: tuple[float, float] = (0.0, 1.8)
    elbow: tuple[float, float] = (-2.6, 0.0)
    wrist: tuple[float, float] = (-2.0, 2.0)


@dataclass(frozen=True)
class JointAngles:
    """Base yaw, shoulder, elbow and wrist angles in radians."""

    base: float
    shoulder: float
    elbow: float
    wrist: float


def _within(value: float, bounds: tuple[float, float]) -> bool:
    """Return whether a value lies inside an inclusive (min, max) range."""
    return bounds[0] <= value <= bounds[1]


def _two_link(
    wrist_r: float,
    wrist_z: float,
    geometry: ArmGeometry,
) -> tuple[float, float] | None:
    """Solve the elbow-up shoulder and elbow angles for a wrist point."""
    upper = geometry.upper_arm_m
    forearm = geometry.forearm_m
    reach_sq = wrist_r * wrist_r + wrist_z * wrist_z
    cos_elbow = (reach_sq - upper * upper - forearm * forearm) / (2.0 * upper * forearm)
    if abs(cos_elbow) > 1.0:
        return None
    elbow = -math.acos(cos_elbow)
    k1 = upper + forearm * math.cos(elbow)
    k2 = forearm * math.sin(elbow)
    shoulder = math.atan2(wrist_z, wrist_r) - math.atan2(k2, k1)
    return shoulder, elbow


def solve_ik(
    x: float,
    y: float,
    z: float,
    geometry: ArmGeometry = ArmGeometry(),
    limits: JointLimits = JointLimits(),
    pitch_steps: int = 19,
) -> JointAngles | None:
    """Return joint angles that place the fingertip at (x, y, z), or None.

    Tries gripper pitches from vertical (-pi/2) toward -pi/4 and returns
    the first solution inside the joint limits.
    """
    if not all(math.isfinite(value) for value in (x, y, z)):
        return None
    base = math.atan2(y, x)
    if not _within(base, limits.base):
        return None
    radius = math.hypot(x, y)
    height = z - geometry.shoulder_height_m
    span = (math.pi / 4) / max(pitch_steps - 1, 1)
    for step in range(pitch_steps):
        pitch = -math.pi / 2 + step * span
        wrist_r = radius - geometry.gripper_m * math.cos(pitch)
        wrist_z = height - geometry.gripper_m * math.sin(pitch)
        solved = _two_link(wrist_r, wrist_z, geometry)
        if solved is None:
            continue
        shoulder, elbow = solved
        wrist = pitch - shoulder - elbow
        if (
            _within(shoulder, limits.shoulder)
            and _within(elbow, limits.elbow)
            and _within(wrist, limits.wrist)
        ):
            return JointAngles(base, shoulder, elbow, wrist)
    return None
