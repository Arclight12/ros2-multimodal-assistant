import math

import pytest

from assistant_motion.arm_kinematics import (
    ArmGeometry,
    JointLimits,
    forward_kinematics,
    solve_ik,
)

GEOMETRY = ArmGeometry()
LIMITS = JointLimits()


def within(value, bounds):
    return bounds[0] <= value <= bounds[1]


@pytest.mark.parametrize(
    "point",
    [(0.12, 0.0, 0.0), (0.10, 0.05, 0.02), (0.08, -0.06, 0.0)],
)
def test_ik_round_trips_through_forward_kinematics(point):
    angles = solve_ik(*point)

    assert angles is not None
    tip = forward_kinematics(angles, GEOMETRY)
    assert all(abs(a - b) < 1e-6 for a, b in zip(tip, point))


def test_solution_angles_stay_inside_joint_limits():
    angles = solve_ik(0.10, 0.05, 0.02)

    assert angles is not None
    assert within(angles.base, LIMITS.base)
    assert within(angles.shoulder, LIMITS.shoulder)
    assert within(angles.elbow, LIMITS.elbow)
    assert within(angles.wrist, LIMITS.wrist)


def test_vertical_pitch_is_preferred_when_reachable():
    angles = solve_ik(0.12, 0.0, 0.0)

    assert angles is not None
    pitch = angles.shoulder + angles.elbow + angles.wrist
    assert abs(pitch + math.pi / 2) < 1e-6


def test_far_point_is_unreachable():
    assert solve_ik(0.5, 0.0, 0.0) is None


def test_point_behind_base_is_outside_base_limit():
    assert solve_ik(-0.1, 0.0, 0.0) is None


@pytest.mark.parametrize(
    "point",
    [
        (float("nan"), 0.0, 0.0),
        (0.1, float("inf"), 0.0),
        (0.1, 0.0, float("-inf")),
    ],
)
def test_non_finite_input_returns_none(point):
    assert solve_ik(*point) is None
