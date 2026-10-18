"""Unit tests for 2D continuous tabletop kinematics and serial protocol framing."""

from __future__ import annotations

import pytest

from assistant_motion.arm_protocol import command_frame
from assistant_motion.kinematics import (
    ARC_CENTER_X,
    ARC_CENTER_Y,
    MAX_SAFE_BASE,
    MIN_SAFE_BASE,
    R_MAX_SAFE_M,
    R_MIN_SAFE_M,
    compute_arm_pulses,
)
from assistant_motion.workspace_checks import clamp_command


def test_continuous_pick_command_translation():
    """Verify continuous pick translates to P,<base>,<ch1>,<ch2>,<gripper>."""
    cmd = {
        "command": "pick",
        "base": 330,
        "link1": 450,
        "link2": 510,
        "gripper": 435,
    }
    assert command_frame(cmd) == "P,330,450,510,435\n"

    # Default gripper
    cmd_no_grip = {"command": "pick", "base": 300, "link1": 440, "link2": 500}
    assert command_frame(cmd_no_grip) == "P,300,440,500,435\n"


def test_continuous_pick_kinematics_valid():
    """Verify kinematics compute valid pulses within calibrated bounds."""
    # Test sharpener at known reachable tabletop location
    # Board center roughly X=0.155, Y=0.100 -> dist ~ 0.203m
    base, ch1, ch2, grip, dist, angle = compute_arm_pulses(
        "sharpener", ARC_CENTER_X, 0.100
    )
    assert MIN_SAFE_BASE <= base <= MAX_SAFE_BASE
    assert 390 <= ch1 <= 510
    assert 400 <= ch2 <= 550
    assert grip == 440  # sharpener tighter grip
    assert R_MIN_SAFE_M <= dist <= R_MAX_SAFE_M


def test_continuous_pick_kinematics_rejections():
    """Verify kinematics reject out-of-envelope coordinates."""
    # Point way too close
    with pytest.raises(ValueError, match="safe envelope"):
        compute_arm_pulses("block_a", ARC_CENTER_X, ARC_CENTER_Y - 0.05)

    # Point way too far
    with pytest.raises(ValueError, match="safe envelope"):
        compute_arm_pulses("block_a", ARC_CENTER_X, ARC_CENTER_Y - 0.40)


def test_clamp_command_continuous_pick():
    """Verify clamp_command bounds over-limit continuous pick pulses."""
    limits = {
        0: {"min": 230, "max": 440, "center": 330},
        1: {"min": 390, "max": 510, "center": 430},
        2: {"min": 400, "max": 550, "center": 400},
        3: {"min": 300, "max": 480, "center": 300},
    }
    over_limit_cmd = {
        "command": "pick",
        "base": 500,  # exceeds 440
        "link1": 350,  # below 390
        "link2": 580,  # exceeds 550
        "gripper": 435,
    }
    clamped, notes = clamp_command(over_limit_cmd, limits)
    assert clamped["base"] == 440
    assert clamped["link1"] == 390
    assert clamped["link2"] == 550
    assert clamped["gripper"] == 435
    assert len(notes) == 3
