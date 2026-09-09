"""Safety gates shared by the motion planner and hardware-free tests."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GraspSafetyState:
    """Evidence required before allowing a real grasp."""

    mock_mode: bool
    workspace_calibrated: bool
    stable_selection: bool
    target_pose_valid: bool
    target_in_workspace: bool
    target_reachable: bool
    hardware_available: bool
    moveit_enabled: bool


@dataclass(frozen=True)
class SafetyDecision:
    """Result of the real-motion safety gate."""

    allowed: bool
    reason: str = ""


def validate_real_grasp(state: GraspSafetyState) -> SafetyDecision:
    """Return the first explicit reason a real grasp must be refused."""
    checks = (
        (state.mock_mode, "mock_mode is enabled"),
        (not state.workspace_calibrated, "workspace calibration is invalid"),
        (not state.stable_selection, "selection is not stable"),
        (not state.target_pose_valid, "target pose is invalid"),
        (not state.target_in_workspace, "target is outside workspace bounds"),
        (not state.target_reachable, "target is unreachable"),
        (not state.hardware_available, "robot hardware is unavailable"),
        (not state.moveit_enabled, "MoveIt execution is disabled"),
    )
    for failed, reason in checks:
        if failed:
            return SafetyDecision(False, reason)
    return SafetyDecision(True)
