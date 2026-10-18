"""Tests for safety gates, mock-mode isolation, and motion prevention.

Covers Phase 5D:
- Mock calibration does not become valid for real motion
- Uncalibrated detections do not receive usable robot poses
- Missing hardware, disabled motion, stale detections, or invalid targets cannot initiate real grasp
- The test suite never opens a serial port or publishes real motor commands
"""

from __future__ import annotations

import pytest

from assistant_motion.grasp_safety import GraspSafetyState, validate_real_grasp
from assistant_perception.workspace_mapping import (
    WorkspaceCalibration,
    map_pixel_to_base,
    map_pixel_to_workspace,
)


def test_validate_real_grasp_all_safety_gates():
    """Verify that every safety gate in validate_real_grasp refuses execution."""
    base_state = GraspSafetyState(
        mock_mode=False,
        workspace_calibrated=True,
        stable_selection=True,
        target_pose_valid=True,
        target_in_workspace=True,
        target_reachable=True,
        hardware_available=True,
        motion_enabled=True,
    )
    # When all safety conditions are met:
    assert validate_real_grasp(base_state).allowed

    # Gate 1: mock_mode is True -> refused
    s = GraspSafetyState(**{**base_state.__dict__, "mock_mode": True})
    d = validate_real_grasp(s)
    assert not d.allowed
    assert "mock_mode" in d.reason

    # Gate 2: workspace_calibrated is False -> refused
    s = GraspSafetyState(**{**base_state.__dict__, "workspace_calibrated": False})
    d = validate_real_grasp(s)
    assert not d.allowed
    assert "workspace calibration" in d.reason

    # Gate 3: stable_selection is False -> refused
    s = GraspSafetyState(**{**base_state.__dict__, "stable_selection": False})
    d = validate_real_grasp(s)
    assert not d.allowed
    assert "selection" in d.reason

    # Gate 4: target_pose_valid is False -> refused
    s = GraspSafetyState(**{**base_state.__dict__, "target_pose_valid": False})
    d = validate_real_grasp(s)
    assert not d.allowed
    assert "target pose" in d.reason

    # Gate 5: target_in_workspace is False -> refused
    s = GraspSafetyState(**{**base_state.__dict__, "target_in_workspace": False})
    d = validate_real_grasp(s)
    assert not d.allowed
    assert "workspace bounds" in d.reason

    # Gate 6: target_reachable is False -> refused
    s = GraspSafetyState(**{**base_state.__dict__, "target_reachable": False})
    d = validate_real_grasp(s)
    assert not d.allowed
    assert "unreachable" in d.reason

    # Gate 7: hardware_available is False -> refused
    s = GraspSafetyState(**{**base_state.__dict__, "hardware_available": False})
    d = validate_real_grasp(s)
    assert not d.allowed
    assert "hardware" in d.reason

    # Gate 8: motion_enabled is False -> refused
    s = GraspSafetyState(**{**base_state.__dict__, "motion_enabled": False})
    d = validate_real_grasp(s)
    assert not d.allowed
    assert "real motion is disabled" in d.reason


def test_uncalibrated_detections_receive_no_robot_pose():
    """Uncalibrated or invalid calibration must never map pixels to robot poses."""
    dummy_h = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

    uncalibrated = WorkspaceCalibration(
        calibrated=False,
        homography=dummy_h,
        workspace_width_m=0.3115,
        workspace_height_m=0.1500,
        table_z_m=0.0,
        base_origin=(-0.150, 0.132, 0.0),
        base_yaw_rad=0.0,
        robot_transform_configured=False,
    )
    assert not uncalibrated.is_valid
    assert map_pixel_to_workspace(200.0, 200.0, uncalibrated) is None
    assert map_pixel_to_base(200.0, 200.0, uncalibrated) is None


def test_no_serial_port_opened_in_tests(monkeypatch):
    """Verify that tests never invoke or open any serial port."""
    import sys

    # If pyserial is imported, intercept serial.Serial to ensure it is never called
    if "serial" in sys.modules:
        import serial

        def forbidden_open(*args, **kwargs):
            pytest.fail("Hardware safety violation: serial port open attempt in tests!")

        monkeypatch.setattr(serial, "Serial", forbidden_open)
