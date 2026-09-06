from assistant_motion.grasp_safety import (
    GraspSafetyState,
    validate_real_grasp,
)


def safe_state(**overrides):
    values = {
        "mock_mode": False,
        "workspace_calibrated": True,
        "stable_selection": True,
        "target_pose_valid": True,
        "target_in_workspace": True,
        "target_reachable": True,
        "hardware_available": True,
        "moveit_enabled": True,
    }
    values.update(overrides)
    return GraspSafetyState(**values)


def test_real_grasp_is_allowed_only_when_every_gate_is_valid():
    decision = validate_real_grasp(safe_state())

    assert decision.allowed
    assert decision.reason == ""


def test_mock_mode_refuses_real_motion():
    decision = validate_real_grasp(safe_state(mock_mode=True))

    assert not decision.allowed
    assert "mock_mode" in decision.reason


def test_uncalibrated_or_invalid_targets_are_refused():
    assert not validate_real_grasp(
        safe_state(workspace_calibrated=False)
    ).allowed
    assert not validate_real_grasp(safe_state(target_pose_valid=False)).allowed
    assert not validate_real_grasp(safe_state(target_in_workspace=False)).allowed
    assert not validate_real_grasp(safe_state(target_reachable=False)).allowed
    assert not validate_real_grasp(safe_state(hardware_available=False)).allowed
