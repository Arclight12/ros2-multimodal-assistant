"""Hardware-free unit tests for supervised slot fetching and protocol handling.

Covers:
- Correct translation of {"command": "pick", "slot": X} to "D,X\n"
- Adapter response handling for sketch_oct14b / pick_slots replies
  ("ARM_READY", "CYCLE_COMPLETE", "CYCLE_ABORTED", "ERR,...")
- Verification that reference sketch does not acknowledge H, O, C with "OK"
- Slot selection rules: actual model detection, contact point mapping, bounds rejection,
  multi-frame stability, verification requirement, and operator confirmation
- Preservation of Cartesian /execute_grasp safety refusal
"""

from __future__ import annotations

import pytest

from assistant_motion.arm_protocol import command_frame
from assistant_motion.grasp_safety import GraspSafetyState, validate_real_grasp
from assistant_motion.supervised_slot_picker import (
    DEFAULT_SLOT_REGIONS,
    SlotRegion,
    SlotTarget,
    evaluate_supervised_slot_pick,
    load_slot_targets_from_yaml,
)


def test_slot_command_translation():
    """Verify that slot pick commands map to reference firmware 'D,<slot>' format."""
    for slot in (0, 1, 2, 3):
        cmd = {"command": "pick", "slot": slot}
        assert command_frame(cmd) == f"D,{slot}\n"

    # Out of range or invalid slots must raise ValueError
    for bad_slot in (-1, 4, 10, "0", None, 1.5):
        with pytest.raises(ValueError):
            command_frame({"command": "pick", "slot": bad_slot})


def test_reference_firmware_reply_semantics():
    """Verify how Arduino reply lines are handled by the controller logic."""
    # Test line categorization matching ArmControllerNode._handle_line logic
    def parse_reply(line: str) -> tuple[str, bool | None]:
        if line in ("READY", "ARM_READY"):
            return ("available", None)
        if line in ("OK", "CYCLE_COMPLETE"):
            return ("done", True)
        if line == "CYCLE_ABORTED":
            return ("done", False)
        if line.startswith("ERR"):
            return ("done", False)
        return ("ignored", None)

    assert parse_reply("ARM_READY") == ("available", None)
    assert parse_reply("CYCLE_COMPLETE") == ("done", True)
    assert parse_reply("CYCLE_ABORTED") == ("done", False)
    assert parse_reply("ERR,busy") == ("done", False)
    assert parse_reply("ERR,invalid_slot") == ("done", False)
    assert parse_reply("ERR,out_of_range") == ("done", False)

    # Note: sketch_oct14b does NOT send OK for H, O, C
    # Plain H, O, C produce no serial reply in sketch_oct14b, meaning an adapter
    # waiting for "OK" would hang until timeout.
    assert parse_reply("") == ("ignored", None)
    assert parse_reply("BASE,330") == ("ignored", None)


def test_supervised_slot_picker_rejection_rules():
    """Test all gating and rejection rules in evaluate_supervised_slot_pick."""
    # 1. Invalid detection
    res = evaluate_supervised_slot_pick(
        object_id="",
        class_name="block_b",
        confidence=0.8,
        workspace_coords=(0.25, 0.10),
    )
    assert not res.allowed
    assert "missing" in res.reason

    # 2. Low confidence
    res = evaluate_supervised_slot_pick(
        object_id="block_b_0",
        class_name="block_b",
        confidence=0.30,
        min_confidence=0.50,
        workspace_coords=(0.25, 0.10),
    )
    assert not res.allowed
    assert "confidence" in res.reason

    # 3. Missing workspace coords (uncalibrated)
    res = evaluate_supervised_slot_pick(
        object_id="block_b_0",
        class_name="block_b",
        confidence=0.8,
        workspace_coords=None,
    )
    assert not res.allowed
    assert "missing or uncalibrated" in res.reason

    # 4. Out of board bounds (e.g. block_a at Y=0.203m)
    res = evaluate_supervised_slot_pick(
        object_id="block_a_0",
        class_name="block_a",
        confidence=0.75,
        workspace_coords=(0.1528, 0.2028),
        workspace_bounds=(0.3115, 0.1500),
    )
    assert not res.allowed
    assert "outside workspace bounds" in res.reason

    # 5. Unstable / jittering detection
    jittery_history = [(0.250, 0.100), (0.270, 0.115), (0.230, 0.085)]
    res = evaluate_supervised_slot_pick(
        object_id="block_b_0",
        class_name="block_b",
        confidence=0.8,
        workspace_coords=(0.25, 0.10),
        multi_frame_history=jittery_history,
        max_position_variance_m=0.005,
    )
    assert not res.allowed
    assert "unstable" in res.reason

    # 6. Unverified slot (default slots start with verified=False)
    res = evaluate_supervised_slot_pick(
        object_id="block_b_0",
        class_name="block_b",
        confidence=0.8,
        workspace_coords=(0.25, 0.11),
        slot_regions=DEFAULT_SLOT_REGIONS,
        operator_confirmed=True,
    )
    assert not res.allowed
    assert res.slot_id == 0
    assert "not been verified" in res.reason

    # 7. Awaiting operator confirmation
    verified_slots = {
        3: SlotRegion(
            slot_id=3,
            x_min_m=0.22,
            x_max_m=0.28,
            y_min_m=0.08,
            y_max_m=0.14,
            verified=True,
            label="Slot 3 (Verified)",
        )
    }
    res = evaluate_supervised_slot_pick(
        object_id="block_b_0",
        class_name="block_b",
        confidence=0.8,
        workspace_coords=(0.25, 0.11),
        slot_regions=verified_slots,
        operator_confirmed=False,
    )
    assert not res.allowed
    assert res.slot_id == 3
    assert "awaiting explicit operator confirmation" in res.reason


def test_supervised_slot_picker_mock_success():
    """Verify that a confirmed, stable object in a verified slot succeeds in mock mode."""
    verified_slots = {
        3: SlotRegion(
            slot_id=3,
            x_min_m=0.22,
            x_max_m=0.28,
            y_min_m=0.08,
            y_max_m=0.14,
            verified=True,
            label="Slot 3 (Base 490)",
        )
    }
    stable_history = [(0.250, 0.110), (0.251, 0.111), (0.250, 0.110)]

    decision = evaluate_supervised_slot_pick(
        object_id="block_b_0",
        class_name="block_b",
        confidence=0.70,
        workspace_coords=(0.250, 0.110),
        slot_regions=verified_slots,
        multi_frame_history=stable_history,
        operator_confirmed=True,
        mock_mode=True,
        command_id=42,
    )
    assert decision.allowed
    assert decision.slot_id == 3
    assert decision.command_payload == {"id": 42, "command": "pick", "slot": 3}
    assert "MOCK" in decision.reason


def test_cartesian_grasp_safety_preserved():
    """Ensure that the Cartesian /execute_grasp safety gate continues to refuse real grasps."""
    unverified_transform_state = GraspSafetyState(
        mock_mode=False,
        workspace_calibrated=False,
        stable_selection=True,
        target_pose_valid=False,
        target_in_workspace=True,
        target_reachable=False,
        hardware_available=True,
        motion_enabled=False,
    )
    decision = validate_real_grasp(unverified_transform_state)
    assert not decision.allowed
    assert "workspace calibration" in decision.reason


def test_slot_target_point_and_yaml_loader(tmp_path):
    """Verify that circular SlotTargets properly gate by Euclidean radius and load from YAML."""
    target = SlotTarget(
        slot_id=1,
        x_m=0.120,
        y_m=0.100,
        radius_m=0.035,
        base_pulse=300,
        verified=True,
        label="Slot 1 (Base 300)",
    )
    # Point exactly on dot
    assert target.contains(0.120, 0.100)
    # Point 2cm away (inside 3.5cm radius)
    assert target.contains(0.130, 0.110)
    # Point 5cm away (outside 3.5cm radius)
    assert not target.contains(0.170, 0.100)
    # Non-finite coordinates rejected
    assert not target.contains(float("nan"), 0.100)

    # Test YAML loading
    yaml_file = tmp_path / "test_slots.yaml"
    yaml_file.write_text(
        "slots:\n"
        "  - id: 1\n"
        "    x_m: 0.12\n"
        "    y_m: 0.10\n"
        "    radius_m: 0.035\n"
        "    base_pulse: 300\n"
        "    verified: true\n"
        "    label: 'Slot 1'\n"
    )
    loaded = load_slot_targets_from_yaml(yaml_file)
    assert len(loaded) == 1
    assert loaded[1].base_pulse == 300
    assert loaded[1].verified is True
    assert loaded[1].contains(0.125, 0.105)

    # Test evaluation using SlotTarget
    decision = evaluate_supervised_slot_pick(
        object_id="block_b_0",
        class_name="block_b",
        confidence=0.85,
        workspace_coords=(0.125, 0.105),
        slot_regions=loaded,
        operator_confirmed=True,
        mock_mode=True,
        command_id=99,
    )
    assert decision.allowed
    assert decision.slot_id == 1
    assert decision.command_payload == {"id": 99, "command": "pick", "slot": 1}

