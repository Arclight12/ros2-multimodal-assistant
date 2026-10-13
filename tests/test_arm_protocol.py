import json
import math

import pytest

from assistant_motion.arm_protocol import command_frame, parse_command

OFFSETS = [90.0] * 4
SIGNS = [1.0] * 4


def test_simple_commands_map_to_single_letters():
    assert command_frame({"command": "home"}, OFFSETS, SIGNS) == "H\n"
    assert command_frame({"command": "open_gripper"}, OFFSETS, SIGNS) == "O\n"
    assert command_frame({"command": "close_gripper"}, OFFSETS, SIGNS) == "C\n"


def test_move_joints_applies_offset_and_sign():
    command = {"command": "move_joints", "angles": [0.0, math.pi / 4, 0, 0]}
    frame = command_frame(command, OFFSETS, [1.0, -1.0, 1.0, 1.0])
    assert frame == "J,90.0,45.0,90.0,90.0\n"


@pytest.mark.parametrize(
    "command",
    [
        {"command": "fly"},
        {"command": "move_joints"},
        {"command": "move_joints", "angles": [0, 0, 0]},
        {"command": "move_joints", "angles": [0, 0, 0, math.nan]},
        {"command": "move_joints", "angles": [3.0, 0, 0, 0]},  # > 180 deg
    ],
)
def test_bad_commands_are_rejected(command):
    with pytest.raises(ValueError):
        command_frame(command, OFFSETS, SIGNS)


def test_parse_command_rejects_non_objects():
    assert parse_command(json.dumps({"command": "home"}))["command"] == "home"
    with pytest.raises(ValueError):
        parse_command("[1, 2]")
    with pytest.raises(ValueError):
        parse_command("not json")
