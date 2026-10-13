"""Pure translation of arm commands into the Arduino serial protocol."""

from __future__ import annotations

import json
import math
from typing import Any, Sequence

SERVO_MIN_DEG = 0.0
SERVO_MAX_DEG = 180.0
_SIMPLE_FRAMES = {"home": "H", "open_gripper": "O", "close_gripper": "C"}


def joints_to_servo_degrees(
    angles: Sequence[float],
    offsets_deg: Sequence[float],
    signs: Sequence[float],
) -> list[float]:
    """Map model joint angles (rad) to servo degrees per joint."""
    if not len(angles) == len(offsets_deg) == len(signs) == 4:
        raise ValueError("expected four angles, offsets and signs")
    degrees = []
    for angle, offset, sign in zip(angles, offsets_deg, signs):
        value = float(offset) + float(sign) * math.degrees(float(angle))
        if not math.isfinite(value) or not (
            SERVO_MIN_DEG <= value <= SERVO_MAX_DEG
        ):
            raise ValueError(f"servo angle {value} is outside 0..180")
        degrees.append(value)
    return degrees


def parse_command(payload: str) -> dict[str, Any]:
    """Decode and shape-check one /arm/command JSON message."""
    command = json.loads(payload)
    if not isinstance(command, dict) or not isinstance(
        command.get("command"), str
    ):
        raise ValueError("command must be an object with a string command")
    return command


def command_frame(
    command: dict[str, Any],
    offsets_deg: Sequence[float],
    signs: Sequence[float],
) -> str:
    """Return the newline-terminated serial frame for one command."""
    name = command["command"]
    if name in _SIMPLE_FRAMES:
        return _SIMPLE_FRAMES[name] + "\n"
    if name != "move_joints":
        raise ValueError(f"unknown command {name!r}")
    angles = command.get("angles")
    if not isinstance(angles, (list, tuple)) or len(angles) != 4:
        raise ValueError("move_joints requires four joint angles")
    degrees = joints_to_servo_degrees(angles, offsets_deg, signs)
    return "J," + ",".join(f"{value:.1f}" for value in degrees) + "\n"
