"""Pure translation of arm commands into the Arduino serial protocol."""

from __future__ import annotations

import json
import math
from typing import Any

_SIMPLE_FRAMES = {"home": "H", "open_gripper": "O", "close_gripper": "C"}

# PCA9685 pulse counts, mirroring firmware/arm_controller/arm_calibration.h.
# Channel order: ch0 base, ch1 + ch2 coupled linkage, ch3 gripper.
PULSE_MIN = 225
PULSE_MAX = 389
PULSE_CENTER = 307
# The firmware's BASE_COUNTS_PER_DEG; the base is the one channel whose pulse
# this side can compute from a command, so it is the one we can clamp.
BASE_COUNTS_PER_DEG = 2.0
CHANNELS = (0, 1, 2, 3)


def parse_command(payload: str) -> dict[str, Any]:
    """Decode and shape-check one /arm/command JSON message."""
    command = json.loads(payload)
    if not isinstance(command, dict) or not isinstance(command.get("command"), str):
        raise ValueError("command must be an object with a string command")
    return command


def command_frame(command: dict[str, Any]) -> str:
    """Return the newline-terminated serial frame for one command."""
    name = command["command"]
    if name in _SIMPLE_FRAMES:
        return _SIMPLE_FRAMES[name] + "\n"
    if name != "move_to":
        raise ValueError(f"unknown command {name!r}")
    try:
        base_deg = float(command["base_deg"])
        reach_mm = float(command["reach_mm"])
        level = int(command["level"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("move_to needs base_deg, reach_mm and level") from exc
    if not (math.isfinite(base_deg) and math.isfinite(reach_mm) and level in (0, 1)):
        raise ValueError("move_to values are invalid")
    return f"P,{base_deg:.1f},{reach_mm:.1f},{level}\n"


def base_deg_to_pulse(base_deg: float) -> int:
    """Return the ch0 pulse the firmware would use for this base angle."""
    return int(round(PULSE_CENTER + base_deg * BASE_COUNTS_PER_DEG))


def pulse_to_base_deg(pulse: int) -> float:
    """Return the base angle for a ch0 pulse (the inverse mapping)."""
    return (pulse - PULSE_CENTER) / BASE_COUNTS_PER_DEG
