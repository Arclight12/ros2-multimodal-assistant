"""Translate ROS arm commands to the Arduino serial protocol."""

from __future__ import annotations

import json
import math
from typing import Any

_SIMPLE_FRAMES = {
    "home": "H",
    "open_gripper": "O",
    "close_gripper": "C",
    "demo_cycle": "D",
    "stop": "x",
}
# Predefined pick slots. The firmware owns the pose of each slot, so the PC
# only sends the slot label and is framed like the other one-character
# commands, with a trailing newline.
PICK_SLOTS = (0, 1, 2, 3)
# Legacy generic-motion constants. Fixed-pose demo uses per-channel limits
# in firmware/arm_controller/arm_calibration.h.
PULSE_MIN, PULSE_MAX, PULSE_CENTER = 225, 389, 307
BASE_COUNTS_PER_DEG = 2.0
CHANNELS = (0, 1, 2, 3)


def parse_command(payload: str) -> dict[str, Any]:
    command = json.loads(payload)
    if not isinstance(command, dict) or not isinstance(command.get("command"), str):
        raise ValueError("command must be an object with a string command")
    return command


def command_frame(command: dict[str, Any]) -> str:
    name = command["command"]
    if name in _SIMPLE_FRAMES:
        return _SIMPLE_FRAMES[name] + "\n"
    if name == "pick":
        # firmware/pick_slots: "D,<slot>" picks from that slot and drops the
        # object at its fixed drop position; it ends with CYCLE_COMPLETE or
        # CYCLE_ABORTED (see arm_controller_node).
        slot = command.get("slot")
        if isinstance(slot, bool) or not isinstance(slot, int):
            raise ValueError("pick needs an integer slot 0..3")
        if slot not in PICK_SLOTS:
            raise ValueError(f"pick slot {slot} is outside 0..3")
        return f"D,{slot}\n"
    if name != "move_to":
        raise ValueError(f"unknown command {name!r}")
    try:
        deg, reach, level = (
            float(command["base_deg"]),
            float(command["reach_mm"]),
            int(command["level"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("move_to needs base_deg, reach_mm and level") from exc
    if not (math.isfinite(deg) and math.isfinite(reach) and level in (0, 1)):
        raise ValueError("move_to values are invalid")
    return f"P,{deg:.1f},{reach:.1f},{level}\n"


def base_deg_to_pulse(base_deg: float) -> int:
    return int(round(PULSE_CENTER + base_deg * BASE_COUNTS_PER_DEG))


def pulse_to_base_deg(pulse: int) -> float:
    return (pulse - PULSE_CENTER) / BASE_COUNTS_PER_DEG
