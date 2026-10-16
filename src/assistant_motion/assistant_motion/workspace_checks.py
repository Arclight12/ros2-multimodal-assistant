"""Small pose checks and channel clamps for the motion boundary."""

from __future__ import annotations

import math

from .arm_protocol import (
    CHANNELS,
    PULSE_CENTER,
    PULSE_MAX,
    PULSE_MIN,
    base_deg_to_pulse,
    pulse_to_base_deg,
)

# A target that overshoots the calibrated band by less than the margin is
# clamped onto the band (the arm lands on the nearest safe pose). Past it the
# target is somewhere the arm cannot go, so it is refused instead.
REACH_CLAMP_MARGIN_M = 0.02
BASE_CLAMP_MARGIN_DEG = 5.0

# Used when robot.yaml has no channel_limits entry: the firmware's own range.
DEFAULT_CHANNEL_LIMITS = {
    channel: {"min": PULSE_MIN, "max": PULSE_MAX, "center": PULSE_CENTER}
    for channel in CHANNELS
}

_LIMIT_FIELDS = ("min", "max", "center")


def valid_pose(pose: tuple[float, float, float] | None) -> bool:
    """Return whether a Cartesian target contains finite coordinates."""
    return (
        pose is not None
        and len(pose) == 3
        and all(math.isfinite(value) for value in pose)
    )


def load_channel_limits(node) -> dict[int, dict[str, int]]:
    """Read the per-channel pulse limits from the node's parameters.

    robot.yaml nests them as ``channel_limits: {0: {min: ..}}``; ROS flattens
    that into the dotted parameters ``channel_limits.<channel>.<field>``. The
    firmware values are the defaults, so a missing file changes nothing.
    """
    limits: dict[int, dict[str, int]] = {}
    for channel, defaults in DEFAULT_CHANNEL_LIMITS.items():
        entry: dict[str, int] = {}
        for field in _LIMIT_FIELDS:
            name = f"channel_limits.{channel}.{field}"
            node.declare_parameter(name, defaults[field])
            entry[field] = int(node.get_parameter(name).value)
        limits[channel] = entry
    return limits


def clamp_channel(
    channel: int,
    pulse: float,
    channel_limits: dict[int, dict[str, int]] | None = None,
) -> tuple[int, bool]:
    """Clamp one PCA9685 pulse into the channel's range.

    Returns the clamped pulse and whether it had to move. The Arduino keeps its
    own check (``ERR,out_of_range``), which stays the final gate.
    """
    limits = (channel_limits or {}).get(channel) or DEFAULT_CHANNEL_LIMITS.get(
        channel, {}
    )
    low = int(limits.get("min", PULSE_MIN))
    high = int(limits.get("max", PULSE_MAX))
    if low > high:
        low, high = high, low
    value = int(round(pulse))
    clamped = min(max(value, low), high)
    return clamped, clamped != value


def base_deg_range(
    channel_limits: dict[int, dict[str, int]] | None = None,
) -> tuple[float, float]:
    """Return the base angle band implied by the ch0 pulse limits."""
    limits = (channel_limits or {}).get(0) or DEFAULT_CHANNEL_LIMITS[0]
    low = float(limits.get("min", PULSE_MIN))
    high = float(limits.get("max", PULSE_MAX))
    if low > high:
        low, high = high, low
    return pulse_to_base_deg(low), pulse_to_base_deg(high)


def clamp_command(
    command: dict,
    channel_limits: dict[int, dict[str, int]] | None = None,
    reach_range_m: tuple[float, float] | None = None,
) -> tuple[dict, list[str]]:
    """Clamp a ``move_to`` command onto the calibrated bands before sending it.

    The base angle is clamped through its ch0 pulse, and the reach is clamped
    into the grid band (ch1/ch2 are only commanded through recorded pairs, so
    their pulses are bounded by reach, exactly as the firmware constrains it).
    Returns the command and a note per channel that had to move.
    """
    if command.get("command") != "move_to":
        return command, []
    try:
        base_deg = float(command["base_deg"])
        reach_mm = float(command["reach_mm"])
    except (KeyError, TypeError, ValueError):
        # Malformed here means malformed in command_frame(); let it complain.
        return command, []
    clamped = dict(command)
    notes: list[str] = []
    pulse = base_deg_to_pulse(base_deg)
    pulse, moved = clamp_channel(0, pulse, channel_limits)
    if moved:
        clamped["base_deg"] = pulse_to_base_deg(pulse)
        notes.append(f"ch0 base {base_deg:.1f}deg -> pulse {pulse}")
    if reach_range_m is not None:
        low = min(reach_range_m) * 1000.0
        high = max(reach_range_m) * 1000.0
        bounded = min(max(reach_mm, low), high)
        if bounded != reach_mm:
            clamped["reach_mm"] = bounded
            notes.append(f"ch1/2 reach {reach_mm:.1f}mm -> {bounded:.1f}mm")
    return clamped, notes


def arm_target(
    pose: tuple[float, float, float],
    reach_min_m: float,
    reach_max_m: float,
    base_limit_deg: float,
    channel_limits: dict[int, dict[str, int]] | None = None,
) -> tuple[float, float] | None:
    """Return the clamped (base_deg, reach_mm) for a target, or None if unreachable.

    Base 0 is straight ahead (+x), positive to the left. The arm grasps at its
    calibrated table height, so z is ignored. The base angle is clamped so its
    ch0 pulse stays inside the channel limits, and the reach is clamped into
    the calibrated grid band; only a target outside a band by more than the
    clamp margin (genuinely out of reach) is refused.
    """
    if not all(math.isfinite(value) for value in pose):
        return None
    base_deg = math.degrees(math.atan2(pose[1], pose[0]))
    reach_m = math.hypot(pose[0], pose[1])

    band_low, band_high = base_deg_range(channel_limits)
    sweep = min(base_limit_deg, abs(band_low), abs(band_high))
    if abs(base_deg) > sweep + BASE_CLAMP_MARGIN_DEG:
        return None
    base_deg = min(max(base_deg, -sweep), sweep)

    reach_low = min(reach_min_m, reach_max_m)
    reach_high = max(reach_min_m, reach_max_m)
    if (
        not reach_low - REACH_CLAMP_MARGIN_M
        <= reach_m
        <= (reach_high + REACH_CLAMP_MARGIN_M)
    ):
        return None
    reach_m = min(max(reach_m, reach_low), reach_high)
    return base_deg, reach_m * 1000.0
