"""Empirical 2D polar kinematics for tabletop continuous object fetching.

Converts planar table coordinates (X, Y) in metres to MeArm PCA9685 servo pulses:
(X, Y) -> (R, theta) -> (Base, CH1, CH2, Gripper).
"""

from __future__ import annotations

import math

# Kinematics geometry & empirical calibration nodes
ARC_CENTER_X = 0.1539  # metres
ARC_CENTER_Y = 0.3034  # metres

RADIAL_NODES_M = [0.155, 0.199, 0.228]
LINK1_NODES = [400, 440, 500]
LINK2_NODES = [490, 500, 540]

R_MIN_SAFE_M = 0.145  # 14.5 cm
R_MAX_SAFE_M = 0.285  # 28.5 cm

REF_ANGLES_DEG = [-123.82, -102.27, -78.79, -56.27]
REF_PULSES = [440, 370, 300, 230]

MIN_SAFE_BASE = 230
MAX_SAFE_BASE = 440

# Height compensation: flatter objects need deeper CH2 tabletop touchdown
HEIGHT_OFFSET_CH2 = {
    "sharpener": 8,  # ~2.8 cm height -> +8 counts on CH2 for table touchdown
    "eraser": 10,  # ~1.8 cm height -> +10 counts on CH2
    "thermocol": 0,  # ~4 cm height -> standard touchdown
    "block_a": 0,  # ~4 cm height
    "block_b": 0,
    "block_c": 0,
}

# Radial pullback: smaller/narrower objects must not overshoot past front teeth into throat cavity
RADIAL_OFFSET_M = {
    "sharpener": -0.012,  # -1.2 cm pullback so gripper teeth grasp object at the tips
    "eraser": -0.010,  # -1.0 cm pullback
    "thermocol": 0.0,
    "block_a": 0.0,
    "block_b": 0.0,
    "block_c": 0.0,
}

# Gripper close pulse: smaller/narrower objects close tighter (440 vs 415 for wide blocks)
GRIPPER_CLOSE_PULSES = {
    "sharpener": 440,  # ~4-5 mm tighter closure for narrow body
    "eraser": 440,
    "thermocol": 415,
    "block_a": 415,
    "block_b": 415,
    "block_c": 415,
}

DEFAULT_GRIPPER_CLOSE = 435


def _interp(x: float, xp: list[float], fp: list[float]) -> float:
    """1D linear interpolation."""
    if x <= xp[0]:
        return float(fp[0])
    if x >= xp[-1]:
        return float(fp[-1])
    for i in range(len(xp) - 1):
        if xp[i] <= x <= xp[i + 1]:
            t = (x - xp[i]) / (xp[i + 1] - xp[i])
            return float(fp[i] + t * (fp[i + 1] - fp[i]))
    return float(fp[-1])


def compute_arm_pulses(
    cls_name: str, ws_x: float, ws_y: float
) -> tuple[int, int, int, int, float, float]:
    """Compute base, ch1, ch2, gripper pulses from workspace (ws_x, ws_y) coordinates."""
    dx = ws_x - ARC_CENTER_X
    dy = ws_y - ARC_CENTER_Y
    dist = math.hypot(dx, dy)

    if dist < R_MIN_SAFE_M or dist > R_MAX_SAFE_M:
        raise ValueError(
            f"Radial distance {dist*100:.1f} cm outside safe envelope "
            f"[{R_MIN_SAFE_M*100:.1f}, {R_MAX_SAFE_M*100:.1f}] cm"
        )

    angle_deg = math.degrees(math.atan2(dy, dx))
    if angle_deg < (REF_ANGLES_DEG[0] - 8.0) or angle_deg > (REF_ANGLES_DEG[-1] + 8.0):
        raise ValueError(
            f"Target azimuth {angle_deg:.1f}° outside sweep range "
            f"[{REF_ANGLES_DEG[0]:.1f}°, {REF_ANGLES_DEG[-1]:.1f}°]"
        )

    # Interpolate Base servo pulse
    raw_base = _interp(angle_deg, REF_ANGLES_DEG, REF_PULSES)
    base_pulse = int(round(min(max(raw_base, MIN_SAFE_BASE), MAX_SAFE_BASE)))

    # Multi-node interpolation for CH1 (reach) and CH2 (height/touchdown)
    r_offset = RADIAL_OFFSET_M.get(cls_name, 0.0)
    effective_dist = dist + r_offset
    raw_ch1 = _interp(effective_dist, RADIAL_NODES_M, LINK1_NODES)
    raw_ch2 = _interp(effective_dist, RADIAL_NODES_M, LINK2_NODES)

    offset_ch2 = HEIGHT_OFFSET_CH2.get(cls_name, 0)
    ch1_pulse = int(round(raw_ch1))
    ch2_pulse = int(round(raw_ch2 + offset_ch2))

    ch1_pulse = int(min(max(ch1_pulse, 390), 510))
    ch2_pulse = int(min(max(ch2_pulse, 400), 550))
    gripper_pulse = GRIPPER_CLOSE_PULSES.get(cls_name, DEFAULT_GRIPPER_CLOSE)

    return base_pulse, ch1_pulse, ch2_pulse, gripper_pulse, dist, angle_deg
