"""Supervised slot-based pickup evaluation and gating.

Connects camera-localized tabletop object coordinates to discrete, physically
calibrated Arduino slot positions (e.g. from sketch_oct14b.ino) without requiring
analytical inverse kinematics.

Guarantees:
1. Only actual model detections with valid confidence are evaluated.
2. The contact-point estimate on the table plane is mapped to workspace coordinates.
3. Points outside the board or outside the configured slot box are rejected (no clamping).
4. Slots that have not been physically verified are disabled by default.
5. Explicit operator confirmation is required before any command can be generated.
6. The Cartesian /execute_grasp safety model remains untouched.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import yaml


@dataclass(frozen=True)
class SlotRegion:
    """A spatial rectangular bounding box on the tabletop for a tested slot."""

    slot_id: int
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    verified: bool = False
    label: str = ""

    def contains(self, x_m: float, y_m: float) -> bool:
        """Return whether (x_m, y_m) lies strictly inside the box."""
        if not (math.isfinite(x_m) and math.isfinite(y_m)):
            return False
        return (
            self.x_min_m <= x_m <= self.x_max_m
            and self.y_min_m <= y_m <= self.y_max_m
        )


@dataclass(frozen=True)
class SlotTarget:
    """A physical circular waypoint target (dot) on the board for a tested slot."""

    slot_id: int
    x_m: float
    y_m: float
    radius_m: float = 0.035
    base_pulse: int = 230
    verified: bool = False
    label: str = ""

    def distance_to(self, x_m: float, y_m: float) -> float:
        """Calculate Euclidean distance from target center to given coordinates."""
        return math.hypot(x_m - self.x_m, y_m - self.y_m)

    def contains(self, x_m: float, y_m: float) -> bool:
        """Return whether (x_m, y_m) falls within radius_m of the dot center."""
        if not (math.isfinite(x_m) and math.isfinite(y_m)):
            return False
        return self.distance_to(x_m, y_m) <= self.radius_m


def load_slot_targets_from_yaml(yaml_path: str | Path) -> dict[int, SlotTarget]:
    """Load calibrated slot dot targets from pickup_slots.yaml."""
    path = Path(yaml_path)
    if not path.is_file():
        return {}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not data or "slots" not in data:
        return {}
    targets = {}
    for item in data["slots"]:
        sid = int(item["id"])
        targets[sid] = SlotTarget(
            slot_id=sid,
            x_m=float(item.get("x_m", 0.0)),
            y_m=float(item.get("y_m", 0.0)),
            radius_m=float(item.get("radius_m", 0.035)),
            base_pulse=int(item.get("base_pulse", 330)),
            verified=bool(item.get("verified", False)),
            label=str(item.get("label", f"Slot {sid}")),
        )
    return targets


@dataclass
class SupervisedSlotDecision:
    """Outcome of evaluating a target candidate for a supervised slot fetch."""

    allowed: bool
    slot_id: int | None
    reason: str
    command_payload: dict[str, Any] | None = None


# Default slot definitions matching sketch_oct14b.ino PICKUP_BASE[4] = {230, 300, 370, 440}
# All slots start with verified=False until physically measured on the tabletop.
DEFAULT_SLOT_REGIONS: dict[int, SlotRegion] = {
    0: SlotRegion(
        slot_id=0,
        x_min_m=0.23,
        x_max_m=0.31,
        y_min_m=0.05,
        y_max_m=0.15,
        verified=False,
        label="Slot 0 (Base 230 - Far Right)",
    ),
    1: SlotRegion(
        slot_id=1,
        x_min_m=0.15,
        x_max_m=0.23,
        y_min_m=0.05,
        y_max_m=0.15,
        verified=False,
        label="Slot 1 (Base 300 - Mid Right)",
    ),
    2: SlotRegion(
        slot_id=2,
        x_min_m=0.08,
        x_max_m=0.15,
        y_min_m=0.05,
        y_max_m=0.15,
        verified=False,
        label="Slot 2 (Base 370 - Mid Left)",
    ),
    3: SlotRegion(
        slot_id=3,
        x_min_m=0.00,
        x_max_m=0.08,
        y_min_m=0.05,
        y_max_m=0.15,
        verified=False,
        label="Slot 3 (Base 440 - Far Left)",
    ),
}


def evaluate_supervised_slot_pick(
    *,
    object_id: str,
    class_name: str,
    confidence: float,
    min_confidence: float = 0.12,
    workspace_coords: tuple[float, float] | None,
    workspace_bounds: tuple[float, float] = (0.3115, 0.1500),
    slot_regions: dict[int, SlotRegion] | None = None,
    multi_frame_history: Sequence[tuple[float, float]] | None = None,
    max_position_variance_m: float = 0.005,
    operator_confirmed: bool = False,
    mock_mode: bool = True,
    hardware_available: bool = False,
    motion_authorized: bool = False,
    command_id: int = 1,
) -> SupervisedSlotDecision:
    """Evaluate whether an object detection can trigger a supervised slot fetch."""
    # 1. Detection validity
    if not object_id or not class_name:
        return SupervisedSlotDecision(
            allowed=False,
            slot_id=None,
            reason="invalid detection: missing object_id or class_name",
        )
    if confidence < min_confidence:
        return SupervisedSlotDecision(
            allowed=False,
            slot_id=None,
            reason=f"confidence {confidence:.2f} below threshold {min_confidence:.2f}",
        )

    # 2. Workspace coordinate presence
    if workspace_coords is None:
        return SupervisedSlotDecision(
            allowed=False,
            slot_id=None,
            reason="missing or uncalibrated workspace coordinates",
        )
    x, y = workspace_coords
    if not (math.isfinite(x) and math.isfinite(y)):
        return SupervisedSlotDecision(
            allowed=False,
            slot_id=None,
            reason="non-finite workspace coordinates",
        )

    # 3. Overall board bounds (strictly un-clamped)
    w_max, h_max = workspace_bounds
    if not (0.0 <= x <= w_max and 0.0 <= y <= h_max):
        return SupervisedSlotDecision(
            allowed=False,
            slot_id=None,
            reason=f"coordinates ({x:.3f}, {y:.3f})m outside workspace bounds [0..{w_max}, 0..{h_max}]m",
        )

    # 4. Multi-frame stability
    if multi_frame_history is not None:
        if len(multi_frame_history) < 3:
            return SupervisedSlotDecision(
                allowed=False,
                slot_id=None,
                reason="insufficient history for stability check",
            )
        mean_x = sum(p[0] for p in multi_frame_history) / len(multi_frame_history)
        mean_y = sum(p[1] for p in multi_frame_history) / len(multi_frame_history)
        var = sum(
            (p[0] - mean_x) ** 2 + (p[1] - mean_y) ** 2
            for p in multi_frame_history
        ) / len(multi_frame_history)
        std_dev = math.sqrt(var)
        if std_dev > max_position_variance_m:
            return SupervisedSlotDecision(
                allowed=False,
                slot_id=None,
                reason=f"unstable detection: position std dev {std_dev*1000:.1f}mm > {max_position_variance_m*1000:.1f}mm",
            )

    # 5. Check against slot regions
    regions = slot_regions or DEFAULT_SLOT_REGIONS
    matching_slots = [s for s in regions.values() if s.contains(x, y)]
    if not matching_slots:
        return SupervisedSlotDecision(
            allowed=False,
            slot_id=None,
            reason=f"target at ({x*100:.1f}, {y*100:.1f})cm does not lie in any configured slot region",
        )
    if len(matching_slots) > 1:
        return SupervisedSlotDecision(
            allowed=False,
            slot_id=None,
            reason=f"ambiguous target: overlaps multiple slot regions {[s.slot_id for s in matching_slots]}",
        )
    target_slot = matching_slots[0]

    # 6. Physical slot verification gate
    if not target_slot.verified:
        return SupervisedSlotDecision(
            allowed=False,
            slot_id=target_slot.slot_id,
            reason=f"Slot {target_slot.slot_id} physical coordinates have not been verified; pickup disabled",
        )

    # 7. Operator confirmation gate
    if not operator_confirmed:
        return SupervisedSlotDecision(
            allowed=False,
            slot_id=target_slot.slot_id,
            reason=f"target confirmed in Slot {target_slot.slot_id}; awaiting explicit operator confirmation",
        )

    # 8. Hardware & safety authorization
    if not mock_mode:
        if not hardware_available:
            return SupervisedSlotDecision(
                allowed=False,
                slot_id=target_slot.slot_id,
                reason="hardware is unavailable",
            )
        if not motion_authorized:
            return SupervisedSlotDecision(
                allowed=False,
                slot_id=target_slot.slot_id,
                reason="real motion is not authorized",
            )

    payload = {"id": command_id, "command": "pick", "slot": target_slot.slot_id}
    sim_note = " (MOCK)" if mock_mode else ""
    return SupervisedSlotDecision(
        allowed=True,
        slot_id=target_slot.slot_id,
        reason=f"Supervised fetch authorized for Slot {target_slot.slot_id}{sim_note}",
        command_payload=payload,
    )
