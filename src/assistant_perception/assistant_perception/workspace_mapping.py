"""Pure camera-pixel to robot-base workspace mapping helpers."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence


@dataclass(frozen=True)
class WorkspaceCalibration:
    """Validated planar calibration used by perception and motion checks."""

    calibrated: bool
    homography: tuple[tuple[float, float, float], ...]
    workspace_width_m: float
    workspace_height_m: float
    table_z_m: float
    base_origin: tuple[float, float, float]
    base_yaw_rad: float
    robot_transform_configured: bool = True

    @property
    def is_valid(self) -> bool:
        """Return whether this calibration is safe to use for robot poses."""
        return self.calibrated and self.robot_transform_configured


def workspace_contains(
    x_m: float, y_m: float, width_m: float, height_m: float
) -> bool:
    """Return whether a workspace-local point lies inside its rectangle."""
    return (
        all(math.isfinite(value) for value in (x_m, y_m, width_m, height_m))
        and width_m > 0.0
        and height_m > 0.0
        and 0.0 <= x_m <= width_m
        and 0.0 <= y_m <= height_m
    )


def map_pixel_to_base(
    pixel_x: float,
    pixel_y: float,
    calibration: WorkspaceCalibration,
) -> tuple[float, float, float] | None:
    """Map a phone pixel through normalized workspace coordinates to base pose.

    The calibration homography maps pixels to normalized workspace coordinates.
    The normalized point is then scaled into metres and rotated/translated into
    the robot base frame.
    """
    workspace = map_pixel_to_workspace(pixel_x, pixel_y, calibration)
    if workspace is None or not calibration.is_valid:
        return None
    workspace_x, workspace_y = workspace
    cosine = math.cos(calibration.base_yaw_rad)
    sine = math.sin(calibration.base_yaw_rad)
    base_x = calibration.base_origin[0] + cosine * workspace_x - sine * workspace_y
    base_y = calibration.base_origin[1] + sine * workspace_x + cosine * workspace_y
    base_z = calibration.base_origin[2] + calibration.table_z_m
    point = (base_x, base_y, base_z)
    return point if all(math.isfinite(value) for value in point) else None


def map_pixel_to_workspace(
    pixel_x: float,
    pixel_y: float,
    calibration: WorkspaceCalibration,
) -> tuple[float, float] | None:
    """Map a phone pixel to workspace-local metres."""
    if not calibration.is_valid or not all(
        math.isfinite(value) for value in (pixel_x, pixel_y)
    ):
        return None
    try:
        row0, row1, row2 = calibration.homography
        denominator = row2[0] * pixel_x + row2[1] * pixel_y + row2[2]
        if not math.isfinite(denominator) or abs(denominator) < 1e-12:
            return None
        normalized_x = (
            row0[0] * pixel_x + row0[1] * pixel_y + row0[2]
        ) / denominator
        normalized_y = (
            row1[0] * pixel_x + row1[1] * pixel_y + row1[2]
        ) / denominator
    except (TypeError, ValueError):
        return None
    if not workspace_contains(normalized_x, normalized_y, 1.0, 1.0):
        return None
    return (
        normalized_x * calibration.workspace_width_m,
        normalized_y * calibration.workspace_height_m,
    )


def _matrix(value: Any) -> tuple[tuple[float, float, float], ...] | None:
    """Convert a JSON matrix to a finite 3x3 tuple."""
    if not isinstance(value, Sequence) or len(value) != 3:
        return None
    rows = []
    for row in value:
        if not isinstance(row, Sequence) or len(row) != 3:
            return None
        try:
            numbers = tuple(float(item) for item in row)
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(item) for item in numbers):
            return None
        rows.append(numbers)
    return tuple(rows)


def calibration_from_document(
    document: Mapping[str, Any],
) -> WorkspaceCalibration | None:
    """Parse and validate a persisted calibration document."""
    if not isinstance(document, Mapping) or not document.get("calibrated", False):
        return None
    homography = _matrix(document.get("homography"))
    transform = document.get("workspace_to_base")
    if homography is None or not isinstance(transform, Mapping):
        return None
    try:
        width = float(document["workspace_width_m"])
        height = float(document["workspace_height_m"])
        table_z = float(document["table_z_m"])
        origin = (
            float(transform["x_m"]),
            float(transform["y_m"]),
            float(transform["z_m"]),
        )
        yaw = float(transform["yaw_rad"])
    except (KeyError, TypeError, ValueError):
        return None
    values = (width, height, table_z, *origin, yaw)
    if (
        not all(math.isfinite(value) for value in values)
        or width <= 0.0
        or height <= 0.0
    ):
        return None
    return WorkspaceCalibration(
        calibrated=True,
        homography=homography,
        workspace_width_m=width,
        workspace_height_m=height,
        table_z_m=table_z,
        base_origin=origin,
        base_yaw_rad=yaw,
        robot_transform_configured=bool(
            document.get("robot_transform_configured", False)
        ),
    )


def load_calibration(path: str | Path) -> WorkspaceCalibration | None:
    """Load a calibration file, returning ``None`` for any invalid input."""
    try:
        document = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return calibration_from_document(document)


def save_calibration(path: str | Path, calibration: WorkspaceCalibration) -> None:
    """Persist calibration data to a caller-selected runtime path."""
    document = {
        "calibrated": calibration.calibrated,
        "robot_transform_configured": calibration.robot_transform_configured,
        "homography": calibration.homography,
        "workspace_width_m": calibration.workspace_width_m,
        "workspace_height_m": calibration.workspace_height_m,
        "table_z_m": calibration.table_z_m,
        "workspace_to_base": {
            "x_m": calibration.base_origin[0],
            "y_m": calibration.base_origin[1],
            "z_m": calibration.base_origin[2],
            "yaw_rad": calibration.base_yaw_rad,
        },
    }
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(document, indent=2), encoding="utf-8")
