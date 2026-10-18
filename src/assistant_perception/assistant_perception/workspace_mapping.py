"""Pure camera-pixel to robot-base workspace mapping helpers."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence


try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - optional runtime dependency
    cv2 = None
    np = None

# Corner order associated with marker IDs (e.g. IDs [0, 1, 2, 3])
CORNER_ORDER = ("top_left", "top_right", "bottom_right", "bottom_left")

# Destination normalized points:
# Workspace origin (0.0, 0.0) is at the Bottom-Left marker center.
# Positive X extends to the right across the board.
# Positive Y extends toward the far/top end of the board (forward arm workspace).
DESTINATION_POINTS = (
    (0.0, 1.0),  # top-left: x=0, y=height
    (1.0, 1.0),  # top-right: x=width, y=height
    (1.0, 0.0),  # bottom-right: x=width, y=0
    (0.0, 0.0),  # bottom-left: x=0, y=0
)


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
    robot_transform_configured: bool = False

    @property
    def has_planar_calibration(self) -> bool:
        """Return whether camera-to-workspace 2D homography is calibrated and valid."""
        return self.calibrated and self.homography is not None

    @property
    def has_robot_transform(self) -> bool:
        """Return whether 3D robot transform (including table_z and base frame) is verified."""
        return self.calibrated and self.robot_transform_configured

    @property
    def is_valid(self) -> bool:
        """Return whether this calibration is safe to use for full robot poses.

        Requires both valid planar calibration and verified 3D robot transform.
        For planar camera-to-workspace queries, check `has_planar_calibration`.
        """
        return self.has_robot_transform


def object_contact_pixel(
    center_x: float,
    center_y: float,
    bbox_height: float,
) -> tuple[float, float]:
    """Estimate tabletop contact point from bounding-box center and height.

    This is a geometric heuristic approximation: for an object resting on a flat
    horizontal tabletop viewed by an overhead or angled camera, the bottom edge
    of the 2D bounding box (center_x, center_y + bbox_height / 2) approximates the
    ground-contact footpoint on the table plane where planar homography mapping is
    geometrically valid.

    Limitations:
    - This is an approximation that requires physical validation on real objects.
    - Shadows, bounding-box margins, non-flat object bases, and perspective effects
      on tall objects can shift the true contact point away from this estimate.
    - Visual gaze selection MUST continue to use visual image_center (centroid),
      while planar tabletop mapping targets this estimated contact footpoint.
    """
    if not (math.isfinite(center_x) and math.isfinite(center_y)):
        return (center_x, center_y)
    effective_h = max(0.0, bbox_height) if math.isfinite(bbox_height) else 0.0
    return (center_x, center_y + effective_h / 2.0)


def polygon_signed_area(pts: Sequence[Sequence[float]]) -> float:
    """Return 2D signed area using the shoelace formula."""
    n = len(pts)
    if n < 3:
        return 0.0
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += pts[i][0] * pts[j][1] - pts[j][0] * pts[i][1]
    return 0.5 * area


def find_marker_homography(
    marker_centers: Mapping[int, Sequence[float]],
    marker_ids: Sequence[int] = (0, 1, 2, 3),
) -> tuple[tuple[float, float, float], ...] | None:
    """Compute and validate pixel-to-normalized-workspace homography from marker centers.

    Associates marker_ids in order: top-left, top-right, bottom-right, bottom-left.
    Rejects missing markers, degenerate geometry, self-intersecting polygons,
    inverted windings, and ill-conditioned or singular homographies.

    Geometric Limitations:
    Geometric checks (convexity, winding, area, conditioning) reject non-convex,
    self-intersecting, or reversed-order configurations, but CANNOT distinguish
    cyclic permutations of marker IDs (e.g. [1, 2, 3, 0], [2, 3, 0, 1], [3, 0, 1, 2]).
    Cyclic permutations form valid convex polygons with positive clockwise winding,
    so they are mathematically accepted by this API, but yield a rotated workspace.
    Therefore, physical marker ID assignment (ID 0 = TL, ID 1 = TR, ID 2 = BR,
    ID 3 = BL) MUST be visually verified against the camera image before recording
    a calibration.
    """
    if cv2 is None or np is None:
        return None
    if len(marker_ids) != 4 or len(set(marker_ids)) != 4:
        return None
    if any(m_id not in marker_centers for m_id in marker_ids):
        return None
    try:
        source_pts = [
            (float(marker_centers[m_id][0]), float(marker_centers[m_id][1]))
            for m_id in marker_ids
        ]
    except (IndexError, TypeError, ValueError):
        return None
    if not all(math.isfinite(x) and math.isfinite(y) for x, y in source_pts):
        return None

    source = np.float32(source_pts)
    destination = np.float32(DESTINATION_POINTS)

    # Geometry checks: minimum area and convexity
    if cv2.contourArea(source) < 100.0:
        return None
    if not cv2.isContourConvex(np.int32(source)):
        return None

    # Winding check: in image coordinates (y down), TL -> TR -> BR -> BL
    # must have positive signed area in this shoelace orientation.
    signed_area = polygon_signed_area(source_pts)
    if signed_area <= 0.0:
        return None

    homography, _ = cv2.findHomography(source, destination)
    if homography is None or homography.shape != (3, 3):
        return None
    if not np.isfinite(homography).all():
        return None

    det = float(np.linalg.det(homography))
    if abs(det) < 1e-12:
        return None
    try:
        cond = float(np.linalg.cond(homography))
        if not math.isfinite(cond) or cond > 1e8:
            return None
    except np.linalg.LinAlgError:
        return None

    # Forward projection consistency check
    src_h = np.column_stack([source, np.ones(4)])
    dst_proj = (homography @ src_h.T).T
    if any(abs(w) < 1e-12 for w in dst_proj[:, 2]):
        return None
    dst_calc = dst_proj[:, :2] / dst_proj[:, 2:3]
    if not np.allclose(dst_calc, destination, atol=1e-3):
        return None

    return tuple(tuple(float(v) for v in row) for row in homography)


def workspace_contains(
    x_m: float,
    y_m: float,
    width_m: float,
    height_m: float,
    tolerance_m: float = 1e-6,
) -> bool:
    """Return whether a workspace-local point lies inside its rectangle."""
    return (
        all(math.isfinite(value) for value in (x_m, y_m, width_m, height_m))
        and width_m > 0.0
        and height_m > 0.0
        and -tolerance_m <= x_m <= width_m + tolerance_m
        and -tolerance_m <= y_m <= height_m + tolerance_m
    )


def map_pixel_to_base(
    pixel_x: float,
    pixel_y: float,
    calibration: WorkspaceCalibration,
) -> tuple[float, float, float] | None:
    """Map a phone pixel through normalized workspace coordinates to base pose.

    Requires both valid planar calibration and verified 3D robot transform
    (calibration.has_robot_transform == True). If the robot transform is unverified,
    returns None to ensure uncalibrated 3D poses cannot initiate motion.
    """
    if not calibration.has_robot_transform:
        return None
    workspace = map_pixel_to_workspace(pixel_x, pixel_y, calibration)
    if workspace is None:
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
    """Map a phone pixel to workspace-local metres.

    Only requires valid planar calibration (calibration.has_planar_calibration == True).
    Independent of whether the 3D robot transform is verified.
    """
    if not calibration.has_planar_calibration or not all(
        math.isfinite(value) for value in (pixel_x, pixel_y)
    ):
        return None
    try:
        row0, row1, row2 = calibration.homography
        denominator = row2[0] * pixel_x + row2[1] * pixel_y + row2[2]
        if not math.isfinite(denominator) or abs(denominator) < 1e-12:
            return None
        normalized_x = (row0[0] * pixel_x + row0[1] * pixel_y + row0[2]) / denominator
        normalized_y = (row1[0] * pixel_x + row1[1] * pixel_y + row1[2]) / denominator
    except (TypeError, ValueError):
        return None
    if not workspace_contains(normalized_x, normalized_y, 1.0, 1.0, tolerance_m=1e-5):
        return None
    clamped_x = min(max(normalized_x, 0.0), 1.0)
    clamped_y = min(max(normalized_y, 0.0), 1.0)
    return (
        clamped_x * calibration.workspace_width_m,
        clamped_y * calibration.workspace_height_m,
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
    det = (
        homography[0][0] * (homography[1][1] * homography[2][2] - homography[1][2] * homography[2][1])
        - homography[0][1] * (homography[1][0] * homography[2][2] - homography[1][2] * homography[2][0])
        + homography[0][2] * (homography[1][0] * homography[2][1] - homography[1][1] * homography[2][0])
    )
    if not math.isfinite(det) or abs(det) < 1e-12:
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
