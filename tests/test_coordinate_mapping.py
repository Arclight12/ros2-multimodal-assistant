"""Tests for workspace-to-robot coordinate mapping and numerical accuracy.

Covers Phase 5B:
- Four synthetic image marker centres map to their expected workspace positions
- Four marker centres map back to measured robot-base coordinates within tolerance
- X and Y signs are correct
- Far/top versus near/bottom orientation is correct
- Points inside workspace map to finite positions
- Points outside calibrated rectangle are rejected
- NaN, infinity, singular homographies, and malformed calibration JSON are rejected
"""

from __future__ import annotations

import json
import math
import pytest

from assistant_perception.workspace_mapping import (
    WorkspaceCalibration,
    calibration_from_document,
    find_marker_homography,
    map_pixel_to_base,
    map_pixel_to_workspace,
    workspace_contains,
)

# Measured physical coordinates in metres:
# TL: (-0.150, 0.282)
# TR: ( 0.163, 0.283)
# BR: ( 0.160, 0.133)
# BL: (-0.150, 0.132)
MEASURED_ROBOT_COORDS = {
    0: (-0.150, 0.282, 0.0),  # Top-Left
    1: (0.163, 0.283, 0.0),   # Top-Right
    2: (0.160, 0.133, 0.0),   # Bottom-Right
    3: (-0.150, 0.132, 0.0),  # Bottom-Left
}

WORKSPACE_WIDTH_M = 0.3115
WORKSPACE_HEIGHT_M = 0.1500
BASE_ORIGIN = (-0.150, 0.132, 0.0)
BASE_YAW_RAD = 0.0


@pytest.fixture
def calibrated_setup():
    """Create a calibrated WorkspaceCalibration fixture."""
    # Synthetic pixel positions:
    centers = {
        0: (100.0, 100.0),  # TL
        1: (500.0, 100.0),  # TR
        2: (500.0, 300.0),  # BR
        3: (100.0, 300.0),  # BL
    }
    h = find_marker_homography(centers, [0, 1, 2, 3])
    assert h is not None

    calibration = WorkspaceCalibration(
        calibrated=True,
        homography=h,
        workspace_width_m=WORKSPACE_WIDTH_M,
        workspace_height_m=WORKSPACE_HEIGHT_M,
        table_z_m=0.0,
        base_origin=BASE_ORIGIN,
        base_yaw_rad=BASE_YAW_RAD,
        robot_transform_configured=True,
    )
    return centers, calibration


def test_four_markers_map_to_expected_workspace_positions(calibrated_setup):
    """Marker centres must map to (0, h), (w, h), (w, 0), (0, 0) in workspace."""
    centers, calibration = calibrated_setup

    expected_workspace = {
        0: (0.0, WORKSPACE_HEIGHT_M),               # TL
        1: (WORKSPACE_WIDTH_M, WORKSPACE_HEIGHT_M), # TR
        2: (WORKSPACE_WIDTH_M, 0.0),                # BR
        3: (0.0, 0.0),                              # BL (origin)
    }

    for mid, px in centers.items():
        w_pt = map_pixel_to_workspace(px[0], px[1], calibration)
        assert w_pt is not None
        exp = expected_workspace[mid]
        assert pytest.approx(exp[0], abs=1e-4) == w_pt[0]
        assert pytest.approx(exp[1], abs=1e-4) == w_pt[1]


def test_four_markers_map_to_measured_robot_base_coordinates(calibrated_setup):
    """Synthetic math validation: idealized rectangle maps back to measured coords within 2.0 mm.

    CRITICAL NOTE ON NUMERICAL ACCURACY:
    - This is a synthetic mathematical check, NOT a measured camera-localization error.
    - The ~1.80 mm residuals at TR and BR arise purely from modeling the slightly non-rectangular
      measured marker positions (top width 0.3130 m, bottom width 0.3100 m) as an idealized
      rigid rectangle (0.3115 m x 0.1500 m).
    - Physical camera calibration accuracy has NOT yet been tested on physical hardware.
    """
    centers, calibration = calibrated_setup

    tolerances_mm = {
        0: 0.1,  # TL: exact match (0.0 mm)
        1: 2.0,  # TR: ~1.80 mm residual due to approximating measured quadrilateral as rigid rectangle
        2: 2.0,  # BR: ~1.80 mm residual due to approximating measured quadrilateral as rigid rectangle
        3: 0.1,  # BL: exact match (0.0 mm, origin)
    }

    errors = {}
    for mid, px in centers.items():
        robot_pt = map_pixel_to_base(px[0], px[1], calibration)
        assert robot_pt is not None
        meas = MEASURED_ROBOT_COORDS[mid]

        dx = robot_pt[0] - meas[0]
        dy = robot_pt[1] - meas[1]
        dz = robot_pt[2] - meas[2]
        euclidean_err_mm = math.sqrt(dx * dx + dy * dy + dz * dz) * 1000.0
        errors[mid] = euclidean_err_mm

        assert euclidean_err_mm <= tolerances_mm[mid], (
            f"Marker {mid} mapping error {euclidean_err_mm:.2f} mm exceeded "
            f"tolerance {tolerances_mm[mid]} mm (dx={dx*1000:.2f}mm, dy={dy*1000:.2f}mm)"
        )

    # Document the exact residual errors
    print(f"\nSynthetic model corner residuals: TL={errors[0]:.2f}mm, TR={errors[1]:.2f}mm, "
          f"BR={errors[2]:.2f}mm, BL={errors[3]:.2f}mm")


def test_planar_mapping_independent_of_robot_transform(calibrated_setup):
    """Planar camera-to-workspace mapping succeeds even when robot transform is unverified."""
    centers, full_calibration = calibrated_setup

    # Create planar-only calibration with robot_transform_configured=False
    planar_only_cal = WorkspaceCalibration(
        calibrated=True,
        homography=full_calibration.homography,
        workspace_width_m=full_calibration.workspace_width_m,
        workspace_height_m=full_calibration.workspace_height_m,
        table_z_m=0.0,
        base_origin=BASE_ORIGIN,
        base_yaw_rad=BASE_YAW_RAD,
        robot_transform_configured=False,
    )

    assert planar_only_cal.has_planar_calibration
    assert not planar_only_cal.has_robot_transform
    assert not planar_only_cal.is_valid

    # Planar mapping works for all markers
    for mid, px in centers.items():
        w_pt = map_pixel_to_workspace(px[0], px[1], planar_only_cal)
        assert w_pt is not None
        assert 0.0 <= w_pt[0] <= WORKSPACE_WIDTH_M + 1e-4
        assert 0.0 <= w_pt[1] <= WORKSPACE_HEIGHT_M + 1e-4

        # Robot-frame base coordinates MUST return None because transform is unverified
        assert map_pixel_to_base(px[0], px[1], planar_only_cal) is None


def test_coordinate_signs_and_orientation(calibrated_setup):
    """Verify X/Y signs and top/far vs bottom/near orientation."""
    centers, calibration = calibrated_setup

    bl_base = map_pixel_to_base(centers[3][0], centers[3][1], calibration)
    br_base = map_pixel_to_base(centers[2][0], centers[2][1], calibration)
    tl_base = map_pixel_to_base(centers[0][0], centers[0][1], calibration)
    tr_base = map_pixel_to_base(centers[1][0], centers[1][1], calibration)

    # 1. Moving left to right across the board: X must increase
    assert br_base[0] > bl_base[0]
    assert tr_base[0] > tl_base[0]
    # Left markers are at negative X, right markers are at positive X
    assert bl_base[0] < 0.0
    assert br_base[0] > 0.0

    # 2. Moving near to far along the board: Y must increase
    assert tl_base[1] > bl_base[1]
    assert tr_base[1] > br_base[1]
    # Near markers have smaller positive Y, far markers have larger positive Y
    assert bl_base[1] > 0.0
    assert tl_base[1] > bl_base[1]


def test_points_inside_workspace_map_to_finite_positions(calibrated_setup):
    """Any point strictly inside the marker polygon maps to a valid finite coordinate."""
    _, calibration = calibrated_setup

    # Center of synthetic image marker rectangle: (300, 200)
    w_pt = map_pixel_to_workspace(300.0, 200.0, calibration)
    assert w_pt is not None
    assert 0.0 < w_pt[0] < WORKSPACE_WIDTH_M
    assert 0.0 < w_pt[1] < WORKSPACE_HEIGHT_M

    base_pt = map_pixel_to_base(300.0, 200.0, calibration)
    assert base_pt is not None
    assert all(math.isfinite(v) for v in base_pt)
    # Midpoint X should be near 0.0575 m (-0.150 + 0.3115 / 2 = 0.00575)
    assert pytest.approx(-0.150 + WORKSPACE_WIDTH_M / 2.0, abs=1e-3) == base_pt[0]
    # Midpoint Y should be near 0.207 m (0.132 + 0.150 / 2 = 0.207)
    assert pytest.approx(0.132 + WORKSPACE_HEIGHT_M / 2.0, abs=1e-3) == base_pt[1]


def test_points_outside_calibrated_rectangle_rejected(calibrated_setup):
    """Points outside the marker rectangle must return None."""
    _, calibration = calibrated_setup

    # Far to the left: x = 50.0 (< 100.0)
    assert map_pixel_to_workspace(50.0, 200.0, calibration) is None
    assert map_pixel_to_base(50.0, 200.0, calibration) is None

    # Far to the right: x = 550.0 (> 500.0)
    assert map_pixel_to_workspace(550.0, 200.0, calibration) is None
    assert map_pixel_to_base(550.0, 200.0, calibration) is None

    # Far above (beyond top edge in image): y = 50.0 (< 100.0)
    assert map_pixel_to_workspace(300.0, 50.0, calibration) is None
    assert map_pixel_to_base(300.0, 50.0, calibration) is None

    # Far below (closer than bottom edge in image): y = 350.0 (> 300.0)
    assert map_pixel_to_workspace(300.0, 350.0, calibration) is None
    assert map_pixel_to_base(300.0, 350.0, calibration) is None


def test_nan_infinity_and_singular_inputs_rejected(calibrated_setup):
    """Non-finite coordinates or singular matrices must return None."""
    _, calibration = calibrated_setup

    assert map_pixel_to_workspace(float("nan"), 200.0, calibration) is None
    assert map_pixel_to_workspace(300.0, float("inf"), calibration) is None
    assert map_pixel_to_base(float("nan"), 200.0, calibration) is None
    assert map_pixel_to_base(300.0, float("-inf"), calibration) is None


def test_malformed_and_singular_calibration_documents_rejected():
    """Verify calibration_from_document rejects invalid JSON or singular homographies."""
    # Singular homography (all zeros)
    singular_doc = {
        "calibrated": True,
        "robot_transform_configured": True,
        "homography": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        "workspace_width_m": 0.3115,
        "workspace_height_m": 0.1500,
        "table_z_m": 0.0,
        "workspace_to_base": {"x_m": -0.150, "y_m": 0.132, "z_m": 0.0, "yaw_rad": 0.0},
    }
    assert calibration_from_document(singular_doc) is None

    # Malformed non-numeric matrix
    malformed_matrix_doc = dict(singular_doc)
    malformed_matrix_doc["homography"] = [["a", "b", "c"], [1, 2, 3], [4, 5, 6]]
    assert calibration_from_document(malformed_matrix_doc) is None

    # Missing width/height
    missing_dim_doc = dict(singular_doc)
    missing_dim_doc["workspace_width_m"] = -1.0
    assert calibration_from_document(missing_dim_doc) is None

    # Not calibrated flag
    uncalibrated_doc = dict(singular_doc)
    uncalibrated_doc["calibrated"] = False
    assert calibration_from_document(uncalibrated_doc) is None
