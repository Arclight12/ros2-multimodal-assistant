"""Tests for configuration consistency across YAML, geometry, and implementation.

Covers Phase 5C:
- workspace.yaml dimensions agree with the measured marker-centre geometry
- Coordinate units are consistent in metres (not cm)
- Marker IDs and dictionary match implementation
- Transform cannot be considered valid without explicit configuration
"""

from __future__ import annotations

from pathlib import Path
import yaml
import pytest

from assistant_perception.workspace_mapping import (
    CORNER_ORDER,
    DESTINATION_POINTS,
    WorkspaceCalibration,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
WORKSPACE_YAML = REPO_ROOT / "src" / "assistant_bringup" / "config" / "workspace.yaml"
PERCEPTION_YAML = REPO_ROOT / "src" / "assistant_bringup" / "config" / "perception.yaml"


def test_workspace_yaml_dimensions_and_units():
    """Verify workspace.yaml dimensions match marker geometry in metres."""
    assert WORKSPACE_YAML.is_file(), f"Missing config file {WORKSPACE_YAML}"
    data = yaml.safe_load(WORKSPACE_YAML.read_text(encoding="utf-8"))

    # All three nodes must agree on dimensions
    for node_name in ("workspace_mapper_node", "calibration_node", "motion_planner_node"):
        params = data[node_name]["ros__parameters"]
        width = params["workspace_width_m"]
        height = params["workspace_height_m"]

        # Expected: width=0.3115 m, height=0.1500 m
        assert pytest.approx(0.3115, abs=1e-4) == width, (
            f"{node_name} width is {width}, expected 0.3115 m (not 0.4 m or cm)"
        )
        assert pytest.approx(0.1500, abs=1e-4) == height, (
            f"{node_name} height is {height}, expected 0.1500 m (not 0.3 m or cm)"
        )

        # Unit verification: values must be < 1.0 m (if in cm, they would be 31.15 / 15.0)
        assert 0.1 < width < 1.0, f"{node_name} width {width} appears to be in wrong units"
        assert 0.05 < height < 1.0, f"{node_name} height {height} appears to be in wrong units"

    calib_params = data["calibration_node"]["ros__parameters"]
    ox = calib_params["base_origin_x_m"]
    oy = calib_params["base_origin_y_m"]
    oz = calib_params["base_origin_z_m"]
    yaw = calib_params["base_yaw_rad"]

    # Base origin in metres (measured Bottom-Left marker center: -15.0 cm, 13.2 cm)
    assert pytest.approx(-0.150, abs=1e-4) == ox
    assert pytest.approx(0.132, abs=1e-4) == oy
    assert pytest.approx(0.0, abs=1e-4) == oz
    assert pytest.approx(0.0, abs=1e-4) == yaw
    # 3D robot transform must remain False in workspace.yaml until table_z is physically verified
    assert calib_params["robot_transform_configured"] is False


def test_perception_yaml_marker_configuration():
    """Verify perception.yaml defines the expected ArUco dictionary and marker IDs."""
    assert PERCEPTION_YAML.is_file(), f"Missing config file {PERCEPTION_YAML}"
    data = yaml.safe_load(PERCEPTION_YAML.read_text(encoding="utf-8"))

    calib_params = data["calibration_node"]["ros__parameters"]
    assert calib_params["aruco_dictionary"] == "DICT_4X4_50"
    assert calib_params["marker_ids"] == [0, 1, 2, 3]
    assert len(calib_params["marker_ids"]) == len(CORNER_ORDER)


def test_transform_requires_explicit_configuration():
    """Verify planar calibration can be tested independently while robot poses require explicit transform."""
    dummy_h = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

    # Default robot_transform_configured should be False
    calib_default = WorkspaceCalibration(
        calibrated=True,
        homography=dummy_h,
        workspace_width_m=0.3115,
        workspace_height_m=0.1500,
        table_z_m=0.0,
        base_origin=(-0.150, 0.132, 0.0),
        base_yaw_rad=0.0,
    )
    assert not calib_default.robot_transform_configured
    assert calib_default.has_planar_calibration
    assert not calib_default.has_robot_transform
    assert not calib_default.is_valid

    # When robot_transform_configured is explicitly False:
    # Planar mapping works, but robot-frame mapping returns None!
    from assistant_perception.workspace_mapping import map_pixel_to_base, map_pixel_to_workspace

    calib_unconfigured = WorkspaceCalibration(
        calibrated=True,
        homography=dummy_h,
        workspace_width_m=0.3115,
        workspace_height_m=0.1500,
        table_z_m=0.0,
        base_origin=(-0.150, 0.132, 0.0),
        base_yaw_rad=0.0,
        robot_transform_configured=False,
    )
    assert calib_unconfigured.has_planar_calibration
    assert not calib_unconfigured.has_robot_transform
    assert not calib_unconfigured.is_valid

    # Inside normalized range [0..1] x [0..1] with identity homography: pixel (0.5, 0.5)
    w_pt = map_pixel_to_workspace(0.5, 0.5, calib_unconfigured)
    assert w_pt is not None
    assert pytest.approx(0.5 * 0.3115, abs=1e-4) == w_pt[0]
    assert pytest.approx(0.5 * 0.1500, abs=1e-4) == w_pt[1]

    # But robot-base mapping MUST be refused because transform is unverified
    assert map_pixel_to_base(0.5, 0.5, calib_unconfigured) is None

    # When calibrated is False: neither planar nor robot mapping works
    calib_not_calibrated = WorkspaceCalibration(
        calibrated=False,
        homography=dummy_h,
        workspace_width_m=0.3115,
        workspace_height_m=0.1500,
        table_z_m=0.0,
        base_origin=(-0.150, 0.132, 0.0),
        base_yaw_rad=0.0,
        robot_transform_configured=True,
    )
    assert not calib_not_calibrated.has_planar_calibration
    assert not calib_not_calibrated.has_robot_transform
    assert not calib_not_calibrated.is_valid
    assert map_pixel_to_workspace(0.5, 0.5, calib_not_calibrated) is None
    assert map_pixel_to_base(0.5, 0.5, calib_not_calibrated) is None

    # Only when both calibrated AND robot_transform_configured are True is robot transform valid
    calib_valid = WorkspaceCalibration(
        calibrated=True,
        homography=dummy_h,
        workspace_width_m=0.3115,
        workspace_height_m=0.1500,
        table_z_m=0.0,
        base_origin=(-0.150, 0.132, 0.0),
        base_yaw_rad=0.0,
        robot_transform_configured=True,
    )
    assert calib_valid.has_planar_calibration
    assert calib_valid.has_robot_transform
    assert calib_valid.is_valid
    assert map_pixel_to_base(0.5, 0.5, calib_valid) is not None
