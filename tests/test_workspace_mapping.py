import json

from assistant_perception.workspace_mapping import (
    WorkspaceCalibration,
    load_calibration,
    map_pixel_to_base,
    workspace_contains,
)


def valid_calibration() -> WorkspaceCalibration:
    return WorkspaceCalibration(
        calibrated=True,
        homography=((0.001, 0.0, 0.0), (0.0, 0.001, 0.0), (0.0, 0.0, 1.0)),
        workspace_width_m=0.4,
        workspace_height_m=0.3,
        table_z_m=0.02,
        base_origin=(0.1, -0.2, 0.4),
        base_yaw_rad=0.0,
    )


def test_pixel_maps_through_normalized_workspace_to_base():
    point = map_pixel_to_base(500.0, 250.0, valid_calibration())

    assert point == (0.30000000000000004, -0.125, 0.42000000000000004)


def test_uncalibrated_mapping_returns_no_robot_point():
    calibration = valid_calibration()
    invalid = WorkspaceCalibration(
        calibrated=False,
        homography=calibration.homography,
        workspace_width_m=calibration.workspace_width_m,
        workspace_height_m=calibration.workspace_height_m,
        table_z_m=calibration.table_z_m,
        base_origin=calibration.base_origin,
        base_yaw_rad=calibration.base_yaw_rad,
    )

    assert map_pixel_to_base(500.0, 250.0, invalid) is None


def test_workspace_bounds_are_inclusive():
    assert workspace_contains(0.0, 0.3, 0.4, 0.3)
    assert not workspace_contains(-0.001, 0.1, 0.4, 0.3)
    assert not workspace_contains(0.2, 0.301, 0.4, 0.3)


def test_malformed_calibration_file_is_rejected(tmp_path):
    path = tmp_path / "calibration.json"
    path.write_text(json.dumps({"calibrated": True}), encoding="utf-8")

    assert load_calibration(path) is None
