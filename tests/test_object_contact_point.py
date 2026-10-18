"""Tests for object ground-contact point estimation and gaze/grasp coordinate separation.

Covers Phase 5E:
- Test planar object pixel against contact-point footpoint policy
- Keep gaze-selection image coordinates separate from grasp coordinates
"""

from __future__ import annotations

import pytest

from assistant_interaction.selection_logic import (
    DetectionObservation,
    GazeObservation,
    SelectionConfig,
    nearest_valid_detection,
)
from assistant_perception.workspace_mapping import (
    WorkspaceCalibration,
    find_marker_homography,
    map_pixel_to_base,
    map_pixel_to_workspace,
    object_contact_pixel,
)


def test_object_contact_pixel_estimation():
    """Verify ground-contact footpoint calculation from bounding box center and height."""
    # Cup with height 80 px: center=(300, 200), bottom footpoint=(300, 240)
    contact = object_contact_pixel(300.0, 200.0, 80.0)
    assert contact == (300.0, 240.0)

    # Flat object / zero height: footpoint equals center
    assert object_contact_pixel(300.0, 200.0, 0.0) == (300.0, 200.0)

    # Negative height or invalid inputs
    assert object_contact_pixel(300.0, 200.0, -10.0) == (300.0, 200.0)


def test_gaze_selection_uses_center_while_mapping_uses_contact():
    """Gaze selection must match visual bounding-box center, while grasping targets footpoint."""
    image_w, image_h = 1000.0, 1000.0

    # Detection: bounding-box center at (300, 200), height = 100 px (bottom at 250 px)
    detection = DetectionObservation(
        object_id="cup_0",
        class_name="cup",
        confidence=0.95,
        image_x=300.0,
        image_y=200.0,
        image_width=image_w,
        image_height=image_h,
    )

    # Gaze focused near visual center: (0.30, 0.20) normalized
    gaze_center = GazeObservation(x=0.30, y=0.20, confidence=0.90, valid=True)
    config = SelectionConfig()

    candidate = nearest_valid_detection(gaze_center, [detection], config)
    assert candidate is not None
    assert candidate.object_id == "cup_0"
    # Gaze distance to center is zero
    assert pytest.approx(0.0, abs=1e-4) == candidate.distance

    # Tabletop mapping uses footpoint (300.0, 250.0):
    contact_px = object_contact_pixel(detection.image_x, detection.image_y, 100.0)
    assert contact_px == (300.0, 250.0)

    # Create calibrated setup
    centers = {
        0: (100.0, 100.0),  # TL
        1: (500.0, 100.0),  # TR
        2: (500.0, 300.0),  # BR
        3: (100.0, 300.0),  # BL
    }
    h = find_marker_homography(centers, [0, 1, 2, 3])
    cal = WorkspaceCalibration(
        calibrated=True,
        homography=h,
        workspace_width_m=0.3115,
        workspace_height_m=0.1500,
        table_z_m=0.0,
        base_origin=(-0.150, 0.132, 0.0),
        base_yaw_rad=0.0,
        robot_transform_configured=True,
    )

    # Visual center mapping vs footpoint mapping:
    w_center = map_pixel_to_workspace(detection.image_x, detection.image_y, cal)
    w_contact = map_pixel_to_workspace(contact_px[0], contact_px[1], cal)

    assert w_center is not None
    assert w_contact is not None
    # Contact footpoint is closer to camera / nearer end of board, so workspace Y is smaller
    assert w_contact[1] < w_center[1]
    # Footpoint X matches center X (same vertical midline)
    assert pytest.approx(w_center[0], abs=1e-4) == w_contact[0]
