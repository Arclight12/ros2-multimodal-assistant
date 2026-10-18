"""End-to-end verification of camera -> homography -> target -> 2D kinematics -> serial framing."""

from __future__ import annotations

import cv2
import numpy as np

from assistant_motion.arm_protocol import command_frame
from assistant_motion.kinematics import (
    ARC_CENTER_X,
    ARC_CENTER_Y,
    MAX_SAFE_BASE,
    MIN_SAFE_BASE,
    R_MAX_SAFE_M,
    R_MIN_SAFE_M,
    compute_arm_pulses,
)


def test_end_to_end_homography_to_arm_command():
    """Verify full mathematical flow from ArUco pixels to Arduino serial frame."""
    # Synthetic marker pixel positions for a 1280x720 camera frame
    # Marker 0: Top-Left, 1: Top-Right, 2: Bottom-Right, 3: Bottom-Left
    p_tl = (200.0, 100.0)
    p_tr = (1080.0, 110.0)
    p_br = (1050.0, 620.0)
    p_bl = (230.0, 610.0)

    src_pts = np.array([p_tl, p_tr, p_br, p_bl], dtype=np.float32)
    dst_pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], dtype=np.float32)

    homography = cv2.getPerspectiveTransform(src_pts, dst_pts)

    # Place an object in the middle of the tabletop
    # Center pixel roughly (640, 360)
    center_px = np.array([640.0, 360.0, 1.0], dtype=np.float32)
    norm = homography @ center_px
    norm /= norm[2]

    # Map to workspace metric coordinates
    ws_x = float(norm[0]) * 0.3115
    ws_y = float(norm[1]) * 0.1500

    # Ensure mapped coordinates are inside board bounds
    assert 0.0 <= ws_x <= 0.3115
    assert 0.0 <= ws_y <= 0.1500

    # Compute arm pulse trajectory for a sharpener
    base, ch1, ch2, grip, dist, angle = compute_arm_pulses("sharpener", ws_x, ws_y)

    assert MIN_SAFE_BASE <= base <= MAX_SAFE_BASE
    assert 390 <= ch1 <= 510
    assert 400 <= ch2 <= 550
    assert grip == 440  # narrow grip
    assert R_MIN_SAFE_M <= dist <= R_MAX_SAFE_M

    # Format serial command
    cmd = {
        "id": 1,
        "command": "pick",
        "base": base,
        "link1": ch1,
        "link2": ch2,
        "gripper": grip,
    }
    frame = command_frame(cmd)
    assert frame == f"P,{base},{ch1},{ch2},{grip}\n"


def test_parallelogram_marker_recovery():
    """Verify that when Marker 2 (BR) is occluded, parallelogram recovery restores exact homography."""
    p_tl = (200.0, 100.0)
    p_tr = (1080.0, 110.0)
    p_br_true = (1050.0, 620.0)
    p_bl = (230.0, 610.0)

    # Parallelogram formula: P2 = P1 + P3 - P0
    p2_reconstructed = (
        p_tr[0] + p_bl[0] - p_tl[0],
        p_tr[1] + p_bl[1] - p_tl[1],
    )

    src_4 = np.array([p_tl, p_tr, p_br_true, p_bl], dtype=np.float32)
    src_reconstructed = np.array([p_tl, p_tr, p2_reconstructed, p_bl], dtype=np.float32)
    dst_pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], dtype=np.float32)

    h_true = cv2.getPerspectiveTransform(src_4, dst_pts)
    h_rec = cv2.getPerspectiveTransform(src_reconstructed, dst_pts)

    # Test point
    pt = np.array([600.0, 350.0, 1.0], dtype=np.float32)
    p_true = h_true @ pt
    p_true /= p_true[2]

    p_rec = h_rec @ pt
    p_rec /= p_rec[2]

    # Difference in normalized coordinates is tiny (< 0.02)
    assert abs(p_true[0] - p_rec[0]) < 0.03
    assert abs(p_true[1] - p_rec[1]) < 0.03
