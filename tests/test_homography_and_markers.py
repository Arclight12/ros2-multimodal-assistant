"""Tests for ArUco marker homography, corner ordering, and geometry validation.

Covers Phase 5A:
- Correctly ordered four-marker fixture
- Expected marker ID-to-corner association
- Perspective-distorted image fixture
- Synthetic ArUco marker rendering and detection
- Missing markers
- Degenerate / invalid geometry
- ID-ordering mistakes
"""

from __future__ import annotations

import cv2
import cv2.aruco as aruco
import numpy as np
import pytest

from assistant_perception.workspace_mapping import (
    CORNER_ORDER,
    DESTINATION_POINTS,
    find_marker_homography,
)


def _project_points(h_matrix: tuple[tuple[float, float, float], ...], pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    h = np.array(h_matrix, dtype=np.float64)
    src_h = np.column_stack([pts, np.ones(len(pts))])
    proj = (h @ src_h.T).T
    calc = proj[:, :2] / proj[:, 2:3]
    return [(float(x), float(y)) for x, y in calc]


def test_corner_order_and_destination_points():
    """Verify corner association order and normalized coordinates."""
    assert CORNER_ORDER == ("top_left", "top_right", "bottom_right", "bottom_left")
    # Top-Left: (x=0, y=height -> norm y=1.0)
    assert DESTINATION_POINTS[0] == (0.0, 1.0)
    # Top-Right: (x=width, y=height -> norm x=1.0, y=1.0)
    assert DESTINATION_POINTS[1] == (1.0, 1.0)
    # Bottom-Right: (x=width, y=0 -> norm x=1.0, y=0.0)
    assert DESTINATION_POINTS[2] == (1.0, 0.0)
    # Bottom-Left: (x=0, y=0 -> norm x=0.0, y=0.0)
    assert DESTINATION_POINTS[3] == (0.0, 0.0)


def test_standard_four_marker_fixture():
    """Standard rectangular fixture with IDs [0, 1, 2, 3]."""
    # Camera image with Y increasing downwards:
    # Top edge has lower image Y, bottom edge has higher image Y.
    centers = {
        0: (100.0, 100.0),  # top-left
        1: (500.0, 100.0),  # top-right
        2: (500.0, 300.0),  # bottom-right
        3: (100.0, 300.0),  # bottom-left
    }
    h = find_marker_homography(centers, [0, 1, 2, 3])
    assert h is not None

    mapped = _project_points(h, [centers[i] for i in [0, 1, 2, 3]])
    for mapped_pt, expected in zip(mapped, DESTINATION_POINTS):
        assert pytest.approx(expected[0], abs=1e-3) == mapped_pt[0]
        assert pytest.approx(expected[1], abs=1e-3) == mapped_pt[1]


def test_perspective_distorted_fixture():
    """Trapezoidally foreshortened fixture from an angled camera."""
    # Top edge is further away, so appears narrower in the image
    centers = {
        0: (200.0, 120.0),  # top-left (narrower top)
        1: (450.0, 110.0),  # top-right
        2: (560.0, 380.0),  # bottom-right (wider bottom)
        3: (110.0, 370.0),  # bottom-left
    }
    h = find_marker_homography(centers, [0, 1, 2, 3])
    assert h is not None

    mapped = _project_points(h, [centers[i] for i in [0, 1, 2, 3]])
    for mapped_pt, expected in zip(mapped, DESTINATION_POINTS):
        assert pytest.approx(expected[0], abs=1e-3) == mapped_pt[0]
        assert pytest.approx(expected[1], abs=1e-3) == mapped_pt[1]


def test_synthetic_aruco_rendering_and_detection():
    """Render real ArUco markers into an image, detect them, and verify homography."""
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    image = np.ones((600, 800, 3), dtype=np.uint8) * 255

    placed_centers = {
        0: (150, 120),  # TL
        1: (650, 120),  # TR
        2: (650, 450),  # BR
        3: (150, 450),  # BL
    }
    marker_size = 70
    for mid, (cx, cy) in placed_centers.items():
        if hasattr(aruco, "generateImageMarker"):
            m_img = aruco.generateImageMarker(dictionary, mid, marker_size)
        else:
            m_img = aruco.drawMarker(dictionary, mid, marker_size)
        if len(m_img.shape) == 2:
            m_img = cv2.cvtColor(m_img, cv2.COLOR_GRAY2BGR)
        x1, y1 = cx - marker_size // 2, cy - marker_size // 2
        image[y1 : y1 + marker_size, x1 : x1 + marker_size] = m_img

    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(image)
    else:
        corners, ids, _ = aruco.detectMarkers(image, dictionary)

    assert ids is not None
    assert len(ids) == 4

    detected_centers = {}
    for m_corners, m_id in zip(corners, ids.flatten()):
        detected_centers[int(m_id)] = tuple(m_corners[0].mean(axis=0))

    h = find_marker_homography(detected_centers, [0, 1, 2, 3])
    assert h is not None

    # Mapped detected centers should hit target corners within half a pixel of error
    mapped = _project_points(h, [detected_centers[i] for i in [0, 1, 2, 3]])
    for mapped_pt, expected in zip(mapped, DESTINATION_POINTS):
        assert pytest.approx(expected[0], abs=0.01) == mapped_pt[0]
        assert pytest.approx(expected[1], abs=0.01) == mapped_pt[1]


def test_missing_marker_rejection():
    """Verify that omitting any one of the 4 markers refuses calibration."""
    full_centers = {
        0: (100.0, 100.0),
        1: (500.0, 100.0),
        2: (500.0, 300.0),
        3: (100.0, 300.0),
    }
    for missing_id in (0, 1, 2, 3):
        subset = {k: v for k, v in full_centers.items() if k != missing_id}
        assert find_marker_homography(subset, [0, 1, 2, 3]) is None


def test_degenerate_geometry_rejection():
    """Collinear points, insufficient area, or non-finite inputs must be rejected."""
    # 4 points on a line
    collinear = {
        0: (100.0, 100.0),
        1: (200.0, 100.0),
        2: (300.0, 100.0),
        3: (400.0, 100.0),
    }
    assert find_marker_homography(collinear, [0, 1, 2, 3]) is None

    # Tiny area (< 100 px^2)
    tiny = {
        0: (100.0, 100.0),
        1: (102.0, 100.0),
        2: (102.0, 102.0),
        3: (100.0, 102.0),
    }
    assert find_marker_homography(tiny, [0, 1, 2, 3]) is None

    # Non-finite values
    nan_pts = {
        0: (float("nan"), 100.0),
        1: (500.0, 100.0),
        2: (500.0, 300.0),
        3: (100.0, 300.0),
    }
    assert find_marker_homography(nan_pts, [0, 1, 2, 3]) is None


def test_id_ordering_mistakes_rejected():
    """Permuted IDs with broken geometry (crossing diagonals or inverted windings) are rejected."""
    centers = {
        0: (100.0, 100.0),  # TL
        1: (500.0, 100.0),  # TR
        2: (500.0, 300.0),  # BR
        3: (100.0, 300.0),  # BL
    }
    # Crossed order: [0, 2, 1, 3] forms a self-intersecting bowtie polygon (non-convex)
    assert find_marker_homography(centers, [0, 2, 1, 3]) is None

    # Swapped adjacent corners: [1, 0, 2, 3] (swapped TL and TR -> inverted winding)
    assert find_marker_homography(centers, [1, 0, 2, 3]) is None

    # Inverted winding: [3, 2, 1, 0] (counter-clockwise instead of clockwise in image coords)
    assert find_marker_homography(centers, [3, 2, 1, 0]) is None

    # Duplicate IDs: [0, 1, 2, 0]
    assert find_marker_homography(centers, [0, 1, 2, 0]) is None


def test_cyclic_marker_id_permutations_behavior_and_limitations():
    """Document and test that cyclic ID permutations are accepted by 2D geometry checks.

    CRITICAL LIMITATION OF 2D HOMOGRAPHY GEOMETRY CHECKS:
    A cyclic shift of 4 vertices (e.g. [1, 2, 3, 0], [2, 3, 0, 1], [3, 0, 1, 2]):
    - Preserves polygon convexity (isContourConvex remains True).
    - Preserves clockwise winding (polygon_signed_area remains positive).
    - Preserves non-zero area and well-conditioned matrix math.

    Consequently, pure 2D geometric checks CANNOT determine whether Marker 0 is
    actually at Top-Left or rotated to another corner. The current API mathematically
    ACCEPTS cyclic permutations, but maps the workspace rotated by 90, 180, or 270 degrees.

    Therefore:
    1. A convex polygon and valid homography DO NOT prove that physical marker IDs
       correspond to the correct physical board corners.
    2. The operator MUST visually verify against a real camera frame that:
       - Marker ID 0 is physically at Top-Left
       - Marker ID 1 is physically at Top-Right
       - Marker ID 2 is physically at Bottom-Right
       - Marker ID 3 is physically at Bottom-Left
       before recording a physical calibration.
    """
    centers = {
        0: (100.0, 100.0),  # physical TL
        1: (500.0, 100.0),  # physical TR
        2: (500.0, 300.0),  # physical BR
        3: (100.0, 300.0),  # physical BL
    }

    # Cyclic shift 1: [1, 2, 3, 0] (90 deg CCW rotation of ID sequence)
    # The API accepts this because TR->BR->BL->TL is still a convex clockwise quadrilateral
    h_rot90 = find_marker_homography(centers, [1, 2, 3, 0])
    assert h_rot90 is not None, "Cyclic permutation [1, 2, 3, 0] is geometrically valid and accepted"
    # But physical TR (marker 1) was erroneously mapped to normalized TL (0.0, 1.0)!
    proj_tr = _project_points(h_rot90, [centers[1]])[0]
    assert pytest.approx(0.0, abs=1e-3) == proj_tr[0]
    assert pytest.approx(1.0, abs=1e-3) == proj_tr[1]

    # Cyclic shift 2: [2, 3, 0, 1] (180 deg rotation)
    h_rot180 = find_marker_homography(centers, [2, 3, 0, 1])
    assert h_rot180 is not None, "Cyclic permutation [2, 3, 0, 1] is geometrically valid and accepted"
    # Physical BR (marker 2) was erroneously mapped to normalized TL (0.0, 1.0)!
    proj_br = _project_points(h_rot180, [centers[2]])[0]
    assert pytest.approx(0.0, abs=1e-3) == proj_br[0]
    assert pytest.approx(1.0, abs=1e-3) == proj_br[1]

    # Cyclic shift 3: [3, 0, 1, 2] (270 deg rotation)
    h_rot270 = find_marker_homography(centers, [3, 0, 1, 2])
    assert h_rot270 is not None, "Cyclic permutation [3, 0, 1, 2] is geometrically valid and accepted"
    # Physical BL (marker 3) was erroneously mapped to normalized TL (0.0, 1.0)!
    proj_bl = _project_points(h_rot270, [centers[3]])[0]
    assert pytest.approx(0.0, abs=1e-3) == proj_bl[0]
    assert pytest.approx(1.0, abs=1e-3) == proj_bl[1]
