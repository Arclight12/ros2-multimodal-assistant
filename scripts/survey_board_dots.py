#!/usr/bin/env python3
"""Survey and calibrate the physical 4 slot dots on the board.

Detects the 4 corner ArUco markers, rectifies the tabletop board plane,
locates the 4 small physical dot marks, and computes their exact metric
workspace coordinates (x_m, y_m).

Exports the surveyed targets to src/assistant_bringup/config/pickup_slots.yaml.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "assistant_perception"))

from assistant_perception.workspace_mapping import find_marker_homography

DEFAULT_URL = "http://192.168.1.2:8080/video"
CONFIG_PATH = REPO_ROOT / "src" / "assistant_bringup" / "config" / "pickup_slots.yaml"
SAVE_DIR = REPO_ROOT / "camera_check"

# Default pulse values from tested firmware
SLOT_BASE_PULSES = [230, 300, 370, 440]
SLOT_LABELS = [
    "Slot 0 (Base 230 - Left)",
    "Slot 1 (Base 300 - Mid Left)",
    "Slot 2 (Base 370 - Mid Right)",
    "Slot 3 (Base 440 - Right)",
]


def detect_dots_in_rectified_board(
    rectified_bgr: np.ndarray,
    width_m: float = 0.3115,
    height_m: float = 0.1500,
    expected_dots: int = 4,
) -> list[tuple[float, float, float, float]]:
    """Find the small dark dot marks on the rectified tabletop image.

    Returns list of tuples: (x_px, y_px, x_m, y_m) sorted left-to-right.
    """
    h_px, w_px = rectified_bgr.shape[:2]
    gray = cv2.cvtColor(rectified_bgr, cv2.COLOR_BGR2GRAY)

    # Blur slightly to suppress paper grain
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Adaptive threshold to isolate small dark marks on lighter surface
    thresh = cv2.adaptiveThreshold(
        blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV, 15, 8
    )

    # Exclude outer border margins (where markers or table borders lie)
    margin_x = int(w_px * 0.05)
    margin_y = int(h_px * 0.10)
    thresh[:margin_y, :] = 0
    thresh[h_px - margin_y:, :] = 0
    thresh[:, :margin_x] = 0
    thresh[:, w_px - margin_x:] = 0

    # Find connected components / contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # Dots are small (between 5 and 250 px^2 in typical rectified resolution)
        if 4.0 <= area <= 300.0:
            perimeter = cv2.arcLength(cnt, True)
            if perimeter > 0:
                circularity = 4.0 * math.pi * (area / (perimeter * perimeter))
                # Accept roughly circular contours
                if circularity >= 0.35:
                    m = cv2.moments(cnt)
                    if m["m00"] > 0:
                        cx = m["m10"] / m["m00"]
                        cy = m["m01"] / m["m00"]
                        candidates.append((cx, cy, area, circularity))

    if not candidates:
        return []

    # Sort left-to-right along X
    candidates.sort(key=lambda c: c[0])

    # If more than 4, cluster into 4 bins along X or pick the 4 strongest
    # Partition X range into 4 equal bands and pick best candidate in each band
    dots = []
    band_width = w_px / expected_dots
    for band in range(expected_dots):
        band_min = band * band_width
        band_max = (band + 1) * band_width
        in_band = [c for c in candidates if band_min <= c[0] < band_max]
        if in_band:
            # Pick one closest to central Y
            in_band.sort(key=lambda c: abs(c[1] - (h_px / 2.0)))
            best = in_band[0]
            cx, cy = best[0], best[1]
            x_m = (cx / w_px) * width_m
            y_m = (cy / h_px) * height_m
            dots.append((cx, cy, x_m, y_m))

    return dots


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=DEFAULT_URL, help="IP Webcam URL")
    parser.add_argument("--image", default="", help="Optional existing snapshot path")
    parser.add_argument("--save-config", action="store_true", help="Write detected targets to pickup_slots.yaml")
    args = parser.parse_args()

    SAVE_DIR.mkdir(parents=True, exist_ok=True)

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"[ERROR] Cannot load image: {args.image}")
            return 1
        print(f"[OK] Loaded snapshot: {args.image}")
    else:
        print(f"[1/4] Connecting to camera: {args.url} ...")
        cap = cv2.VideoCapture(args.url)
        if not cap.isOpened():
            print(f"[ERROR] Cannot connect to camera stream at {args.url}")
            return 1
        ret, frame = cap.read()
        cap.release()
        if not ret or frame is None:
            print("[ERROR] Failed to grab frame from camera")
            return 1
        print(f"[OK] Captured live frame: {frame.shape[1]}x{frame.shape[0]} px")

    # Detect ArUco markers [0, 1, 2, 3]
    print("[2/4] Detecting workspace ArUco markers ...")
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    corners, ids, _ = aruco.detectMarkers(frame, aruco_dict)

    if ids is None or len(ids) < 4:
        found_ids = list(ids.flatten()) if ids is not None else []
        print(f"[ERROR] Need all 4 markers [0, 1, 2, 3], found only: {found_ids}")
        return 1

    id_map = {int(mid): tuple(c[0].mean(axis=0)) for mid, c in zip(ids.flatten(), corners)}
    if not all(k in id_map for k in (0, 1, 2, 3)):
        print(f"[ERROR] Missing required markers. Detected: {list(id_map.keys())}")
        return 1

    print("[OK] All 4 markers detected:")
    for k in (0, 1, 2, 3):
        print(f"  ID {k}: ({id_map[k][0]:.1f}, {id_map[k][1]:.1f}) px")

    # Rectify board to metric aspect ratio (623 x 300 px = ~2000 px/m)
    W_px, H_px = 623, 300
    src_pts = np.array([id_map[0], id_map[1], id_map[2], id_map[3]], dtype=np.float32)
    dst_pts = np.array([[0, 0], [W_px - 1, 0], [W_px - 1, H_px - 1], [0, H_px - 1]], dtype=np.float32)

    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    rectified = cv2.warpPerspective(frame, M, (W_px, H_px))

    rect_path = SAVE_DIR / "board_dots_rectified.jpg"
    cv2.imwrite(str(rect_path), rectified)

    print("\n[3/4] Searching for physical dot marks inside board ...")
    dots = detect_dots_in_rectified_board(rectified)

    annotated = rectified.copy()
    print(f"\n--- Survey Results ({len(dots)}/4 dots found) ---")

    slot_entries = []
    for i, (cx, cy, x_m, y_m) in enumerate(dots):
        pulse = SLOT_BASE_PULSES[i] if i < len(SLOT_BASE_PULSES) else 330
        label = SLOT_LABELS[i] if i < len(SLOT_LABELS) else f"Slot {i}"

        print(f"  {label}:")
        print(f"    Tabletop Position: X={x_m*100:.2f} cm, Y={y_m*100:.2f} cm ({x_m:.4f}m, {y_m:.4f}m)")
        print(f"    Base Servo Pulse: {pulse}")

        # Draw on annotated image
        cv2.circle(annotated, (int(cx), int(cy)), 8, (0, 0, 255), 2)
        cv2.circle(annotated, (int(cx), int(cy)), 2, (0, 255, 0), -1)
        cv2.putText(
            annotated, f"Slot {i} ({x_m*100:.1f}cm)",
            (int(cx) - 30, max(20, int(cy) - 12)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1
        )

        slot_entries.append({
            "id": i,
            "label": label,
            "base_pulse": pulse,
            "x_m": round(x_m, 4),
            "y_m": round(y_m, 4),
            "radius_m": 0.035,
            "verified": True,
        })

    annotated_path = SAVE_DIR / "board_dots_survey.jpg"
    cv2.imwrite(str(annotated_path), annotated)
    print(f"\nSaved survey visualization: {annotated_path}")

    if args.save_config and len(slot_entries) == 4:
        config_data = {
            "workspace": {
                "width_m": 0.3115,
                "height_m": 0.1500,
                "table_z_m": 0.0,
            },
            "slots": slot_entries,
        }
        with open(CONFIG_PATH, "w") as f:
            yaml.dump(config_data, f, sort_keys=False)
        print(f"[OK] Calibrated targets written to {CONFIG_PATH}")
    elif len(slot_entries) < 4:
        print("[NOTE] Did not automatically overwrite pickup_slots.yaml because fewer than 4 dots were detected.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
