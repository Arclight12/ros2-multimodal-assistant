#!/usr/bin/env python3
"""Diagnostic utility to verify workspace camera stream and detect four ArUco markers.

Expected physical setup:
- Camera stream: http://192.168.1.2:8080/video (IP Webcam)
- ArUco dictionary: DICT_4X4_50
- Marker IDs & physical corners:
    ID 0 = Top-Left
    ID 1 = Top-Right
    ID 2 = Bottom-Right
    ID 3 = Bottom-Left

Usage:
    python3 scripts/check_aruco_markers.py [--url http://192.168.1.2:8080/video] [--frames 30] [--save-dir camera_check]
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections import defaultdict
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "assistant_perception"))

from assistant_perception.workspace_mapping import (
    CORNER_ORDER,
    DESTINATION_POINTS,
    find_marker_homography,
)

DEFAULT_URL = "http://192.168.1.2:8080/video"
EXPECTED_IDS = (0, 1, 2, 3)
ID_TO_CORNER = {
    0: "Top-Left",
    1: "Top-Right",
    2: "Bottom-Right",
    3: "Bottom-Left",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify real camera stream and detect physical ArUco markers.",
    )
    parser.add_argument(
        "--url",
        default=DEFAULT_URL,
        help=f"Workspace camera stream URL (default: {DEFAULT_URL})",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=30,
        help="Number of consecutive frames to evaluate for detection stability (default: 30)",
    )
    parser.add_argument(
        "--save-dir",
        default="camera_check",
        help="Directory to save diagnostic snapshots (default: camera_check)",
    )
    parser.add_argument(
        "--dictionary",
        default="DICT_4X4_50",
        help="OpenCV ArUco dictionary name (default: DICT_4X4_50)",
    )
    parser.add_argument(
        "--prefix",
        default="",
        help="Prefix for saved diagnostic snapshot images (default: none)",
    )
    return parser.parse_args()


def get_aruco_detector(dict_name: str):
    aruco_dict_id = getattr(aruco, dict_name, None)
    if aruco_dict_id is None:
        raise ValueError(f"Unknown ArUco dictionary: {dict_name}")
    dictionary = aruco.getPredefinedDictionary(aruco_dict_id)
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        return detector, dictionary
    return None, dictionary


def detect_markers(frame: np.ndarray, detector, dictionary):
    if detector is not None:
        corners, ids, rejected = detector.detectMarkers(frame)
    else:
        corners, ids, rejected = aruco.detectMarkers(frame, dictionary)
    return corners, ids, rejected


def main() -> int:
    args = parse_args()
    print("=" * 70)
    print("REAL WORKSPACE CAMERA & ARUCO MARKER DIAGNOSTIC")
    print("=" * 70)
    print(f"Target stream URL : {args.url}")
    print(f"Dictionary        : {args.dictionary}")
    print(f"Expected IDs      : {EXPECTED_IDS} (0=TL, 1=TR, 2=BR, 3=BL)")
    print(f"Sample frames     : {args.frames}")
    print(f"Output directory  : {args.save_dir}")
    print("-" * 70)

    # 1. Connect to camera stream
    print(f"[1/4] Connecting to stream at {args.url} ...")
    cap = cv2.VideoCapture(args.url)
    if not cap.isOpened():
        print(f"\n[ERROR] Failed to open video stream at {args.url}")
        print("Please check:")
        print("  1. Phone is on the same Wi-Fi/local network as this machine.")
        print("  2. The IP Webcam app is running and server is active.")
        print(f"  3. The phone IP address has not changed from {args.url}.")
        return 1

    detector, dictionary = get_aruco_detector(args.dictionary)
    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    # Read initial frames to clear buffer and determine resolution
    ret, test_frame = cap.read()
    if not ret or test_frame is None:
        print("\n[ERROR] Connected to stream URL, but failed to read initial frame.")
        cap.release()
        return 1

    img_h, img_w = test_frame.shape[:2]
    print(f"[OK] Stream connected successfully!")
    print(f"     Resolution: {img_w} x {img_h} px")

    # 2. Multi-frame detection check
    print(f"\n[2/4] Evaluating {args.frames} consecutive frames for marker detection & stability ...")
    detection_counts = defaultdict(int)
    marker_centers_history = defaultdict(list)
    marker_pixel_widths = defaultdict(list)
    last_annotated_frame = None
    last_detected_centers = {}

    start_time = time.monotonic()
    frames_read = 0

    for i in range(args.frames):
        ret, frame = cap.read()
        if not ret or frame is None:
            continue
        frames_read += 1
        corners, ids, _ = detect_markers(frame, detector, dictionary)

        current_frame_centers = {}
        if ids is not None and len(ids) > 0:
            for m_corners, m_id in zip(corners, ids.flatten()):
                mid = int(m_id)
                detection_counts[mid] += 1
                c = m_corners[0]
                cx, cy = float(c[:, 0].mean()), float(c[:, 1].mean())
                current_frame_centers[mid] = (cx, cy)
                marker_centers_history[mid].append((cx, cy))

                # Approximate width as average side length of marker square
                side1 = np.linalg.norm(c[0] - c[1])
                side2 = np.linalg.norm(c[1] - c[2])
                side3 = np.linalg.norm(c[2] - c[3])
                side4 = np.linalg.norm(c[3] - c[0])
                avg_side = float((side1 + side2 + side3 + side4) / 4.0)
                marker_pixel_widths[mid].append(avg_side)

        last_detected_centers = current_frame_centers

        # Keep a diagnostic annotated copy of a frame with all markers or latest
        if len(current_frame_centers) >= 4 or last_annotated_frame is None:
            annotated = frame.copy()
            if ids is not None and len(ids) > 0:
                aruco.drawDetectedMarkers(annotated, corners, ids)
                for mid, (cx, cy) in current_frame_centers.items():
                    cv2.circle(annotated, (int(round(cx)), int(round(cy))), 6, (0, 0, 255), -1)
                    label = f"ID {mid}: {ID_TO_CORNER.get(mid, 'Unknown')} ({int(cx)}, {int(cy)})"
                    cv2.putText(
                        annotated,
                        label,
                        (int(cx) - 40, int(cy) - 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2,
                    )
            # Add resolution & timestamp overlay
            overlay_text = f"Res: {img_w}x{img_h} | Frames evaluated: {i+1}/{args.frames}"
            cv2.putText(
                annotated,
                overlay_text,
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2,
            )
            last_annotated_frame = annotated

    cap.release()
    elapsed = time.monotonic() - start_time
    fps = frames_read / elapsed if elapsed > 0 else 0.0
    print(f"[OK] Completed {frames_read}/{args.frames} frames in {elapsed:.2f}s ({fps:.1f} FPS)")

    # 3. Analyze detections
    print(f"\n[3/4] Detection & Stability Report:")
    all_four_present = True
    for mid in EXPECTED_IDS:
        count = detection_counts[mid]
        rate = (count / frames_read * 100.0) if frames_read > 0 else 0.0
        widths = marker_pixel_widths[mid]
        avg_w = np.mean(widths) if widths else 0.0
        centers = marker_centers_history[mid]
        std_pos = (
            np.std(centers, axis=0).max() if len(centers) > 1 else 0.0
        )
        status = "STABLE" if rate > 80.0 else ("INTERMITTENT" if rate > 0 else "MISSING")
        print(f"  - Marker ID {mid} ({ID_TO_CORNER[mid]:12s}): "
              f"Detected {count:2d}/{frames_read} frames ({rate:5.1f}%) | "
              f"Avg size: {avg_w:5.1f} px | Jitter std: {std_pos:4.2f} px -> {status}")
        if count == 0:
            all_four_present = False

    # Check for unexpected detected IDs
    unexpected_ids = set(detection_counts.keys()) - set(EXPECTED_IDS)
    if unexpected_ids:
        print(f"  [NOTE] Additional unexpected marker IDs detected: {unexpected_ids}")

    # Save diagnostic images
    prefix = f"{args.prefix}_" if args.prefix and not args.prefix.endswith("_") else args.prefix
    snapshot_filename = f"{prefix}workspace_aruco_diagnostic.jpg"
    raw_snapshot_filename = f"{prefix}workspace_raw_snapshot.jpg"
    snapshot_path = save_dir / snapshot_filename
    raw_snapshot_path = save_dir / raw_snapshot_filename
    if last_annotated_frame is not None and all_four_present:
        cv2.imwrite(str(snapshot_path), last_annotated_frame)
        cv2.imwrite(str(raw_snapshot_path), test_frame)
        print(f"\n[SAVED] Annotated diagnostic snapshot saved to: {snapshot_path}")
        print(f"[SAVED] Raw camera frame saved to:              {raw_snapshot_path}")
    elif last_annotated_frame is not None:
        fail_path = save_dir / f"{prefix}failed_detection.jpg"
        cv2.imwrite(str(fail_path), last_annotated_frame)
        print(f"\n[SAVED] Incomplete detection snapshot saved to: {fail_path}")

    # 4. Diagnostic Homography Assessment
    print(f"\n[4/4] Diagnostic Planar Homography Check:")
    if not all_four_present or len(last_detected_centers) < 4:
        print("  [FAIL] Cannot compute homography: not all 4 markers were detected simultaneously.")
        return 2

    # Check that each expected ID is in last_detected_centers
    if any(mid not in last_detected_centers for mid in EXPECTED_IDS):
        # Fall back to median centers across history
        avg_centers = {
            mid: (float(np.mean([pt[0] for pt in pts])), float(np.mean([pt[1] for pt in pts])))
            for mid, pts in marker_centers_history.items()
            if len(pts) > 0
        }
    else:
        avg_centers = last_detected_centers

    # Verify corner sequence: ID 0 -> ID 1 -> ID 2 -> ID 3 traces perimeter in clockwise order
    pts = [avg_centers[i] for i in EXPECTED_IDS]
    pts_array = np.float32(pts)
    is_convex = bool(cv2.isContourConvex(np.int32(pts_array)))
    from assistant_perception.workspace_mapping import polygon_signed_area
    signed_area = polygon_signed_area(pts)
    is_clockwise = signed_area > 0.0

    tl = avg_centers[0]
    tr = avg_centers[1]
    br = avg_centers[2]
    bl = avg_centers[3]

    print("  [CORNER PERIMETER ORDER]:")
    print(f"       Far edge (TL -> TR):    ID 0 ({tl[0]:.1f}, {tl[1]:.1f}) -> ID 1 ({tr[0]:.1f}, {tr[1]:.1f})")
    print(f"       Right edge (TR -> BR):  ID 1 ({tr[0]:.1f}, {tr[1]:.1f}) -> ID 2 ({br[0]:.1f}, {br[1]:.1f})")
    print(f"       Near edge (BR -> BL):   ID 2 ({br[0]:.1f}, {br[1]:.1f}) -> ID 3 ({bl[0]:.1f}, {bl[1]:.1f})")
    print(f"       Left edge (BL -> TL):   ID 3 ({bl[0]:.1f}, {bl[1]:.1f}) -> ID 0 ({tl[0]:.1f}, {tl[1]:.1f})")
    print(f"       Convex: {is_convex} | Clockwise winding: {is_clockwise} (signed area: {signed_area:.1f} px^2)")

    if is_convex and is_clockwise:
        print("  [OK] Canonical sequence [0, 1, 2, 3] forms a valid, non-self-intersecting clockwise quadrilateral.")
    else:
        print("  [WARNING] Marker ordering does not form a valid convex clockwise quadrilateral.")
        print("  Please check physical marker placements on the board.")

    homography = find_marker_homography(avg_centers, EXPECTED_IDS)
    if homography is None:
        print("  [FAIL] find_marker_homography() returned None (geometry invalid or non-convex).")
        return 3

    print("  [OK] Valid planar homography successfully computed from detected marker centers.")
    print("  (Note: 3D robot base transform remains unverified; planar homography is for diagnostics only).")
    print("=" * 70)
    print("DIAGNOSTIC VERIFICATION COMPLETE: ALL 4 MARKERS DETECTED & VALIDATED")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    sys.exit(main())
