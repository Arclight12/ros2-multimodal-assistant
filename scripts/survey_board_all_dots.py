#!/usr/bin/env python3
"""Survey all dots on the tabletop board (outer arc and newly marked inner dots)."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URL = "http://192.168.1.2:8080/video"
SAVE_DIR = REPO_ROOT / "camera_check"

ARC_CENTER_X = 0.1539
ARC_CENTER_Y = 0.3034


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default=DEFAULT_URL, help="Camera video stream URL")
    parser.add_argument("--image", default="", help="Optional snapshot image path")
    args = parser.parse_args()

    SAVE_DIR.mkdir(parents=True, exist_ok=True)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    frames = []
    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print(f"[ERROR] Cannot load image: {args.image}")
            return 1
        frames = [img]
    else:
        print(f"Connecting to camera: {args.url} ...")
        cap = cv2.VideoCapture(args.url)
        if not cap.isOpened():
            print(f"[ERROR] Cannot connect to camera stream at {args.url}")
            return 1

        print("Flushing buffer and sampling 10 frames ...")
        for _ in range(5):
            cap.read()
        for _ in range(10):
            ret, frame = cap.read()
            if ret and frame is not None:
                frames.append(frame)
        cap.release()

    if not frames:
        print("[ERROR] No frames captured")
        return 1

    cal_live = None
    best_frame = None

    for frame in frames:
        corners, ids, _ = aruco.detectMarkers(frame, aruco_dict)
        if ids is not None:
            id_map = {int(mid): tuple(c[0].mean(axis=0)) for mid, c in zip(ids.flatten(), corners)}
            found = set(id_map.keys()).intersection({0, 1, 2, 3})
            if len(found) == 4:
                src_pts = np.array([id_map[0], id_map[1], id_map[2], id_map[3]], dtype=np.float32)
                best_frame = frame
                break
            elif len(found) == 3:
                p = {k: np.array(id_map[k], dtype=np.float32) for k in found}
                if 2 not in found and {0, 1, 3}.issubset(found):
                    p[2] = p[1] + p[3] - p[0]
                elif 1 not in found and {0, 2, 3}.issubset(found):
                    p[1] = p[0] + p[2] - p[3]
                elif 3 not in found and {0, 1, 2}.issubset(found):
                    p[3] = p[0] + p[2] - p[1]
                elif 0 not in found and {1, 2, 3}.issubset(found):
                    p[0] = p[1] + p[3] - p[2]
                if len(p) == 4:
                    src_pts = np.array([p[0], p[1], p[2], p[3]], dtype=np.float32)
                    best_frame = frame
                    break

    if best_frame is None:
        print("[ERROR] Could not detect required ArUco markers in view")
        return 1

    W_px, H_px = 623, 300
    dst_pts = np.array([[0.0, 0.0], [W_px - 1, 0.0], [W_px - 1, H_px - 1], [0.0, H_px - 1]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)
    rectified = cv2.warpPerspective(best_frame, M, (W_px, H_px))

    cv2.imwrite(str(SAVE_DIR / "board_all_dots_rectified.jpg"), rectified)

    # Detect dots
    gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 15, 8)

    # Exclude borders
    margin_x = int(W_px * 0.04)
    margin_y = int(H_px * 0.08)
    thresh[:margin_y, :] = 0
    thresh[H_px - margin_y:, :] = 0
    thresh[:, :margin_x] = 0
    thresh[:, W_px - margin_x:] = 0

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 4.0 <= area <= 350.0:
            perimeter = cv2.arcLength(cnt, True)
            if perimeter > 0:
                circ = 4.0 * math.pi * (area / (perimeter * perimeter))
                if circ >= 0.30:
                    m = cv2.moments(cnt)
                    if m["m00"] > 0:
                        cx = m["m10"] / m["m00"]
                        cy = m["m01"] / m["m00"]
                        candidates.append((cx, cy, area))

    annotated = rectified.copy()
    print(f"\n[SURVEY] Detected {len(candidates)} candidate marks on the board:")

    # Cluster or sort dots
    candidates.sort(key=lambda c: (c[1], c[0]))  # sort primarily by Y, then X
    dot_index = 0
    for cx, cy, area in candidates:
        xm = cx / W_px * 0.3115
        ym = cy / H_px * 0.1500
        # Distance from reach arc center
        r_dist = math.hypot(xm - ARC_CENTER_X, ym - ARC_CENTER_Y)
        angle_deg = math.degrees(math.atan2(ym - ARC_CENTER_Y, xm - ARC_CENTER_X))
        dot_index += 1

        cv2.circle(annotated, (int(cx), int(cy)), 10, (0, 0, 255), 2)
        cv2.putText(annotated, f"D{dot_index}: ({xm*100:.1f},{ym*100:.1f})cm",
                    (int(cx) - 30, int(cy) - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

        print(f"  Dot #{dot_index}: pos=({xm*100:.2f} cm, {ym*100:.2f} cm), reach R={r_dist*100:.2f} cm, angle={angle_deg:.1f}°")

    out_path = SAVE_DIR / "board_all_dots_survey.jpg"
    cv2.imwrite(str(out_path), annotated)
    print(f"\nSaved survey artifact: {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
