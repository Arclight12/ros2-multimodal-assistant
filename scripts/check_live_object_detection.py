#!/usr/bin/env python3
"""Multi-frame diagnostic to verify live object detection and workspace localization."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import defaultdict
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np

# Ensure robot-venv and assistant_perception packages are discoverable
venv_site = Path.home() / "robot-venv" / "lib" / "python3.12" / "site-packages"
if venv_site.is_dir() and str(venv_site) not in sys.path:
    sys.path.append(str(venv_site))

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "assistant_perception"))
sys.path.insert(0, str(REPO_ROOT / "src" / "assistant_motion"))

from ultralytics import YOLO
from assistant_perception.workspace_mapping import (
    WorkspaceCalibration,
    find_marker_homography,
    load_calibration,
    map_pixel_to_workspace,
    object_contact_pixel,
)
from assistant_motion.supervised_slot_picker import (
    DEFAULT_SLOT_REGIONS,
    load_slot_targets_from_yaml,
)

DEFAULT_URL = "http://192.168.1.2:8080/video"
DEFAULT_CALIBRATION = Path.home() / ".ros" / "ros2_multimodal_assistant" / "workspace_calibration.json"
DEFAULT_SLOTS_CONFIG = REPO_ROOT / "src" / "assistant_bringup" / "config" / "pickup_slots.yaml"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=DEFAULT_URL, help="IP Webcam URL")
    parser.add_argument("--frames", type=int, default=30, help="Frames to evaluate (default 30)")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold (default 0.25)")
    parser.add_argument("--prefix", default="session5", help="File prefix for snapshots")
    parser.add_argument("--save-dir", default="camera_check", help="Save directory")
    args = parser.parse_args()

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n[1/3] Connecting to workspace camera stream: {args.url} ...")
    cap = cv2.VideoCapture(args.url)
    if not cap.isOpened():
        print(f"[ERROR] Cannot connect to camera stream at {args.url}")
        sys.exit(1)

    # Initial frame check
    ret, frame0 = cap.read()
    if not ret or frame0 is None:
        print("[ERROR] Connected but failed to read initial frame.")
        cap.release()
        sys.exit(1)

    h, w = frame0.shape[:2]
    print(f"[OK] Stream active: {w}x{h} px, initial mean brightness: {frame0.mean():.1f}/255")

    # Load model and calibration
    model_path = REPO_ROOT / "models" / "object_detection" / "object_detector.pt"
    print(f"\n[2/3] Loading YOLO model from {model_path} ...")
    model = YOLO(str(model_path))
    print(f"[OK] Model loaded with {len(model.names)} classes: {model.names}")

    cal = load_calibration(str(DEFAULT_CALIBRATION))
    if not cal.has_planar_calibration:
        print("[WARNING] Planar calibration is not active or invalid in workspace_calibration.json!")
    else:
        print(f"[OK] Planar calibration loaded: workspace span {cal.workspace_width_m}m x {cal.workspace_height_m}m")

    slot_targets = load_slot_targets_from_yaml(DEFAULT_SLOTS_CONFIG)
    active_slots = slot_targets if slot_targets else DEFAULT_SLOT_REGIONS
    slot_src = "pickup_slots.yaml" if slot_targets else "built-in defaults"
    print(f"[OK] Loaded {len(active_slots)} target slots from {slot_src}")

    # 3. Capture and evaluate frames
    print(f"\n[3/3] Capturing and evaluating {args.frames} consecutive frames (hold steady) ...")
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    aruco_counts = defaultdict(int)
    object_detections = defaultdict(list)
    frames_evaluated = 0
    start_time = time.monotonic()

    best_frame = None
    best_detections = []
    best_corners = None
    best_ids = None

    for i in range(args.frames):
        ret, frame = cap.read()
        if not ret or frame is None:
            continue
        frames_evaluated += 1

        # Check ArUco and compute live homography if all 4 markers are visible
        corners, ids, _ = aruco.detectMarkers(frame, aruco_dict)
        frame_cal = cal
        if ids is not None and len(ids) > 0:
            for mid in ids.flatten():
                aruco_counts[int(mid)] += 1
            if len(set(ids.flatten()).intersection({0, 1, 2, 3})) == 4:
                centers = {int(mid): tuple(c[0].mean(axis=0)) for mid, c in zip(ids.flatten(), corners)}
                live_H = find_marker_homography(centers, (0, 1, 2, 3))
                if live_H is not None:
                    frame_cal = WorkspaceCalibration(
                        calibrated=True,
                        homography=live_H,
                        workspace_width_m=cal.workspace_width_m if cal.calibrated else 0.3115,
                        workspace_height_m=cal.workspace_height_m if cal.calibrated else 0.1500,
                        table_z_m=0.0,
                        base_origin=(0.0, 0.0, 0.0),
                        base_yaw_rad=0.0,
                        robot_transform_configured=False,
                    )

        # Run YOLO
        res = model(frame, conf=args.conf, verbose=False)[0]
        boxes = res.boxes

        frame_objs = []
        if boxes is not None and len(boxes) > 0:
            for box, conf_val, cls_id in zip(boxes.xyxy.tolist(), boxes.conf.tolist(), boxes.cls.tolist()):
                cls_name = model.names[int(cls_id)]
                x1, y1, x2, y2 = box
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                contact_x, contact_y = object_contact_pixel(cx, cy, y2 - y1)
                ws = map_pixel_to_workspace(contact_x, contact_y, frame_cal) if frame_cal.calibrated else None
                slot_id = None
                slot_label = ""
                if ws is not None:
                    for sid, sreg in active_slots.items():
                        if sreg.contains(ws[0], ws[1]):
                            slot_id = sid
                            slot_label = sreg.label
                            break
                obj_info = {
                    "class": cls_name,
                    "confidence": conf_val,
                    "bbox": (x1, y1, x2, y2),
                    "center": (cx, cy),
                    "contact": (contact_x, contact_y),
                    "workspace": ws,
                    "slot_id": slot_id,
                    "slot_label": slot_label,
                }
                frame_objs.append(obj_info)
                object_detections[cls_name].append(obj_info)

        if len(frame_objs) > len(best_detections) or (best_frame is None and len(frame_objs) > 0):
            best_frame = frame.copy()
            best_detections = frame_objs
            best_corners = corners
            best_ids = ids

    elapsed = time.monotonic() - start_time
    cap.release()
    fps = frames_evaluated / max(elapsed, 0.001)

    print(f"\n[DONE] Captured {frames_evaluated} frames in {elapsed:.2f}s ({fps:.1f} FPS).")
    print("\n--- ArUco Marker Stability ---")
    for mid in (0, 1, 2, 3):
        count = aruco_counts[mid]
        pct = (count / max(frames_evaluated, 1)) * 100
        print(f"  Marker ID {mid}: {count}/{frames_evaluated} frames ({pct:.1f}%)")

    print("\n--- Object Detection Summary ---")
    if not object_detections:
        print("  No objects detected above confidence threshold", args.conf)
    else:
        for cls_name, list_objs in object_detections.items():
            avg_conf = sum(o["confidence"] for o in list_objs) / len(list_objs)
            print(f"  Class '{cls_name}': detected in {len(list_objs)}/{frames_evaluated} frames, avg conf={avg_conf:.3f}")
            valid_ws = [o["workspace"] for o in list_objs if o["workspace"] is not None]
            if valid_ws:
                avg_x = sum(w[0] for w in valid_ws) / len(valid_ws)
                avg_y = sum(w[1] for w in valid_ws) / len(valid_ws)
                slots = [o["slot_label"] for o in list_objs if o["slot_label"]]
                slot_summary = f" -> {slots[0]}" if slots else " (no slot match)"
                print(f"    Avg Tabletop Position: X={avg_x*100:.2f} cm, Y={avg_y*100:.2f} cm ({avg_x:.4f}m, {avg_y:.4f}m){slot_summary}")

    # Save snapshots
    if best_frame is None:
        best_frame = frame0

    raw_path = save_dir / f"{args.prefix}_raw_snapshot.jpg"
    cv2.imwrite(str(raw_path), best_frame)
    print(f"\nSaved raw snapshot: {raw_path}")

    annotated = best_frame.copy()
    if best_ids is not None and len(best_ids) > 0:
        aruco.drawDetectedMarkers(annotated, best_corners, best_ids)
        for corner, mid in zip(best_corners, best_ids.flatten()):
            c = corner[0]
            cx = int(c[:, 0].mean())
            cy = int(c[:, 1].mean())
            cv2.putText(annotated, f"ID {mid}", (cx - 20, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    for o in best_detections:
        x1, y1, x2, y2 = o["bbox"]
        cx, cy = o["contact"]
        ws = o["workspace"]
        cls_name = o["class"]
        conf_val = o["confidence"]
        slot_id = o.get("slot_id")
        slot_str = f" [Slot {slot_id}]" if slot_id is not None else ""
        label = f"{cls_name} {conf_val:.2f}: {ws_str}{slot_str}"

        cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.circle(annotated, (int(cx), int(cy)), 5, (0, 0, 255), -1)
        cv2.putText(annotated, label, (int(x1), max(25, int(y1) - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    diag_path = save_dir / f"{args.prefix}_live_detections.jpg"
    cv2.imwrite(str(diag_path), annotated)
    print(f"Saved annotated diagnostic: {diag_path}")


if __name__ == "__main__":
    main()
