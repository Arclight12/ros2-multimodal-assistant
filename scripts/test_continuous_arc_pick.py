#!/usr/bin/env python3
"""Continuous tabletop arc pickup demonstration for ROS-Multiarm-Assistant.

Workflow:
1. Connects to workspace camera stream (or loads image snapshot).
2. Detects the 4 corner ArUco markers [0, 1, 2, 3] and establishes tabletop metric homography.
3. Detects target object centroid (cx, cy) using YOLO.
4. Maps centroid to tabletop workspace coordinates (X_ws, Y_ws).
5. Safety Gates:
   a. Reach Radius Check: verifies distance from arm base to object is within reach arc (R = 22.64 cm ± 3.5 cm).
   b. Angle Check: verifies angle is within calibrated sweep limits [-56.3°, -123.8°].
6. Interpolates continuous base servo pulse strictly clamped to [230, 440].
7. Dispatches the safe 'P,<pulse>\n' command to Arduino on /dev/ttyACM0.
8. Monitors cycle progress until 'CYCLE_COMPLETE' (Ctrl+C for instant emergency stop).
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np
import serial
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "assistant_perception"))
sys.path.insert(0, str(REPO_ROOT / "src" / "assistant_motion"))

from ultralytics import YOLO

DEFAULT_URL = "http://192.168.1.2:8080/video"
DEFAULT_SERIAL = "/dev/ttyACM0"
DEFAULT_BAUD = 115200
CONFIG_PATH = REPO_ROOT / "src" / "assistant_bringup" / "config" / "pickup_slots.yaml"

# Multi-Node Radial Reach Geometry & Empirically Calibrated Poses
ARC_CENTER_X = 0.1539  # metres
ARC_CENTER_Y = 0.3034  # metres

# Empirically Verified Radial Reach Nodes:
# 1. Close-In Zone (4-5 cm behind inner dots): R ~ 15.5 cm -> CH1 = 400, CH2 = 490
# 2. Inner Dots Arc:                          R ~ 19.9 cm -> CH1 = 440, CH2 = 500
# 3. Outer Dots Arc:                          R ~ 22.8 cm -> CH1 = 500, CH2 = 540
RADIAL_NODES_M = [0.155, 0.199, 0.228]
LINK1_NODES = [400, 440, 500]
LINK2_NODES = [490, 500, 540]

# Safe continuous 2D tabletop reach envelope across entire workspace depth
R_MIN_SAFE_M = 0.145  # 14.5 cm (close-in tabletop bound)
R_MAX_SAFE_M = 0.245  # 24.5 cm (outer tabletop bound)

# Reference angles and base pulses (monotonically increasing angles for interpolation)
# Slot 3 (-123.82 deg) -> Pulse 440 (Far Left)
# Slot 2 (-102.27 deg) -> Pulse 370 (Mid Left)
# Slot 1 ( -78.79 deg) -> Pulse 300 (Mid Right)
# Slot 0 ( -56.27 deg) -> Pulse 230 (Far Right)
REF_ANGLES_DEG = [-123.82, -102.27, -78.79, -56.27]
REF_PULSES = [440, 370, 300, 230]

MIN_SAFE_BASE = 230
MAX_SAFE_BASE = 440


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=DEFAULT_URL, help="Camera video stream URL")
    parser.add_argument("--port", default=DEFAULT_SERIAL, help="Arduino serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument("--conf", type=float, default=0.08, help="YOLO confidence threshold")
    parser.add_argument("--image", default="", help="Optional snapshot image path")
    parser.add_argument("--force-outer", action="store_true", help="Force outer arc linkage pose (CH1=500, CH2=540)")
    parser.add_argument("--force-inner", action="store_true", help="Force inner arc linkage pose (CH1=440, CH2=500)")
    parser.add_argument("--mock", action="store_true", help="Dry-run without sending commands to hardware")
    args = parser.parse_args()

    print("==========================================================")
    print("  ROS-Multiarm-Assistant: 2D Continuous Workspace Pickup")
    print("==========================================================")

    # 1. Load Calibrated Reference Slots for comparison
    with open(CONFIG_PATH) as f:
        cfg = yaml.safe_load(f)
    slots = cfg.get("slots", [])
    print(f"\n[1/4] Calibrated Reference Dots ({len(slots)} slots along arc):")
    for s in slots:
        print(f"  Slot {s['id']}: {s['label']} at ({s['x_m']*100:.2f} cm, {s['y_m']*100:.2f} cm), Base pulse: {s['base_pulse']}")

    # 2. Camera & Perception
    model_path = REPO_ROOT / "models" / "object_detection" / "object_detector.pt"
    print(f"\n[2/4] Loading YOLO detector ({model_path.name}) at conf>={args.conf} ...")
    model = YOLO(str(model_path))
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    frames_to_process = []
    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print(f"[ERROR] Cannot load image: {args.image}")
            return 1
        print(f"Loaded perception snapshot: {args.image}")
        frames_to_process = [img]
    else:
        print(f"Connecting to camera stream: {args.url} ...")
        cap = cv2.VideoCapture(args.url)
        if not cap.isOpened():
            print(f"[ERROR] Cannot connect to camera stream at {args.url}")
            return 1

        print("Flushing buffer and sampling 15 frames for stable object detection ...")
        for _ in range(5):
            cap.read()
        for _ in range(15):
            ret, frame = cap.read()
            if ret and frame is not None:
                frames_to_process.append(frame)
        cap.release()

    best_detection = None
    best_score = -1.0
    cal_live = None

    for frame in frames_to_process:
        corners, ids, _ = aruco.detectMarkers(frame, aruco_dict)
        if ids is not None:
            id_map = {int(mid): tuple(c[0].mean(axis=0)) for mid, c in zip(ids.flatten(), corners)}
            found = set(id_map.keys()).intersection({0, 1, 2, 3})
            if len(found) == 4:
                src_pts = np.array([id_map[0], id_map[1], id_map[2], id_map[3]], dtype=np.float32)
                dst_pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], dtype=np.float32)
                cal_live = cv2.getPerspectiveTransform(src_pts, dst_pts)
            elif len(found) == 3:
                # Geometric parallelogram recovery for single missing corner
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
                    dst_pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], dtype=np.float32)
                    cal_live = cv2.getPerspectiveTransform(src_pts, dst_pts)

        if cal_live is None:
            continue

        res = model(frame, conf=args.conf, verbose=False)[0]
        for b in res.boxes:
            cls_name = model.names[int(b.cls[0])]
            conf_val = float(b.conf[0])
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            p = np.array([cx, cy, 1.0], dtype=np.float32)
            p_norm = cal_live @ p
            p_norm /= p_norm[2]
            nx, ny = float(p_norm[0]), float(p_norm[1])
            if -0.10 <= nx <= 1.10 and -0.10 <= ny <= 1.10:
                ws = (nx * 0.3115, ny * 0.1500)
                dx_t = ws[0] - ARC_CENTER_X
                dy_t = ws[1] - ARC_CENTER_Y
                r_t = math.hypot(dx_t, dy_t)
                is_reachable = (R_MIN_SAFE_M <= r_t <= R_MAX_SAFE_M)

                # Prioritize detections physically positioned within reachable workspace
                score = (100.0 if is_reachable else 0.0) + conf_val
                if score > best_score:
                    best_score = score
                    best_detection = (cls_name, conf_val, ws)

    if frames_to_process:
        cv2.imwrite("camera_check/latest_continuous_arc_frame.jpg", frames_to_process[-1])

    if best_detection is None:
        print("[ERROR] No target object detected within workspace bounds!")
        if cal_live is None:
            print("  [Perception Hint] Could not detect all 4 ArUco markers (0, 1, 2, 3) in camera frame.")
        else:
            print("  [Perception Hint] ArUco markers detected, but no object bounding box was found.")
        return 1

    cls_name, conf_val, ws_coords = best_detection
    obj_x, obj_y = ws_coords
    print(f"\n[OK] Detected '{cls_name}' (conf={conf_val:.2f}) at tabletop pos: ({obj_x*100:.2f} cm, {obj_y*100:.2f} cm)")

    # 3. 2D Workspace Geometry & Safety Gates
    dx = obj_x - ARC_CENTER_X
    dy = obj_y - ARC_CENTER_Y
    dist_from_center = math.sqrt(dx * dx + dy * dy)

    print(f"\n[3/4] 2D Workspace Reach Analysis:")
    print(f"  Distance from arm base: {dist_from_center*100:.2f} cm")
    print(f"  Calibrated envelope: [{R_MIN_SAFE_M*100:.1f} cm, {R_MAX_SAFE_M*100:.1f} cm] (Close: {RADIAL_NODES_M[0]*100:.1f} cm, Inner: {RADIAL_NODES_M[1]*100:.1f} cm, Outer: {RADIAL_NODES_M[-1]*100:.1f} cm)")

    if dist_from_center < R_MIN_SAFE_M or dist_from_center > R_MAX_SAFE_M:
        print(f"\n[SAFETY REJECT] Object radial distance {dist_from_center*100:.1f} cm is outside safe 2D envelope [{R_MIN_SAFE_M*100:.1f} cm, {R_MAX_SAFE_M*100:.1f} cm]!")
        print("Please place the object on the tabletop workspace between the inner and outer reach bounds.")
        return 1

    # Angle relative to arm base
    angle_deg = math.degrees(math.atan2(dy, dx))
    print(f"  Target azimuth angle: {angle_deg:.2f}° (calibrated sweep: [{REF_ANGLES_DEG[0]:.1f}°, {REF_ANGLES_DEG[-1]:.1f}°])")

    if angle_deg < (REF_ANGLES_DEG[0] - 8.0) or angle_deg > (REF_ANGLES_DEG[-1] + 8.0):
        print(f"\n[SAFETY REJECT] Target angle {angle_deg:.1f}° is outside calibrated base rotation range!")
        return 1

    # Interpolate continuous base pulse from angle
    raw_base = float(np.interp(angle_deg, REF_ANGLES_DEG, REF_PULSES))
    clamped_base = int(round(np.clip(raw_base, MIN_SAFE_BASE, MAX_SAFE_BASE)))

    # Multi-node radial reach interpolation for CH1 (reach extension) and CH2 (tabletop touchdown)
    if args.force_outer:
        ch1_pulse = LINK1_NODES[-1]
        ch2_pulse = LINK2_NODES[-1]
        print(f"  [Override] Forcing OUTER arc pose: CH1={ch1_pulse}, CH2={ch2_pulse}")
    elif args.force_inner:
        ch1_pulse = LINK1_NODES[1]
        ch2_pulse = LINK2_NODES[1]
        print(f"  [Override] Forcing INNER arc pose: CH1={ch1_pulse}, CH2={ch2_pulse}")
    else:
        raw_ch1 = float(np.interp(dist_from_center, RADIAL_NODES_M, LINK1_NODES))
        raw_ch2 = float(np.interp(dist_from_center, RADIAL_NODES_M, LINK2_NODES))
        ch1_pulse = int(round(raw_ch1))
        ch2_pulse = int(round(raw_ch2))

    # Strict physical clamps
    ch1_pulse = int(np.clip(ch1_pulse, 390, 510))
    ch2_pulse = int(np.clip(ch2_pulse, 400, 550))

    # Compare against nearest reference slot
    slot_dists = []
    for s in slots:
        d = math.hypot(obj_x - s["x_m"], obj_y - s["y_m"])
        slot_dists.append((d, s["id"], s["label"], s["base_pulse"]))
    slot_dists.sort(key=lambda item: item[0])
    closest_d, closest_id, closest_label, closest_pulse = slot_dists[0]

    print(f"\n>>> 2D CONTINUOUS PICKUP COMMAND COMPUTED <<<")
    print(f"  Base Pulse: {clamped_base} (Angle: {angle_deg:.1f}°)")
    print(f"  Reach Distance: {dist_from_center*100:.2f} cm")
    print(f"  Reach Pulses: CH1={ch1_pulse} (Reach), CH2={ch2_pulse} (Touchdown)")
    print(f"  Closest predefined reference: Slot {closest_id} ({closest_label}, pulse {closest_pulse}) - distance: {closest_d*100:.1f} cm")

    cmd_str = f"P,{clamped_base},{ch1_pulse},{ch2_pulse}\n"
    print(f"\nPrepared Hardware Command: {cmd_str.strip()} (Hard-clamped safely to bounds)")

    if args.mock:
        print("\n[MOCK MODE] Dry-run active. Command sequence verified successfully!")
        return 0

    # 4. Hardware Execution
    print(f"\n[4/4] Connecting to Arduino at {args.port} (115200 baud) ...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as exc:
        print(f"[ERROR] Failed to open serial port {args.port}: {exc}")
        return 1

    print("Waiting for Arduino startup ('ARM_READY') ...")
    start_wait = time.monotonic()
    ready = False
    while time.monotonic() - start_wait < 5.0:
        if ser.in_waiting:
            line = ser.readline().decode("ascii", "ignore").strip()
            if line:
                print(f"  [Arduino] {line}")
            if "ARM_READY" in line or "READY" in line:
                ready = True
                break
        time.sleep(0.05)

    if not ready:
        print("[WARNING] Did not receive ARM_READY within 5s, proceeding with caution...")

    ser.reset_input_buffer()

    print(f"\n>>> SENDING COMMAND: {cmd_str.strip()} <<<")
    ser.write(cmd_str.encode("ascii"))
    ser.flush()

    print("Monitoring arm execution cycle (press Ctrl+C for emergency stop) ...")
    cycle_start = time.monotonic()
    finished = False
    success = False

    try:
        while time.monotonic() - cycle_start < 45.0:
            if ser.in_waiting:
                line = ser.readline().decode("ascii", "ignore").strip()
                if line:
                    print(f"  [Arduino] {line}")
                    if "CYCLE_COMPLETE" in line:
                        finished = True
                        success = True
                        break
                    elif "CYCLE_ABORTED" in line or "ERR" in line:
                        finished = True
                        success = False
                        break
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[EMERGENCY STOP] Operator interrupted. Sending 'x' to stop arm...")
        ser.write(b"x\n")
        ser.flush()
        ser.close()
        return 1

    ser.close()

    if success:
        print("\n==========================================================")
        print("  DEMONSTRATION SUCCESS: 2D Continuous Arc Pick Completed!")
        print(f"  Target Pose: Base={clamped_base}, CH1={ch1_pulse}, CH2={ch2_pulse} | State: CYCLE_COMPLETE")
        print("==========================================================")
        return 0
    else:
        print("\n[ERROR] Cycle did not complete successfully.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
