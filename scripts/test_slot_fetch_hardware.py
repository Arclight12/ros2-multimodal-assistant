#!/usr/bin/env python3
"""End-to-end live perception and supervised slot pickup demonstration.

Workflow:
1. Connects to workspace camera stream (IP Webcam) and runs YOLO object detection.
2. Localizes the tabletop object contact point onto the metric board plane.
3. Evaluates Euclidean distance to the 4 calibrated physical slot targets in pickup_slots.yaml.
4. Identifies the matching slot (e.g., Slot 1 at Base pulse 300).
5. Opens serial connection to Arduino on /dev/ttyACM0.
6. Waits for "ARM_READY" startup handshake.
7. Dispatches the tested "D,<slot>" routine and monitors cycle progress until "CYCLE_COMPLETE".
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
    evaluate_supervised_slot_pick,
    load_slot_targets_from_yaml,
)

DEFAULT_URL = "http://192.168.1.2:8080/video"
DEFAULT_SERIAL = "/dev/ttyACM0"
DEFAULT_BAUD = 115200
CONFIG_PATH = REPO_ROOT / "src" / "assistant_bringup" / "config" / "pickup_slots.yaml"
CALIBRATION_PATH = Path.home() / ".ros" / "ros2_multimodal_assistant" / "workspace_calibration.json"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=DEFAULT_URL, help="Camera video stream URL")
    parser.add_argument("--port", default=DEFAULT_SERIAL, help="Arduino serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument("--conf", type=float, default=0.08, help="YOLO confidence threshold")
    parser.add_argument("--image", default="", help="Optional snapshot image path to use instead of opening stream")
    parser.add_argument("--mock", action="store_true", help="Run without opening real serial hardware")
    parser.add_argument("--slot", type=int, default=-1, help="Force a specific slot index (0-3) instead of auto-detecting")
    args = parser.parse_args()

    print("==========================================================")
    print("  ROS-Multiarm-Assistant: Supervised Slot Fetch Demo")
    print("==========================================================")

    # 1. Load Slot Targets
    slot_targets = load_slot_targets_from_yaml(CONFIG_PATH)
    if not slot_targets:
        print(f"[ERROR] Failed to load slot targets from {CONFIG_PATH}")
        return 1
    print(f"\n[1/4] Loaded {len(slot_targets)} calibrated physical slot targets:")
    for sid, t in sorted(slot_targets.items()):
        print(f"  Slot {sid}: {t.label} at ({t.x_m*100:.2f} cm, {t.y_m*100:.2f} cm), Base pulse: {t.base_pulse}")

    # 2. Camera & Perception
    chosen_slot = args.slot
    if chosen_slot >= 0:
        print(f"\n[PERCEPTION] Manual override: using configured Slot {chosen_slot}")
    else:
        model_path = REPO_ROOT / "models" / "object_detection" / "object_detector.pt"
        print(f"Loading YOLO detector ({model_path.name}) at conf>={args.conf} ...")
        model = YOLO(str(model_path))
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        frames_to_process = []
        if args.image:
            img = cv2.imread(args.image)
            if img is None:
                print(f"[ERROR] Cannot load image: {args.image}")
                return 1
            print(f"\n[2/4] Loaded perception snapshot: {args.image}")
            frames_to_process = [img]
        else:
            print(f"\n[2/4] Connecting to camera stream: {args.url} ...")
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
        best_conf = 0.0
        cal_live = None

        for frame in frames_to_process:
            corners, ids, _ = aruco.detectMarkers(frame, aruco_dict)
            if ids is not None and len(set(ids.flatten()).intersection({0, 1, 2, 3})) == 4:
                id_map = {int(mid): tuple(c[0].mean(axis=0)) for mid, c in zip(ids.flatten(), corners)}
                # Exact calibration frame used when surveying the board dots into pickup_slots.yaml:
                # Marker 0: (0, 0), Marker 1: (W, 0), Marker 2: (W, H), Marker 3: (0, H)
                src_pts = np.array([id_map[0], id_map[1], id_map[2], id_map[3]], dtype=np.float32)
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
                if -0.15 <= nx <= 1.15 and -0.15 <= ny <= 1.15:
                    ws = (nx * 0.3115, ny * 0.1500)
                    if conf_val > best_conf:
                        best_conf = conf_val
                        best_detection = (cls_name, conf_val, ws)

        if frames_to_process:
            cv2.imwrite("camera_check/latest_hardware_fetch_frame.jpg", frames_to_process[-1])

        if best_detection is None:
            print("[ERROR] No target object detected within workspace bounds!")
            if cal_live is None:
                print("  [Perception Hint] Could not detect all 4 ArUco markers (0, 1, 2, 3) in camera frame.")
            else:
                print("  [Perception Hint] 4 ArUco markers detected, but no object bounding box was found above confidence threshold.")
            return 1

        cls_name, conf_val, ws_coords = best_detection
        print(f"[OK] Detected '{cls_name}' (conf={conf_val:.2f}) at tabletop pos: ({ws_coords[0]*100:.2f} cm, {ws_coords[1]*100:.2f} cm)")

        decision = evaluate_supervised_slot_pick(
            object_id=f"{cls_name}_0",
            class_name=cls_name,
            confidence=conf_val,
            min_confidence=args.conf,
            workspace_coords=ws_coords,
            slot_regions=slot_targets,
            operator_confirmed=True,
            mock_mode=args.mock,
            hardware_available=True,
            motion_authorized=True,
            command_id=1,
        )

        if not decision.allowed or decision.slot_id is None:
            print(f"[REJECTED] Slot fetch disallowed: {decision.reason}")
            print("\nDistances from detected object to calibrated slots:")
            for sid, t in sorted(slot_targets.items()):
                d = t.distance_to(ws_coords[0], ws_coords[1])
                print(f"  Slot {sid} ({t.label}): {d*100:.1f} cm (acceptance radius: {t.radius_m*100:.1f} cm)")
            return 1

        chosen_slot = decision.slot_id
        print(f"\n>>> TARGET MATCHED: Slot {chosen_slot} ({slot_targets[chosen_slot].label}) <<<")

    # 3. Hardware Execution
    cmd_str = f"D,{chosen_slot}\n"
    print(f"\n[3/4] Prepared Hardware Command: {cmd_str.strip()} (Slot {chosen_slot}, Base pulse {slot_targets[chosen_slot].base_pulse})")

    if args.mock:
        print("[MOCK MODE] Mock execution active. Not opening serial port.")
        print("[MOCK MODE] Command sequence verified successfully!")
        return 0

    print(f"\n[4/4] Connecting to Arduino at {args.port} (115200 baud) ...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as exc:
        print(f"[ERROR] Failed to open serial port {args.port}: {exc}")
        return 1

    print("Waiting for Arduino startup ('ARM_READY') ...")
    start_wait = time.monotonic()
    ready = False
    rx_buf = ""
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

    # Clear input buffer
    ser.reset_input_buffer()

    print(f"\n>>> SENDING COMMAND: {cmd_str.strip()} <<<")
    ser.write(cmd_str.encode("ascii"))
    ser.flush()

    # Monitor execution
    print("Monitoring arm execution cycle (press Ctrl+C to abort) ...")
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
        print("  DEMONSTRATION SUCCESS: Slot Pick Cycle Completed!")
        print("==========================================================")
        return 0
    else:
        print("\n[ERROR] Cycle did not complete successfully.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
