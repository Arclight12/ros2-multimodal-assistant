#!/usr/bin/env python3
"""Voice-guided continuous 2D tabletop object selection and pickup demonstration.

Workflow:
1. Connects to phone audio stream (http://<ip>:8080/audio.wav).
2. Listens for user voice utterance (RMS-gated segmenter).
3. Transcribes speech via NVIDIA hosted Riva ASR (or manual override prompt).
4. Matches spoken utterance to target class (e.g. 'sharpener', 'block_a', 'eraser').
5. Captures workspace camera frame (http://<ip>:8080/video).
6. Detects all objects via custom YOLO + 4-corner ArUco homography.
7. Filters detected objects matching the spoken class, selecting the highest-confidence one.
8. Applies 2D kinematics (R, theta) -> (Base, CH1, CH2) with object height compensation.
9. Dispatches synchronized safe 'P,<base>,<ch1>,<ch2>\n' to Arduino on /dev/ttyACM0.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np
import serial

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "assistant_interaction"))
sys.path.insert(0, str(REPO_ROOT / "src" / "assistant_perception"))
sys.path.insert(0, str(REPO_ROOT / "src" / "assistant_motion"))

from assistant_interaction.audio_stream import TARGET_RATE, Segmenter, open_wav_stream, to_mono_16k
from assistant_interaction.voice_command import DEFAULT_ALIASES, match_class
from ultralytics import YOLO

try:
    import riva.client
except ImportError:
    riva = None

DEFAULT_VIDEO_URL = "http://192.168.1.2:8080/video"
DEFAULT_AUDIO_URL = "http://192.168.1.2:8080/audio.wav"
DEFAULT_SERIAL = "/dev/ttyACM0"
DEFAULT_BAUD = 115200

# Kinematics geometry & empirical calibration nodes
ARC_CENTER_X = 0.1539  # metres
ARC_CENTER_Y = 0.3034  # metres

RADIAL_NODES_M = [0.155, 0.199, 0.228]
LINK1_NODES = [400, 440, 500]
LINK2_NODES = [490, 500, 540]

R_MIN_SAFE_M = 0.145  # 14.5 cm
R_MAX_SAFE_M = 0.245  # 24.5 cm

REF_ANGLES_DEG = [-123.82, -102.27, -78.79, -56.27]
REF_PULSES = [440, 370, 300, 230]

MIN_SAFE_BASE = 230
MAX_SAFE_BASE = 440

# Height compensation: flatter objects need deeper CH2 tabletop touchdown
HEIGHT_OFFSET_CH2 = {
    "sharpener": 8,   # ~2.8 cm height -> +8 counts on CH2 for table touchdown
    "eraser": 10,     # ~1.8 cm height -> +10 counts on CH2
    "thermocol": 0,   # ~4 cm height -> standard touchdown
    "block_a": 0,     # ~4 cm height
    "block_b": 0,
    "block_c": 0,
}

# Radial pullback: smaller/narrower objects must not overshoot past front teeth into throat cavity
RADIAL_OFFSET_M = {
    "sharpener": -0.012,  # -1.2 cm pullback so gripper teeth grasp object at the tips
    "eraser": -0.010,     # -1.0 cm pullback
    "thermocol": 0.0,
    "block_a": 0.0,
    "block_b": 0.0,
    "block_c": 0.0,
}

# Gripper close pulse: smaller/narrower objects close tighter (440 vs 415 for wide blocks)
GRIPPER_CLOSE_PULSES = {

    "sharpener": 440,  # ~4-5 mm tighter closure for narrow body
    "eraser": 440,
    "thermocol": 415,
    "block_a": 415,
    "block_b": 415,
    "block_c": 415,
}




def create_riva_asr(api_key: str):
    if riva is None:
        raise RuntimeError("nvidia-riva-client is not installed")
    auth = riva.client.Auth(
        None,
        True,
        "grpc.nvcf.nvidia.com:443",
        [
            ["function-id", "bb0837de-8c7b-481f-9ec8-ef5663e9c1fa"],
            ["authorization", f"Bearer {api_key}"],
        ],
    )
    asr = riva.client.ASRService(auth)
    config = riva.client.StreamingRecognitionConfig(
        config=riva.client.RecognitionConfig(
            encoding=riva.client.AudioEncoding.LINEAR_PCM,
            language_code="en-US",
            sample_rate_hertz=TARGET_RATE,
            audio_channel_count=1,
            max_alternatives=1,
        ),
        interim_results=False,
    )
    return asr, config


def transcribe_pcm(asr, config, pcm: bytes) -> tuple[str, float]:
    step = TARGET_RATE // 10 * 2
    chunks = (pcm[i : i + step] for i in range(0, len(pcm), step))
    finals = [
        result.alternatives[0]
        for response in asr.streaming_response_generator(chunks, config)
        for result in response.results
        if result.is_final
    ]
    text = " ".join(alt.transcript.strip() for alt in finals).strip()
    confs = [float(getattr(alt, "confidence", 0.0) or 0.0) for alt in finals]
    mean_conf = sum(confs) / len(confs) if confs else 0.0
    return text, mean_conf


def listen_for_command(audio_url: str, asr, config, min_rms: float = 200.0, timeout_s: float = 45.0) -> str:
    print(f"\n[Voice Engine] Connecting to phone microphone at {audio_url} ...")
    stream, fmt = open_wav_stream(audio_url, timeout_s=5.0)
    print(f"[Voice Engine] Connected ({fmt.sample_rate} Hz, {fmt.channels} ch, {fmt.bits} bit).")
    print("[Voice Engine] >>> SPEAK NOW into phone (e.g., 'pick up the sharpener' or 'block a') <<<")

    chunk_ms = 100
    segmenter = Segmenter(
        min_rms=min_rms,
        end_chunks=int(600 // chunk_ms),
        max_chunks=int(6.0 * 1000 // chunk_ms),
    )
    nbytes = max(1, fmt.sample_rate * chunk_ms // 1000) * fmt.channels * 2

    start_time = time.monotonic()
    try:
        while time.monotonic() - start_time < timeout_s:
            raw = b""
            while len(raw) < nbytes:
                part = stream.read(nbytes - len(raw))
                if not part:
                    raise ConnectionError("Audio stream closed unexpectedly")
                raw += part
            pcm = segmenter.feed(to_mono_16k(raw, fmt))
            if pcm is not None:
                print("[Voice Engine] Utterance captured! Sending to NVIDIA Riva ASR ...")
                text, conf = transcribe_pcm(asr, config, pcm)
                print(f"[Voice Engine] Transcribed: \"{text}\" (confidence: {conf:.2f})")
                matched = match_class(text)
                if matched:
                    print(f"[Voice Engine] Successfully matched to class: '{matched}'")
                    return matched
                else:
                    print(f"[Voice Engine] No valid object class matched in: \"{text}\". Waiting for next command...")
    finally:
        stream.close()

    raise TimeoutError(f"No valid voice command heard within {timeout_s} seconds.")


def detect_workspace_objects(video_url: str, conf_thresh: float = 0.08):
    model_path = REPO_ROOT / "models" / "object_detection" / "object_detector.pt"
    print(f"\n[Perception] Loading YOLO model ({model_path.name}) ...")
    model = YOLO(str(model_path))
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    print(f"[Perception] Connecting to workspace camera at {video_url} ...")
    cap = cv2.VideoCapture(video_url)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot connect to video stream at {video_url}")

    print("[Perception] Sampling frames for stable multi-object detection ...")
    start_t = time.monotonic()
    frames = []
    while time.monotonic() - start_t < 5.0 and len(frames) < 20:
        ret, frame = cap.read()
        if ret and frame is not None:
            frames.append(frame)
        else:
            time.sleep(0.05)
    cap.release()




    if not frames:
        raise RuntimeError("No frames received from workspace camera!")

    cal_live = None
    all_detections = []  # list of dicts: {class, conf, bbox, ws_coords, is_reachable}

    for frame in frames:
        corners, ids, _ = aruco.detectMarkers(frame, aruco_dict)
        if ids is not None:
            id_map = {int(mid): tuple(c[0].mean(axis=0)) for mid, c in zip(ids.flatten(), corners)}
            found = set(id_map.keys()).intersection({0, 1, 2, 3})
            if len(found) == 4:
                src_pts = np.array([id_map[0], id_map[1], id_map[2], id_map[3]], dtype=np.float32)
                dst_pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], dtype=np.float32)
                cal_live = cv2.getPerspectiveTransform(src_pts, dst_pts)
            elif len(found) == 3 and cal_live is None:
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

        res = model(frame, conf=conf_thresh, verbose=False)[0]
        frame_detections = []
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
                ws_x = nx * 0.3115
                ws_y = ny * 0.1500
                dx = ws_x - ARC_CENTER_X
                dy = ws_y - ARC_CENTER_Y
                r = math.hypot(dx, dy)
                is_reachable = (R_MIN_SAFE_M <= r <= R_MAX_SAFE_M)
                frame_detections.append({
                    "class": cls_name,
                    "conf": conf_val,
                    "bbox": (int(x1), int(y1), int(x2), int(y2)),
                    "ws": (ws_x, ws_y),
                    "dist": r,
                    "is_reachable": is_reachable,
                })
        if len(frame_detections) > len(all_detections):
            all_detections = frame_detections

    if cal_live is None:
        raise RuntimeError("Could not establish ArUco homography from camera frame!")


    return all_detections, frames[-1]


def compute_arm_pulses(cls_name: str, ws_x: float, ws_y: float):
    dx = ws_x - ARC_CENTER_X
    dy = ws_y - ARC_CENTER_Y
    dist = math.hypot(dx, dy)

    if dist < R_MIN_SAFE_M or dist > R_MAX_SAFE_M:
        raise ValueError(f"Radial distance {dist*100:.1f} cm outside safe envelope [{R_MIN_SAFE_M*100:.1f}, {R_MAX_SAFE_M*100:.1f}] cm")

    angle_deg = math.degrees(math.atan2(dy, dx))
    if angle_deg < (REF_ANGLES_DEG[0] - 8.0) or angle_deg > (REF_ANGLES_DEG[-1] + 8.0):
        raise ValueError(f"Target azimuth {angle_deg:.1f}° outside sweep range [{REF_ANGLES_DEG[0]:.1f}°, {REF_ANGLES_DEG[-1]:.1f}°]")

    # Interpolate Base servo pulse
    raw_base = float(np.interp(angle_deg, REF_ANGLES_DEG, REF_PULSES))
    base_pulse = int(round(np.clip(raw_base, MIN_SAFE_BASE, MAX_SAFE_BASE)))

    # Multi-node interpolation for CH1 (reach) and CH2 (height/touchdown)
    r_offset = RADIAL_OFFSET_M.get(cls_name, 0.0)
    effective_dist = dist + r_offset
    raw_ch1 = float(np.interp(effective_dist, RADIAL_NODES_M, LINK1_NODES))
    raw_ch2 = float(np.interp(effective_dist, RADIAL_NODES_M, LINK2_NODES))

    # Object-specific height compensation
    offset_ch2 = HEIGHT_OFFSET_CH2.get(cls_name, 0)
    ch1_pulse = int(round(raw_ch1))
    ch2_pulse = int(round(raw_ch2 + offset_ch2))

    ch1_pulse = int(np.clip(ch1_pulse, 390, 510))
    ch2_pulse = int(np.clip(ch2_pulse, 400, 550))

    gripper_pulse = GRIPPER_CLOSE_PULSES.get(cls_name, 435)

    return base_pulse, ch1_pulse, ch2_pulse, gripper_pulse, dist, angle_deg, offset_ch2, r_offset


def execute_pickup(port: str, baud: int, base: int, ch1: int, ch2: int, gripper: int = 435):
    print(f"\n[Hardware Controller] Opening serial connection to Arduino on {port} ({baud} baud) ...")

    ser = serial.Serial(port, baud, timeout=1.0)
    start_boot = time.monotonic()
    while time.monotonic() - start_boot < 4.0:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if line:
            print(f"  [Arduino] {line}")
            if "ARM_READY" in line:
                time.sleep(0.5)
                break

    cmd = f"P,{base},{ch1},{ch2},{gripper}\n"
    print(f"\n[Hardware Controller] >>> DISPATCHING 2D CONTINUOUS PICK: {cmd.strip()} <<<")
    ser.write(cmd.encode("utf-8"))
    ser.flush()



    print("[Hardware Controller] Monitoring cycle execution ... (Press Ctrl+C for instant emergency stop)")
    try:
        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                print(f"  [Arduino] {line}")
                if "CYCLE_COMPLETE" in line:
                    print("\n[SUCCESS] Continuous 2D pickup and handover completed successfully!")
                    break
                elif "CYCLE_ABORTED" in line or "ERR" in line:
                    print(f"\n[ERROR] Motion cycle reported failure: {line}")
                    break
    except KeyboardInterrupt:
        print("\n[EMERGENCY STOP] Sending 'x' abort to Arduino!")
        ser.write(b"x\n")
        time.sleep(0.5)
    finally:
        ser.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video-url", default=DEFAULT_VIDEO_URL, help="Workspace camera stream URL")
    parser.add_argument("--audio-url", default=DEFAULT_AUDIO_URL, help="Phone microphone audio stream URL")
    parser.add_argument("--port", default=DEFAULT_SERIAL, help="Arduino serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    parser.add_argument("--prompt", default="", help="Manual command override (skips live voice capture)")
    parser.add_argument("--min-rms", type=float, default=300.0, help="Minimum RMS volume threshold for speech")
    parser.add_argument("--mock", action="store_true", help="Dry run without sending serial commands to hardware")
    args = parser.parse_args()

    print("==================================================================")
    print("  ROS-Multiarm-Assistant: Voice-Guided 2D Multi-Object Selection  ")
    print("==================================================================")

    # 1. Voice Acquisition
    target_class = None
    if args.prompt:
        target_class = match_class(args.prompt)
        print(f"[Voice Engine] Manual override prompt: \"{args.prompt}\" -> matched class: '{target_class}'")
        if not target_class:
            print(f"[ERROR] Could not match prompt \"{args.prompt}\" to any known object class.")
            print(f"Known aliases: {list(DEFAULT_ALIASES.keys())}")
            return 1
    else:
        api_key = os.environ.get("NVIDIA_API_KEY", "")
        if not api_key:
            print("\n[ERROR] NVIDIA_API_KEY environment variable is not set!")
            print("Please export your key first:")
            print("  export NVIDIA_API_KEY=nvapi-...")
            print("Or provide a manual test prompt with --prompt \"pick sharpener\"")
            return 1

        asr, config = create_riva_asr(api_key)
        try:
            target_class = listen_for_command(args.audio_url, asr, config, min_rms=args.min_rms, timeout_s=30.0)
        except Exception as exc:
            print(f"[ERROR] Voice capture failed: {exc}")
            return 1

    # 2. Multi-Object Tabletop Perception
    detections, last_frame = detect_workspace_objects(args.video_url)
    print(f"\n[Perception] Detected {len(detections)} object(s) in tabletop workspace:")
    for idx, d in enumerate(detections):
        status = "REACHABLE" if d["is_reachable"] else "OUT_OF_BOUNDS"
        print(f"  [{idx+1}] Class: {d['class']:<10} | Conf: {d['conf']:.2f} | Pos: ({d['ws'][0]*100:5.2f} cm, {d['ws'][1]*100:5.2f} cm) | Dist: {d['dist']*100:5.2f} cm [{status}]")

    # 3. Disambiguation: Filter by Spoken Class & Select Highest Confidence
    matching = [d for d in detections if d["class"] == target_class]
    if not matching:
        print(f"\n[ERROR] Spoken object '{target_class}' was not detected in workspace!")
        print(f"Detected classes on table: {[d['class'] for d in detections]}")
        return 1

    # Default to highest confidence
    chosen = max(matching, key=lambda d: d["conf"])
    obj_x, obj_y = chosen["ws"]
    print(f"\n[Target Selected] Highest confidence match for '{target_class}':")
    print(f"  Confidence: {chosen['conf']:.2f}")
    print(f"  Position:   ({obj_x*100:.2f} cm, {obj_y*100:.2f} cm)")
    print(f"  Distance:   {chosen['dist']*100:.2f} cm")

    # 4. Save Visual Overlay for User Inspection
    vis_frame = last_frame.copy()
    for d in detections:
        x1, y1, x2, y2 = d["bbox"]
        is_target = (d is chosen)
        color = (0, 255, 255) if is_target else (0, 255, 0)  # Yellow for target, Green for others
        thickness = 3 if is_target else 1
        cv2.rectangle(vis_frame, (x1, y1), (x2, y2), color, thickness)
        label = f"{d['class']} ({d['conf']:.2f})"
        if is_target:
            label = f"[VOICE TARGET] {label}"
        cv2.putText(vis_frame, label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    os.makedirs("camera_check", exist_ok=True)
    overlay_path = "camera_check/latest_voice_pick_workspace.jpg"
    cv2.imwrite(overlay_path, vis_frame)
    print(f"[Perception] Saved multi-object selection overlay to: {overlay_path}")

    # 5. Compute Kinematics & Height Compensation
    try:
        base, ch1, ch2, grip, dist, angle, h_offset, r_offset = compute_arm_pulses(target_class, obj_x, obj_y)
    except ValueError as err:
        print(f"\n[SAFETY REJECT] Kinematics validation failed: {err}")
        return 1

    print(f"\n[Kinematics Solution]")
    print(f"  Target Azimuth: {angle:.2f}° -> Base Pulse: {base} (bounds: [{MIN_SAFE_BASE}, {MAX_SAFE_BASE}])")
    print(f"  Radial Reach:   {dist*100:.2f} cm (offset: {r_offset*100:+.1f} cm) -> CH1 (Reach): {ch1}")
    print(f"  Height Offset:  +{h_offset} counts for '{target_class}' -> CH2 (Touchdown): {ch2}")
    print(f"  Gripper Close:  {grip} counts for '{target_class}'")
    print(f"  Final Command:  P,{base},{ch1},{ch2},{grip}")

    # 6. Execute on Hardware
    if args.mock:
        print("\n[MOCK MODE] Hardware dispatch skipped.")
        return 0

    execute_pickup(args.port, args.baud, base, ch1, ch2, grip)
    return 0



if __name__ == "__main__":
    sys.exit(main())
