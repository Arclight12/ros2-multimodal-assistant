#!/usr/bin/env python3
"""Check the phone workspace stream and laptop webcam without starting ROS.

Opens each configured camera with the same helpers the ROS nodes use, reports
resolution and measured capture rate, and saves snapshot images you can view
before launching the robot graph.

Examples:
  python scripts/check_cameras.py --workspace-url http://192.168.1.10:8080/video
  python scripts/check_cameras.py --gaze-index 0 --gaze-backend dshow
  python scripts/check_cameras.py --probe 4
  python scripts/check_cameras.py --workspace-url ... --gaze-index 0 --frames 60
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "assistant_perception"))
sys.path.insert(0, str(REPO_ROOT / "src" / "assistant_interaction"))

from assistant_interaction.camera_device import (  # noqa: E402
    WEBCAM_HINT,
    open_webcam,
    resolve_backend,
)
from assistant_perception.camera_stream import (  # noqa: E402
    FFMPEG_HINT,
    open_mjpeg_stream,
    read_frame,
)

try:
    import cv2
except ImportError:  # pragma: no cover - env problem reported in main()
    cv2 = None

CONSECUTIVE_FAILURE_LIMIT = 5


def parse_args() -> argparse.Namespace:
    """Parse and return the diagnostic CLI arguments."""
    parser = argparse.ArgumentParser(
        description="Verify the phone workspace stream and laptop webcam.",
    )
    parser.add_argument(
        "--workspace-url",
        default="",
        help="phone MJPEG stream URL, e.g. http://192.168.1.10:8080/video",
    )
    parser.add_argument(
        "--gaze-index",
        type=int,
        default=0,
        help="laptop webcam device index",
    )
    parser.add_argument(
        "--gaze-device",
        default="",
        help="optional webcam device path overriding --gaze-index",
    )
    parser.add_argument(
        "--gaze-backend",
        default="auto",
        help="capture backend: auto, any, dshow, msmf, v4l2, ffmpeg",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=30,
        help="frames to measure per camera (default: 30)",
    )
    parser.add_argument(
        "--save-dir",
        default="camera_check",
        help="directory for saved snapshots (default: ./camera_check)",
    )
    parser.add_argument("--open-timeout-ms", type=int, default=5000)
    parser.add_argument("--read-timeout-ms", type=int, default=10000)
    parser.add_argument(
        "--probe",
        type=int,
        default=0,
        metavar="N",
        help="only scan webcam indices 0..N-1 and report which ones open",
    )
    parser.add_argument("--skip-workspace", action="store_true")
    parser.add_argument("--skip-gaze", action="store_true")
    args = parser.parse_args()
    if not args.skip_workspace and not args.workspace_url:
        parser.error("--workspace-url is required unless --skip-workspace is given")
    return args


def capture_samples(capture, frame_count: int):
    """Read ``frame_count`` frames, returning (successes, fps, last_frame)."""
    successes = 0
    failures = 0
    last_frame = None
    started = time.monotonic()
    while successes < frame_count and failures < CONSECUTIVE_FAILURE_LIMIT:
        success, frame = read_frame(capture)
        if success and frame is not None:
            successes += 1
            failures = 0
            last_frame = frame
        else:
            failures += 1
    elapsed = time.monotonic() - started
    fps = successes / elapsed if elapsed > 0 else 0.0
    return successes, fps, last_frame


def report_snapshot(directory: Path, name: str, frame) -> str:
    """Save one snapshot image and describe its resolution and brightness."""
    height, width = frame.shape[:2]
    brightness = float(frame.mean())
    path = directory / f"{name}.jpg"
    saved = cv2.imwrite(str(path), frame)
    detail = f"{width}x{height}, mean brightness {brightness:.1f}"
    if saved:
        detail += f", snapshot {path}"
    return detail


def check_workspace(args: argparse.Namespace, directory: Path) -> bool:
    """Open the phone stream, sample frames, and save a snapshot."""
    print(f"\n[workspace] opening phone stream {args.workspace_url!r} ...")
    capture = open_mjpeg_stream(
        cv2, args.workspace_url, args.open_timeout_ms, args.read_timeout_ms
    )
    if capture is None:
        print(f"[workspace] FAIL: could not open stream. {FFMPEG_HINT}")
        return False
    successes, fps, frame = capture_samples(capture, args.frames)
    if successes == 0:
        print(
            "[workspace] FAIL: stream opened but returned no decodable "
            f"frames in {args.frames} tries. {FFMPEG_HINT}"
        )
        capture.release()
        return False
    print(
        f"[workspace] OK: {successes}/{args.frames} frames, "
        f"{fps:.1f} fps, {report_snapshot(directory, 'workspace_frame', frame)}"
    )
    capture.release()
    return True


def check_webcam(args: argparse.Namespace, directory: Path) -> bool:
    """Open the laptop webcam, sample frames, and save a snapshot."""
    source = args.gaze_device.strip() or args.gaze_index
    backend = resolve_backend(cv2, args.gaze_backend)
    print(
        f"\n[gaze] opening webcam {source!r} with backend "
        f"{args.gaze_backend!r} (OpenCV id {backend}) ..."
    )
    capture = open_webcam(cv2, source, args.gaze_backend, width=0, height=0)
    if capture is None:
        print(f"[gaze] FAIL: could not open webcam {source!r}. {WEBCAM_HINT}")
        return False
    successes, fps, frame = capture_samples(capture, args.frames)
    if successes == 0:
        print(f"[gaze] FAIL: webcam opened but returned no frames. {WEBCAM_HINT}")
        capture.release()
        return False
    name = f"gaze_frame_{source}".replace("/", "_").replace(":", "_")
    print(
        f"[gaze] OK: {successes}/{args.frames} frames, {fps:.1f} fps, "
        f"{report_snapshot(directory, name, frame)}"
    )
    capture.release()
    return True


def probe_webcams(args: argparse.Namespace) -> bool:
    """Report which webcam indices 0..N-1 can be opened."""
    print(f"\n[probe] scanning webcam indices 0..{args.probe - 1} ...")
    any_opened = False
    for index in range(max(0, args.probe)):
        capture = open_webcam(cv2, index, args.gaze_backend)
        if capture is None:
            print(f"[probe] index {index}: closed")
            continue
        success, frame = read_frame(capture)
        size = f"{frame.shape[1]}x{frame.shape[0]}" if success else "no frame"
        print(f"[probe] index {index}: OPEN, {size}, backend {args.gaze_backend!r}")
        capture.release()
        any_opened = True
    if not any_opened:
        print(f"[probe] FAIL: no webcam opened. {WEBCAM_HINT}")
    return any_opened


def main() -> int:
    """Run the requested checks and return a process exit code."""
    if cv2 is None:
        print(
            "OpenCV is required: python3 -m pip install opencv-contrib-python",
            file=sys.stderr,
        )
        return 2
    args = parse_args()
    if args.probe > 0:
        return 0 if probe_webcams(args) else 1
    directory = Path(args.save_dir)
    directory.mkdir(parents=True, exist_ok=True)
    results = []
    if not args.skip_workspace:
        results.append(check_workspace(args, directory))
    if not args.skip_gaze:
        results.append(check_webcam(args, directory))
    if not results:
        print("Nothing to check: both cameras were skipped.")
        return 2
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
