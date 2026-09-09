"""Capture phone-camera images and empty YOLO labels for annotation."""

from __future__ import annotations

import argparse
import time
from pathlib import Path


def main() -> None:
    """Capture a configurable number of frames from the workspace camera."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, help="Phone IP stream URL")
    parser.add_argument(
        "--output", type=Path,
        default=Path("datasets/object_detection/raw")
    )
    parser.add_argument("--frames", type=int, default=200)
    parser.add_argument("--interval-seconds", type=float, default=0.5)
    args = parser.parse_args()
    if args.frames < 1:
        raise SystemExit("frames must be positive")
    try:
        import cv2
    except ImportError as exc:
        raise SystemExit(
            "Install opencv-contrib-python to collect object data."
        ) from exc
    capture = cv2.VideoCapture(args.source)
    if not capture.isOpened():
        raise SystemExit(f"Unable to open phone stream {args.source!r}")
    image_dir = args.output / "images"
    label_dir = args.output / "labels"
    image_dir.mkdir(parents=True, exist_ok=True)
    label_dir.mkdir(parents=True, exist_ok=True)
    try:
        for index in range(args.frames):
            success, frame = capture.read()
            if not success:
                continue
            image_path = image_dir / f"frame_{index:06d}.jpg"
            cv2.imwrite(str(image_path), frame)
            (label_dir / f"frame_{index:06d}.txt").touch()
            print(f"captured {image_path}")
            time.sleep(max(0.0, args.interval_seconds))
    finally:
        capture.release()


if __name__ == "__main__":
    main()
