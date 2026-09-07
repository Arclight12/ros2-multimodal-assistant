"""Evaluate a trained YOLO-family detector."""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
    """Run validation and print the detector metrics."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--image-size", type=int, default=640)
    args = parser.parse_args()
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit("Install ultralytics to evaluate the object detector.") from exc
    metrics = YOLO(str(args.model)).val(data=str(args.data), imgsz=args.image_size)
    print(metrics)


if __name__ == "__main__":
    main()
