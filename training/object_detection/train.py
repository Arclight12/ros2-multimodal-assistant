"""Fine-tune a YOLO-family detector on the project dataset."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


def main() -> None:
    """Train and copy the best detector weights into models/."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--base-model", default="yolov8n.pt")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument(
        "--output", type=Path,
        default=Path("models/object_detection/object_detector.pt")
    )
    args = parser.parse_args()
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit("Install ultralytics to train the object detector.") from exc
    model = YOLO(args.base_model)
    result = model.train(
        data=str(args.data), epochs=args.epochs, imgsz=args.image_size
    )
    best = Path(result.save_dir) / "weights" / "best.pt"
    if not best.is_file():
        raise SystemExit(f"training completed without best weights at {best}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best, args.output)
    print(f"saved={args.output}")


if __name__ == "__main__":
    main()
