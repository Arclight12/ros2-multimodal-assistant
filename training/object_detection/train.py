"""Fine-tune a YOLO-family detector on the project dataset."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


def build_parser() -> argparse.ArgumentParser:
    """Build the local and Modal-compatible training CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--base-model", default="yolo26n.pt")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--patience", type=int, default=20)
    parser.add_argument("--project", default="runs/object_detection")
    parser.add_argument("--name", default="train")
    parser.add_argument("--device", default=None)
    parser.add_argument("--exist-ok", action="store_true")
    parser.add_argument(
        "--output", type=Path,
        default=Path("models/object_detection/object_detector.pt")
    )
    return parser


def train_detector(args: argparse.Namespace) -> Path:
    """Fine-tune the selected model and copy its best checkpoint."""
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit("Install ultralytics to train the object detector.") from exc
    model = YOLO(args.base_model)
    train_options = {
        "data": str(args.data),
        "epochs": args.epochs,
        "imgsz": args.image_size,
        "batch": args.batch_size,
        "workers": args.workers,
        "patience": args.patience,
        "project": args.project,
        "name": args.name,
        "exist_ok": args.exist_ok,
    }
    if args.device is not None:
        train_options["device"] = args.device
    result = model.train(**train_options)
    best = Path(result.save_dir) / "weights" / "best.pt"
    if not best.is_file():
        raise SystemExit(f"training completed without best weights at {best}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best, args.output)
    print(f"saved={args.output}")
    return args.output


def main() -> None:
    """Train and copy the best detector weights into models/."""
    train_detector(build_parser().parse_args())


if __name__ == "__main__":
    main()
