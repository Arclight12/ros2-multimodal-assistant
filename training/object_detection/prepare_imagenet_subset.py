"""Prepare a selected ImageNet-LOC subset for Ultralytics YOLO training."""

from __future__ import annotations

import argparse
from pathlib import Path

from imagenet_subset import prepare_dataset


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images-root", type=Path, required=True)
    parser.add_argument("--annotations-root", type=Path, required=True)
    parser.add_argument("--classes", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--validation-fraction", type=float, default=0.2)
    parser.add_argument("--max-images-per-class", type=int, default=0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--force", action="store_true")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    summary = prepare_dataset(
        args.images_root,
        args.annotations_root,
        args.classes,
        args.output,
        args.validation_fraction,
        args.max_images_per_class,
        args.seed,
        args.force,
    )
    print(
        f"prepared={summary.image_count} train={summary.train_count} "
        f"val={summary.validation_count} data={summary.data_yaml}"
    )


if __name__ == "__main__":
    main()
