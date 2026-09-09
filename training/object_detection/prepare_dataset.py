"""Prepare annotated images and YOLO labels into train/validation splits."""

from __future__ import annotations

import argparse
import random
import shutil
from pathlib import Path

import yaml


def main() -> None:
    """Copy raw images/labels and write a dataset YAML file."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--classes", type=Path, required=True)
    parser.add_argument("--validation-fraction", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    class_names = [
        line.strip()
        for line in args.classes.read_text().splitlines()
        if line.strip()
    ]
    if not class_names:
        raise SystemExit("classes file must contain at least one class")
    if not 0.0 < args.validation_fraction < 1.0:
        raise SystemExit("validation-fraction must be between 0 and 1")
    images = sorted((args.input / "images").glob("*.jpg"))
    if not images:
        raise SystemExit("no JPG images found in input/images")
    random.Random(args.seed).shuffle(images)
    validation_count = max(1, int(len(images) * args.validation_fraction))
    validation = set(images[:validation_count])
    for image in images:
        split = "val" if image in validation else "train"
        destination = args.output / "images" / split
        labels = args.output / "labels" / split
        destination.mkdir(parents=True, exist_ok=True)
        labels.mkdir(parents=True, exist_ok=True)
        shutil.copy2(image, destination / image.name)
        source_label = args.input / "labels" / f"{image.stem}.txt"
        if source_label.exists():
            shutil.copy2(source_label, labels / source_label.name)
        else:
            (labels / f"{image.stem}.txt").touch()
    data = {
        "path": str(args.output.resolve()),
        "train": "images/train",
        "val": "images/val",
        "names": {index: name for index, name in enumerate(class_names)},
    }
    (args.output / "data.yaml").write_text(
        yaml.safe_dump(data, sort_keys=False), encoding="utf-8"
    )
    print(
        f"prepared={len(images)} classes={len(class_names)} "
        f"data={args.output / 'data.yaml'}"
    )


if __name__ == "__main__":
    main()
