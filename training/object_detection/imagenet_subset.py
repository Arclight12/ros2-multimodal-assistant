"""Convert selected ImageNet-LOC XML annotations into a YOLO dataset."""

from __future__ import annotations

import math
import random
import shutil
import xml.etree.ElementTree as ET
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import yaml


@dataclass(frozen=True)
class ClassMapping:
    """Map ImageNet synset IDs to stable YOLO class IDs and names."""

    synset_to_index: Mapping[str, int]
    names: tuple[str, ...]


@dataclass(frozen=True)
class Box:
    """One pixel-space bounding box with a YOLO class ID."""

    class_id: int
    xmin: float
    ymin: float
    xmax: float
    ymax: float


@dataclass(frozen=True)
class Annotation:
    """Parsed annotation data needed to create one YOLO label file."""

    image_name: str
    width: int
    height: int
    boxes: tuple[Box, ...]


@dataclass(frozen=True)
class PreparationSummary:
    """Counts and output location returned by dataset preparation."""

    image_count: int
    train_count: int
    validation_count: int
    class_counts: Mapping[str, int]
    data_yaml: Path


@dataclass(frozen=True)
class _Record:
    image: Path
    labels: tuple[tuple[int, str], ...]


def load_class_mapping(path: Path) -> ClassMapping:
    """Read ``synset_id<TAB>class_name`` rows in stable order."""
    synset_to_index: dict[str, int] = {}
    names: list[str] = []
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), 1
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        parts = [part.strip() for part in line.split("\t")]
        if len(parts) != 2 or not all(parts):
            raise ValueError(
                f"{path}:{line_number} must contain synset_id<TAB>class_name"
            )
        synset, name = parts
        if synset in synset_to_index:
            raise ValueError(f"duplicate synset {synset!r} in {path}")
        if name in names:
            raise ValueError(f"duplicate class name {name!r} in {path}")
        synset_to_index[synset] = len(names)
        names.append(name)
    if not names:
        raise ValueError(f"class mapping {path} is empty")
    return ClassMapping(synset_to_index, tuple(names))


def _number(element: ET.Element | None, field: str) -> float | None:
    value = element.findtext(field) if element is not None else None
    try:
        number = float(value) if value is not None else float("nan")
    except ValueError:
        return None
    return number if math.isfinite(number) else None


def parse_annotation(path: Path, mapping: ClassMapping) -> Annotation:
    """Parse one Pascal VOC/ImageNet annotation and filter its objects."""
    root = ET.parse(path).getroot()
    size = root.find("size")
    try:
        width = int(size.findtext("width"))
        height = int(size.findtext("height"))
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid image dimensions in {path}") from exc
    if width <= 0 or height <= 0:
        raise ValueError(f"image dimensions must be positive in {path}")

    image_name = (root.findtext("filename") or f"{path.stem}.JPEG").strip()
    boxes: list[Box] = []
    for item in root.findall("object"):
        synset = (item.findtext("name") or path.parent.name).strip()
        class_id = mapping.synset_to_index.get(synset)
        if class_id is None:
            continue
        coordinates = item.find("bndbox")
        values = tuple(
            _number(coordinates, field)
            for field in ("xmin", "ymin", "xmax", "ymax")
        )
        if any(value is None for value in values):
            continue
        boxes.append(Box(class_id, *(value for value in values if value is not None)))
    return Annotation(image_name, width, height, tuple(boxes))


def to_yolo_line(box: Box, width: int, height: int) -> str | None:
    """Clip a pixel box and return a normalized YOLO label line."""
    xmin = min(width, max(0.0, box.xmin))
    ymin = min(height, max(0.0, box.ymin))
    xmax = min(width, max(0.0, box.xmax))
    ymax = min(height, max(0.0, box.ymax))
    if xmax <= xmin or ymax <= ymin:
        return None
    center_x = ((xmin + xmax) / 2.0) / width
    center_y = ((ymin + ymax) / 2.0) / height
    box_width = (xmax - xmin) / width
    box_height = (ymax - ymin) / height
    return (
        f"{box.class_id} {center_x:.6f} {center_y:.6f} "
        f"{box_width:.6f} {box_height:.6f}"
    )


def _records(
    images_root: Path,
    annotations_root: Path,
    mapping: ClassMapping,
) -> list[_Record]:
    records: list[_Record] = []
    image_paths = [path for path in images_root.rglob("*") if path.is_file()]
    images_by_key = {
        (path.parent.name.lower(), path.stem.lower()): path for path in image_paths
    }
    images_by_stem: dict[str, list[Path]] = defaultdict(list)
    for path in image_paths:
        images_by_stem[path.stem.lower()].append(path)
    for annotation_path in sorted(annotations_root.rglob("*.xml")):
        image = images_by_key.get(
            (annotation_path.parent.name.lower(), annotation_path.stem.lower())
        )
        if image is None:
            candidates = images_by_stem.get(annotation_path.stem.lower(), [])
            image = candidates[0] if len(candidates) == 1 else None
        if image is None:
            continue
        annotation = parse_annotation(annotation_path, mapping)
        labels = tuple(
            (box.class_id, label)
            for box in annotation.boxes
            if (label := to_yolo_line(box, annotation.width, annotation.height))
        )
        if labels:
            records.append(_Record(image, labels))
    return records


def _select_records(
    records: list[_Record], class_count: int, max_images_per_class: int, seed: int
) -> list[_Record]:
    shuffled = list(records)
    random.Random(seed).shuffle(shuffled)
    if max_images_per_class <= 0:
        return shuffled
    counts = [0] * class_count
    selected: list[_Record] = []
    for record in shuffled:
        active = {
            class_id
            for class_id, _ in record.labels
            if counts[class_id] < max_images_per_class
        }
        if not active:
            continue
        labels = tuple(label for label in record.labels if label[0] in active)
        selected.append(_Record(record.image, labels))
        for class_id in active:
            counts[class_id] += 1
    return selected


def prepare_dataset(
    images_root: Path,
    annotations_root: Path,
    classes_path: Path,
    output: Path,
    validation_fraction: float = 0.2,
    max_images_per_class: int = 0,
    seed: int = 7,
    force: bool = False,
) -> PreparationSummary:
    """Prepare a deterministic selected-class YOLO dataset."""
    if not 0.0 < validation_fraction < 1.0:
        raise ValueError("validation_fraction must be between 0 and 1")
    if max_images_per_class < 0:
        raise ValueError("max_images_per_class cannot be negative")
    if output.exists() and any(output.iterdir()):
        if not force:
            raise FileExistsError(f"output is not empty; pass --force: {output}")
        shutil.rmtree(output)
    if not images_root.is_dir() or not annotations_root.is_dir():
        raise FileNotFoundError("images-root and annotations-root must be directories")

    mapping = load_class_mapping(classes_path)
    records = _select_records(
        _records(images_root, annotations_root, mapping),
        len(mapping.names),
        max_images_per_class,
        seed,
    )
    if not records:
        raise ValueError("no selected images with valid boxes were found")

    validation_count = max(1, int(len(records) * validation_fraction)) if len(records) > 1 else 0
    validation = set(range(validation_count))
    class_counts = {name: 0 for name in mapping.names}
    for record in records:
        for class_id, _ in record.labels:
            class_counts[mapping.names[class_id]] += 1

    output.mkdir(parents=True, exist_ok=True)
    for index, record in enumerate(records):
        split = "val" if index in validation else "train"
        image_dir = output / "images" / split
        label_dir = output / "labels" / split
        image_dir.mkdir(parents=True, exist_ok=True)
        label_dir.mkdir(parents=True, exist_ok=True)
        name = f"image_{index:06d}{record.image.suffix.lower() or '.jpg'}"
        shutil.copy2(record.image, image_dir / name)
        (label_dir / f"{Path(name).stem}.txt").write_text(
            "\n".join(label for _, label in record.labels) + "\n",
            encoding="utf-8",
        )

    data_yaml = output / "data.yaml"
    data_yaml.write_text(
        yaml.safe_dump(
            {
                "path": str(output.resolve()),
                "train": "images/train",
                "val": "images/val",
                "names": list(mapping.names),
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return PreparationSummary(
        image_count=len(records),
        train_count=len(records) - validation_count,
        validation_count=validation_count,
        class_counts=class_counts,
        data_yaml=data_yaml,
    )
