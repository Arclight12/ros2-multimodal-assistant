"""Select ImageNet-LOC files without downloading the full archive."""

from __future__ import annotations

import random
from dataclasses import dataclass
from pathlib import Path

try:
    from .imagenet_subset import load_class_mapping
except ImportError:  # Modal adds this directory directly to sys.path.
    from imagenet_subset import load_class_mapping


@dataclass(frozen=True)
class KaggleImage:
    """One ImageNet annotation and its matching Kaggle image path."""

    annotation: Path
    relative_image: Path
    synset: str


def select_kaggle_images(
    annotations_root: Path,
    classes_path: Path,
    max_images_per_class: int = 0,
    seed: int = 7,
) -> list[KaggleImage]:
    """Return deterministic selected-class ImageNet-LOC image paths."""
    if max_images_per_class < 0:
        raise ValueError("max_images_per_class cannot be negative")

    mapping = load_class_mapping(classes_path)
    selected: list[KaggleImage] = []
    for synset in mapping.synset_to_index:
        paths = sorted((annotations_root / synset).glob("*.xml"))
        random.Random(seed + mapping.synset_to_index[synset]).shuffle(paths)
        if max_images_per_class:
            paths = paths[:max_images_per_class]
        selected.extend(
            KaggleImage(
                annotation=path,
                relative_image=Path(
                    "ILSVRC", "Data", "CLS-LOC", "train", synset, f"{path.stem}.JPEG"
                ),
                synset=synset,
            )
            for path in paths
        )
    return selected
