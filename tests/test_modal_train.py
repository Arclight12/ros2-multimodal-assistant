from pathlib import Path

import pytest

from training.object_detection.modal_train import (
    DEFAULT_GPU,
    DEFAULT_MODEL,
    DEFAULT_VOLUME_NAME,
    validate_modal_paths,
)


def test_modal_defaults_use_l40s_paths() -> None:
    assert DEFAULT_MODEL == "yolo11n.pt"
    assert DEFAULT_VOLUME_NAME == "ros-yolo-training"
    assert DEFAULT_GPU == "L40S"


def test_validate_modal_paths_rejects_missing_classes(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        validate_modal_paths(
            tmp_path / "images",
            tmp_path / "annotations",
            tmp_path / "classes.tsv",
        )


def test_validate_modal_paths_accepts_complete_source(tmp_path: Path) -> None:
    images = tmp_path / "images"
    annotations = tmp_path / "annotations"
    classes = tmp_path / "classes.tsv"
    images.mkdir()
    annotations.mkdir()
    classes.write_text("n1\tcup\n", encoding="utf-8")

    validate_modal_paths(images, annotations, classes)
