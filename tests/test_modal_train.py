import subprocess
from pathlib import Path

import pytest

from training.object_detection.modal_train import (
    DEFAULT_GPU,
    DEFAULT_MODEL,
    DEFAULT_VOLUME_NAME,
    _run_kaggle_download,
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


def test_kaggle_download_timeout_is_reported(monkeypatch, tmp_path: Path) -> None:
    def timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired(args[0], 1)

    monkeypatch.setattr("training.object_detection.modal_train.subprocess.run", timeout)
    with pytest.raises(TimeoutError, match="Kaggle download timed out"):
        _run_kaggle_download(["kaggle"], {})
