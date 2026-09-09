"""Fine-tune YOLO26n on an ImageNet-LOC subset using Modal."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

DEFAULT_GPU = "L40S"
DEFAULT_MODEL = "yolo11n.pt"
DEFAULT_VOLUME_NAME = "ros-yolo-training"
DATA_MOUNT = "/mnt/data"
DEFAULT_IMAGES_ROOT = f"{DATA_MOUNT}/imagenet-loc/images"
DEFAULT_ANNOTATIONS_ROOT = f"{DATA_MOUNT}/imagenet-loc/annotations"
DEFAULT_CLASSES_PATH = f"{DATA_MOUNT}/imagenet-loc/classes.tsv"
DEFAULT_OUTPUT_ROOT = f"{DATA_MOUNT}/output"
DEFAULT_KAGGLE_COMPETITION = "imagenet-object-localization-challenge"
DEFAULT_KAGGLE_SECRET = "Kaggle_Secret"


def validate_modal_paths(
    images_root: Path, annotations_root: Path, classes_path: Path
) -> None:
    """Validate the mounted source tree before allocating a GPU."""
    if not images_root.is_dir():
        raise FileNotFoundError(f"images directory not found: {images_root}")
    if not annotations_root.is_dir():
        raise FileNotFoundError(
            f"annotations directory not found: {annotations_root}"
        )
    if not classes_path.is_file():
        raise FileNotFoundError(f"classes file not found: {classes_path}")


try:
    import modal
except ImportError:  # pragma: no cover - remote dependency
    modal = None


if modal is not None:
    app = modal.App("ros-yolo26-training")
    volume = modal.Volume.from_name(DEFAULT_VOLUME_NAME, create_if_missing=True)
    image = (
        modal.Image.debian_slim(python_version="3.11")
        .uv_pip_install("ultralytics", "opencv-python-headless", "pyyaml", "kaggle")
        .add_local_dir(Path(__file__).parent, remote_path="/root/object_detection")
    )

    def _download_kaggle_images(
        annotations_root: Path,
        classes_path: Path,
        images_root: Path,
        max_images_per_class: int,
        seed: int,
    ) -> int:
        """Download only selected JPEGs directly into the mounted volume."""
        import sys

        sys.path.insert(0, "/root/object_detection")
        from kaggle_subset import KaggleImage, select_kaggle_images

        if not os.environ.get("KAGGLE_API_TOKEN") and os.environ.get("KAGGLE_KEY"):
            os.environ["KAGGLE_API_TOKEN"] = os.environ["KAGGLE_KEY"]
        if not os.environ.get("KAGGLE_API_TOKEN"):
            raise RuntimeError("Kaggle secret must provide KAGGLE_KEY or KAGGLE_API_TOKEN")

        items = select_kaggle_images(
            annotations_root, classes_path, max_images_per_class, seed
        )
        if not items:
            raise ValueError("no selected ImageNet annotations were found")

        def download(item: KaggleImage) -> bool:
            target = images_root / item.relative_image.relative_to(
                Path("ILSVRC", "Data", "CLS-LOC", "train")
            )
            if target.is_file() and target.stat().st_size:
                return True
            target.parent.mkdir(parents=True, exist_ok=True)
            for attempt in range(5):
                with tempfile.TemporaryDirectory() as temporary:
                    environment = os.environ.copy()
                    environment["KAGGLE_CONFIG_DIR"] = str(Path(temporary) / "kaggle-config")
                    result = subprocess.run(
                        [
                            "kaggle",
                            "competitions",
                            "download",
                            "-c",
                            DEFAULT_KAGGLE_COMPETITION,
                            "-f",
                            item.relative_image.as_posix(),
                            "-p",
                            temporary,
                            "--quiet",
                        ],
                        check=False,
                        capture_output=True,
                        text=True,
                        env=environment,
                    )
                    if not result.returncode:
                        candidates = [
                            path for path in Path(temporary).rglob("*") if path.is_file()
                        ]
                        if len(candidates) != 1:
                            raise RuntimeError(
                                f"unexpected Kaggle download for {item.relative_image}: {candidates}"
                            )
                        shutil.copy2(candidates[0], target)
                        return True
                    if "404" in result.stderr:
                        return False
                    if "429" not in result.stderr or attempt == 4:
                        raise RuntimeError(
                            f"Kaggle download failed for {item.relative_image}: {result.stderr}"
                        )
                    time.sleep(15 * (attempt + 1))
            return False

        # ponytail: one worker keeps Kaggle below its per-client request limit.
        with ThreadPoolExecutor(max_workers=1) as executor:
            downloaded = sum(executor.map(download, items))
        return downloaded

    @app.function(
        image=image,
        gpu=DEFAULT_GPU,
        timeout=86400,
        secrets=[modal.Secret.from_name(DEFAULT_KAGGLE_SECRET)],
        volumes={DATA_MOUNT: volume},
    )
    def train_remote(
        images_root: str = DEFAULT_IMAGES_ROOT,
        annotations_root: str = DEFAULT_ANNOTATIONS_ROOT,
        classes_path: str = DEFAULT_CLASSES_PATH,
        output_root: str = DEFAULT_OUTPUT_ROOT,
        model_name: str = DEFAULT_MODEL,
        epochs: int = 100,
        image_size: int = 640,
        batch_size: int = 32,
        workers: int = 4,
        patience: int = 20,
        validation_fraction: float = 0.2,
        max_images_per_class: int = 500,
        seed: int = 7,
        force: bool = False,
        download_from_kaggle: bool = True,
    ) -> str:
        """Prepare data, train on L40S, and persist the best checkpoint."""
        import sys

        sys.path.insert(0, "/root/object_detection")
        from imagenet_subset import prepare_dataset
        from train import train_detector

        source_images = Path(images_root)
        source_annotations = Path(annotations_root)
        source_classes = Path(classes_path)
        if download_from_kaggle:
            source_images.mkdir(parents=True, exist_ok=True)
            downloaded = _download_kaggle_images(
                source_annotations,
                source_classes,
                source_images,
                max_images_per_class,
                seed,
            )
            print(f"kaggle_images={downloaded}")
            volume.commit()
        validate_modal_paths(source_images, source_annotations, source_classes)
        output = Path(output_root)
        summary = prepare_dataset(
            source_images,
            source_annotations,
            source_classes,
            output / "yolo",
            validation_fraction,
            max_images_per_class,
            seed,
            force,
        )
        model_output = output / "models" / "object_detector.pt"
        train_detector(
            argparse.Namespace(
                data=summary.data_yaml,
                base_model=model_name,
                epochs=epochs,
                image_size=image_size,
                batch_size=batch_size,
                workers=workers,
                patience=patience,
                project=str(output / "runs"),
                name="yolo11n",
                device="0",
                exist_ok=True,
                output=model_output,
            )
        )
        volume.commit()
        return str(model_output)

    @app.local_entrypoint()
    def main(
        images_root: str = DEFAULT_IMAGES_ROOT,
        annotations_root: str = DEFAULT_ANNOTATIONS_ROOT,
        classes_path: str = DEFAULT_CLASSES_PATH,
        output_root: str = DEFAULT_OUTPUT_ROOT,
        model_name: str = DEFAULT_MODEL,
        epochs: int = 100,
        image_size: int = 640,
        batch_size: int = 32,
        workers: int = 4,
        patience: int = 20,
        validation_fraction: float = 0.2,
        max_images_per_class: int = 500,
        seed: int = 7,
        force: bool = False,
        download_from_kaggle: bool = True,
    ) -> None:
        """Launch the remote training function from ``modal run``."""
        output = train_remote.remote(
            images_root,
            annotations_root,
            classes_path,
            output_root,
            model_name,
            epochs,
            image_size,
            batch_size,
            workers,
            patience,
            validation_fraction,
            max_images_per_class,
            seed,
            force,
            download_from_kaggle,
        )
        print(f"trained_weights={output}")
else:
    app = None

    def main(*_: object, **__: object) -> None:
        """Report the missing local Modal dependency."""
        raise SystemExit("Install Modal with `python -m pip install modal`.")
