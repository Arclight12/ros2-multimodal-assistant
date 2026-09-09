"""Fine-tune YOLO11n on an ImageNet-LOC subset using Modal."""

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
KAGGLE_DOWNLOAD_TIMEOUT_SECONDS = 120


def _run_kaggle_download(
    command: list[str], environment: dict[str, str]
) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            env=environment,
            timeout=KAGGLE_DOWNLOAD_TIMEOUT_SECONDS,
        )
    except subprocess.TimeoutExpired as exc:
        raise TimeoutError(
            f"Kaggle download timed out after {KAGGLE_DOWNLOAD_TIMEOUT_SECONDS}s"
        ) from exc


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
    app = modal.App("ros-yolo11-training")
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
                    try:
                        result = _run_kaggle_download(
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
                            environment,
                        )
                    except TimeoutError as exc:
                        if attempt == 4:
                            raise RuntimeError(
                                f"Kaggle download timed out for {item.relative_image}"
                            ) from exc
                        print(
                            f"kaggle_retry item={item.relative_image} "
                            f"attempt={attempt + 1} reason=timeout",
                            flush=True,
                        )
                        time.sleep(15 * (attempt + 1))
                        continue
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
        downloaded = 0
        with ThreadPoolExecutor(max_workers=1) as executor:
            for index, result in enumerate(executor.map(download, items), 1):
                downloaded += result
                if index == 1 or index % 10 == 0 or index == len(items):
                    print(
                        f"kaggle_progress={index}/{len(items)} "
                        f"downloaded={downloaded}",
                        flush=True,
                    )
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
        print("stage=started", flush=True)
        if download_from_kaggle:
            print("stage=downloading_kaggle", flush=True)
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
        print(
            f"stage=preparing_dataset annotations={sum(1 for _ in source_annotations.rglob('*.xml'))}",
            flush=True,
        )
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
        print(
            f"stage=dataset_ready images={summary.image_count} "
            f"train={summary.train_count} val={summary.validation_count}",
            flush=True,
        )
        model_output = output / "models" / "object_detector.pt"
        print(f"stage=training model={model_name} gpu={DEFAULT_GPU}", flush=True)
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
        print(f"stage=checkpoint_ready path={model_output}", flush=True)
        volume.commit()
        print("stage=complete", flush=True)
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
