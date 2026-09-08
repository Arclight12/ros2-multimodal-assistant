"""Fine-tune YOLO26n on an ImageNet-LOC subset using Modal."""

from __future__ import annotations

import argparse
from pathlib import Path

DEFAULT_GPU = "L40S"
DEFAULT_MODEL = "yolo26n.pt"
DEFAULT_VOLUME_NAME = "ros-yolo-training"
DATA_MOUNT = "/mnt/data"
DEFAULT_IMAGES_ROOT = f"{DATA_MOUNT}/imagenet-loc/images"
DEFAULT_ANNOTATIONS_ROOT = f"{DATA_MOUNT}/imagenet-loc/annotations"
DEFAULT_CLASSES_PATH = f"{DATA_MOUNT}/imagenet-loc/classes.tsv"
DEFAULT_OUTPUT_ROOT = f"{DATA_MOUNT}/output"


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
        .uv_pip_install("ultralytics", "opencv-python-headless", "pyyaml")
        .add_local_dir(Path(__file__).parent, remote_path="/root/object_detection")
    )

    @app.function(
        image=image,
        gpu=DEFAULT_GPU,
        timeout=86400,
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
    ) -> str:
        """Prepare data, train on L40S, and persist the best checkpoint."""
        import sys

        sys.path.insert(0, "/root/object_detection")
        from imagenet_subset import prepare_dataset
        from train import train_detector

        source_images = Path(images_root)
        source_annotations = Path(annotations_root)
        source_classes = Path(classes_path)
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
                name="yolo26n",
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
        )
        print(f"trained_weights={output}")
else:
    app = None

    def main(*_: object, **__: object) -> None:
        """Report the missing local Modal dependency."""
        raise SystemExit("Install Modal with `python -m pip install modal`.")
