"""Train the gaze regressor on MPIIFaceGaze using Modal's cheapest GPU."""

from __future__ import annotations

import sys
from pathlib import Path

DEFAULT_GPU = "T4"
DEFAULT_VOLUME_NAME = "ros-gaze-training"
DATA_MOUNT = "/mnt/gaze"
DATASET_URL = "https://datasets.d2.mpi-inf.mpg.de/MPIIGaze/MPIIFaceGaze.zip"
DATASET_BYTES = 942343290
# .parent chain (not .parents[2]) because the file sits at /root inside Modal.
REPO_ROOT = Path(__file__).resolve().parent.parent.parent

try:
    import modal
except ImportError:  # pragma: no cover - remote dependency
    modal = None


if modal is not None:
    app = modal.App("ros-gaze-training")
    volume = modal.Volume.from_name(DEFAULT_VOLUME_NAME, create_if_missing=True)
    image = (
        modal.Image.debian_slim(python_version="3.11")
        .apt_install("libgl1", "libglib2.0-0")
        .uv_pip_install(
            "mediapipe==0.10.21",
            "numpy<2",
            "scipy<1.14",
            "opencv-contrib-python-headless",
            "torch",
        )
        .add_local_dir(Path(__file__).parent, remote_path="/root/training/gaze")
        .add_local_dir(
            REPO_ROOT / "src" / "assistant_interaction" / "assistant_interaction",
            remote_path="/root/src/assistant_interaction/assistant_interaction",
        )
    )

    def _ensure_dataset(root: Path) -> Path:
        """Download and extract MPIIFaceGaze into the volume once."""
        import urllib.request
        import zipfile

        extracted = root / "MPIIFaceGaze"
        if (extracted / "p14" / "p14.txt").is_file():
            return extracted
        archive = root / "MPIIFaceGaze.zip"
        if not archive.is_file() or archive.stat().st_size != DATASET_BYTES:
            print("stage=downloading_dataset", flush=True)
            urllib.request.urlretrieve(DATASET_URL, archive)
        if archive.stat().st_size != DATASET_BYTES:
            raise RuntimeError("MPIIFaceGaze download is incomplete")
        print("stage=extracting_dataset", flush=True)
        with zipfile.ZipFile(archive) as bundle:
            bundle.extractall(root)
        volume.commit()
        return extracted

    @app.function(
        image=image,
        gpu=DEFAULT_GPU,
        cpu=8,
        timeout=7200,
        volumes={DATA_MOUNT: volume},
    )
    def train_remote(epochs: int = 80, stride: int = 1, force: bool = False) -> dict:
        """Extract features, train on the GPU, and return the TorchScript model."""
        from multiprocessing import Pool

        import numpy as np

        sys.path[:0] = ["/root/training/gaze", "/root/src/assistant_interaction"]
        from prepare_mpiifacegaze import extract_person
        from train import fit

        root = Path(DATA_MOUNT)
        extracted = _ensure_dataset(root)
        cache = root / f"features_stride{stride}.npz"
        if force or not cache.is_file():
            print("stage=extracting_features", flush=True)
            people = sorted(extracted.glob("p[0-9][0-9]"))
            with Pool(8) as pool:
                results = pool.starmap(extract_person, [(p, stride) for p in people])
            features = [row for part, _ in results for row in part]
            targets = [row for _, part in results for row in part]
            np.savez(
                cache,
                features=np.asarray(features, np.float32),
                targets=np.asarray(targets, np.float32),
            )
            volume.commit()
        with np.load(cache) as data:
            features, targets = data["features"], data["targets"]
        print(f"stage=training samples={len(features)} gpu={DEFAULT_GPU}", flush=True)
        scripted, error = fit(features, targets, epochs, device="cuda")
        split = int(len(features) * 0.8)
        baseline = float(np.linalg.norm(targets[split:] - 0.5, axis=1).mean())
        output = root / "output" / "gaze_model.pt"
        output.parent.mkdir(parents=True, exist_ok=True)
        scripted.save(str(output))
        volume.commit()
        print(
            f"stage=complete error={error:.4f} centre_baseline={baseline:.4f}",
            flush=True,
        )
        return {
            "model": output.read_bytes(),
            "error": error,
            "baseline": baseline,
            "samples": len(features),
        }

    @app.local_entrypoint()
    def main(
        epochs: int = 80,
        stride: int = 1,
        force: bool = False,
        output: str = "models/gaze/gaze_model.pt",
    ) -> None:
        """Run remote training and save the model locally."""
        result = train_remote.remote(epochs, stride, force)
        target = Path(output)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(result["model"])
        print(
            f"saved={target} samples={result['samples']} "
            f"validation_error={result['error']:.4f} "
            f"centre_baseline={result['baseline']:.4f}"
        )
else:
    app = None

    def main(*_: object, **__: object) -> None:
        """Report the missing local Modal dependency."""
        raise SystemExit("Install Modal with `python -m pip install modal`.")
