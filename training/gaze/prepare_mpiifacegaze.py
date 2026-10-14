"""Convert the MPIIFaceGaze dataset into the NPZ format used by train.py."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from preprocess import extract_features


def extract_person(person_dir: Path, stride: int = 1):
    """Return features and 0-1 screen targets for one participant folder."""
    import cv2
    import mediapipe as mp
    from scipy.io import loadmat

    screen = loadmat(str(person_dir / "Calibration" / "screenSize.mat"))
    size = np.asarray(
        [screen["width_pixel"].item(), screen["height_pixel"].item()], dtype=np.float32
    )
    face_mesh = mp.solutions.face_mesh.FaceMesh(
        static_image_mode=True, max_num_faces=1, refine_landmarks=True
    )
    features, targets = [], []
    rows = (person_dir / f"{person_dir.name}.txt").read_text().splitlines()[::stride]
    for row in rows:
        fields = row.split()
        image = cv2.imread(str(person_dir / fields[0]))
        if image is None:
            continue
        values = extract_features(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), face_mesh)
        if values is None:
            continue
        features.append(values)
        targets.append(
            np.clip(np.asarray(fields[1:3], dtype=np.float32) / size, 0.0, 1.0)
        )
    face_mesh.close()
    print(f"{person_dir.name}: kept {len(features)}/{len(rows)}", flush=True)
    return features, targets


def main() -> None:
    """Extract features for every participant and write one NPZ."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        required=True,
        help="extracted MPIIFaceGaze directory (contains p00..p14)",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--stride", type=int, default=1, help="keep every Nth image of each participant"
    )
    args = parser.parse_args()
    features, targets = [], []
    # Sorted participants keep subjects contiguous, so train.py's sequential
    # split validates on unseen people.
    for person_dir in sorted(args.root.glob("p[0-9][0-9]")):
        person_features, person_targets = extract_person(person_dir, args.stride)
        features += person_features
        targets += person_targets
    if not features:
        raise SystemExit("No face landmarks were found in the dataset.")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.output,
        features=np.asarray(features, dtype=np.float32),
        targets=np.asarray(targets, dtype=np.float32),
    )
    print(f"Wrote {len(features)} samples to {args.output}")


if __name__ == "__main__":
    main()
