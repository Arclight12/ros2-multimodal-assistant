"""Convert labelled gaze images into face/eye/head feature arrays."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


FEATURE_INDICES = (1, 33, 133, 159, 145, 263, 362, 386, 374, 152)


def extract_features(image, face_mesh):
    """Return left-eye, right-eye, and head landmark features."""
    result = face_mesh.process(image)
    if not result.multi_face_landmarks:
        return None
    landmarks = result.multi_face_landmarks[0].landmark
    features = []
    for index in FEATURE_INDICES:
        landmark = landmarks[index]
        features.extend((landmark.x, landmark.y, landmark.z))
    return features


def main() -> None:
    """Preprocess all usable labelled frames into an NPZ dataset."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--labels", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        import cv2
        import mediapipe as mp
    except ImportError as exc:
        raise SystemExit(
            "Install opencv-contrib-python and mediapipe to preprocess gaze data."
        ) from exc
    face_mesh = mp.solutions.face_mesh.FaceMesh(
        static_image_mode=True, max_num_faces=1, refine_landmarks=True
    )
    features, targets, metadata = [], [], []
    with args.labels.open(encoding="utf-8") as labels:
        for line in labels:
            record = json.loads(line)
            image_path = args.labels.parent / record["image"]
            image = cv2.imread(str(image_path))
            if image is None:
                continue
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            values = extract_features(rgb, face_mesh)
            if values is None:
                continue
            features.append(values)
            targets.append([record["target_x"], record["target_y"]])
            metadata.append(record)
    face_mesh.close()
    if not features:
        raise SystemExit("No face landmarks were found in the labelled images.")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.output,
        features=np.asarray(features, dtype=np.float32),
        targets=np.asarray(targets, dtype=np.float32),
    )
    args.output.with_suffix(".metadata.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )
    print(f"Wrote {len(features)} samples to {args.output}")


if __name__ == "__main__":
    main()
