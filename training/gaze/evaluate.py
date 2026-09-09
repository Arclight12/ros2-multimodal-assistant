"""Evaluate normalized and physical gaze error."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from dataset import load_dataset


def main() -> None:
    """Report mean normalized error and tabletop centimetre error."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--workspace-width-m", type=float, required=True)
    parser.add_argument("--workspace-height-m", type=float, required=True)
    args = parser.parse_args()
    try:
        import torch
    except ImportError as exc:
        raise SystemExit("Install torch to evaluate the gaze model.") from exc
    features, targets = load_dataset(args.data)
    model = torch.jit.load(str(args.model), map_location="cpu").eval()
    with torch.no_grad():
        predictions = model(torch.from_numpy(features)).numpy()
    normalized_error = np.linalg.norm(predictions - targets, axis=1)
    physical_error_m = np.linalg.norm(
        (predictions - targets)
        * np.asarray([args.workspace_width_m, args.workspace_height_m]),
        axis=1,
    )
    print(f"normalized_mean_error={normalized_error.mean():.5f}")
    print(f"physical_mean_error_cm={physical_error_m.mean() * 100.0:.3f}")


if __name__ == "__main__":
    main()
