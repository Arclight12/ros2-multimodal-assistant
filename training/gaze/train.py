"""Train and export the custom gaze regressor."""

from __future__ import annotations

import argparse
from pathlib import Path

from dataset import load_dataset
from model import build_model


def fit(features, targets, epochs=80, batch_size=64, learning_rate=1e-3, device="cpu"):
    """Train on the first 80% of samples; return (TorchScript model, val error).

    Samples are stored participant by participant, so the tail split validates
    on people the model has not seen.
    """
    import torch

    torch.manual_seed(0)
    split = max(1, int(len(features) * 0.8))
    x = torch.from_numpy(features).to(device)
    y = torch.from_numpy(targets).to(device)
    mean = x[:split].mean(0)
    std = x[:split].std(0) + 1e-6
    model = build_model(mean, std).to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=learning_rate, weight_decay=1e-3
    )
    validation_x, validation_y = (x[split:], y[split:]) if split < len(x) else (x, y)
    best_error, best_state = float("inf"), None
    for epoch in range(epochs):
        model.train()
        order = torch.randperm(split, device=device)
        for start in range(0, split, batch_size):
            batch = order[start:start + batch_size]
            optimizer.zero_grad()
            torch.nn.functional.mse_loss(model(x[batch]), y[batch]).backward()
            optimizer.step()
        model.eval()
        with torch.no_grad():
            error = torch.linalg.vector_norm(
                model(validation_x) - validation_y, dim=1
            ).mean().item()
        if error < best_error:
            best_error = error
            best_state = {k: v.detach().clone() for k, v in model.state_dict().items()}
        if (epoch + 1) % 10 == 0 or epoch == 0:
            print(f"epoch={epoch + 1} validation_mean_error={error:.5f}", flush=True)
    model.load_state_dict(best_state)
    return torch.jit.script(model.eval().cpu()), best_error


def main() -> None:
    """Train on the project's labelled gaze samples and export TorchScript."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("models/gaze/gaze_model.pt"))
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    args = parser.parse_args()
    try:
        import torch  # noqa: F401 - availability guard; fit() uses torch
    except ImportError as exc:
        raise SystemExit("Install torch to train the gaze model.") from exc
    features, targets = load_dataset(args.data)
    scripted, best_error = fit(
        features, targets, args.epochs, args.batch_size, args.learning_rate
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    scripted.save(str(args.output))
    print(f"saved={args.output} normalized_mean_error={best_error:.5f}")


if __name__ == "__main__":
    main()
