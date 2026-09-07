"""Train and export the custom gaze regressor."""

from __future__ import annotations

import argparse
from pathlib import Path

from dataset import load_dataset
from model import build_model


def main() -> None:
    """Train on the project's labelled gaze samples and export TorchScript."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("models/gaze/gaze_model.pt"))
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    args = parser.parse_args()
    try:
        import torch
        from torch.utils.data import DataLoader, TensorDataset
    except ImportError as exc:
        raise SystemExit("Install torch to train the gaze model.") from exc
    features, targets = load_dataset(args.data)
    split = max(1, int(len(features) * 0.8))
    train_x = torch.from_numpy(features[:split])
    train_y = torch.from_numpy(targets[:split])
    validation_features = features[split:] if split < len(features) else features
    validation_targets = targets[split:] if split < len(targets) else targets
    validation_x = torch.from_numpy(validation_features)
    validation_y = torch.from_numpy(validation_targets)
    model = build_model(features.shape[1])
    optimizer = torch.optim.Adam(model.parameters(), lr=args.learning_rate)
    loss_function = torch.nn.MSELoss()
    loader = DataLoader(
        TensorDataset(train_x, train_y), args.batch_size, shuffle=True
    )
    best_error = float("inf")
    best_state = None
    for epoch in range(args.epochs):
        model.train()
        for batch_x, batch_y in loader:
            optimizer.zero_grad()
            loss = loss_function(model(batch_x), batch_y)
            loss.backward()
            optimizer.step()
        model.eval()
        with torch.no_grad():
            error = torch.linalg.vector_norm(
                model(validation_x) - validation_y, dim=1
            ).mean().item()
        if error < best_error:
            best_error = error
            best_state = {
                key: value.detach().clone()
                for key, value in model.state_dict().items()
            }
        if (epoch + 1) % 10 == 0 or epoch == 0:
            print(f"epoch={epoch + 1} validation_mean_error={error:.5f}")
    if best_state is not None:
        model.load_state_dict(best_state)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.jit.script(model.eval()).save(str(args.output))
    print(f"saved={args.output} normalized_mean_error={best_error:.5f}")


if __name__ == "__main__":
    main()
