from training.object_detection.train import build_parser


def test_training_defaults_to_yolo26_nano() -> None:
    args = build_parser().parse_args(["--data", "data.yaml"])

    assert args.base_model == "yolo26n.pt"
    assert args.device is None
    assert args.workers == 2


def test_training_exposes_modal_friendly_controls() -> None:
    args = build_parser().parse_args(
        [
            "--data",
            "data.yaml",
            "--batch-size",
            "32",
            "--device",
            "0",
            "--project",
            "/mnt/data/runs",
            "--name",
            "objects",
            "--exist-ok",
        ]
    )

    assert args.batch_size == 32
    assert args.device == "0"
    assert args.project == "/mnt/data/runs"
    assert args.name == "objects"
    assert args.exist_ok is True
