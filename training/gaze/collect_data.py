"""Collect grid-labelled webcam frames for direct gaze regression."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument(
        "--output", type=Path, default=Path("datasets/gaze/raw")
    )
    parser.add_argument("--grid-size", type=int, default=5)
    parser.add_argument("--frames-per-point", type=int, default=10)
    parser.add_argument("--prepare-seconds", type=float, default=1.0)
    parser.add_argument("--session", default=datetime.now().strftime("%Y%m%d-%H%M%S"))
    parser.add_argument("--lighting", default="unspecified")
    return parser.parse_args()


def main() -> None:
    """Capture labelled frames after the user looks at each grid point."""
    args = parse_args()
    try:
        import cv2
    except ImportError as exc:
        raise SystemExit(
            "Install opencv-contrib-python to collect gaze data."
        ) from exc
    if args.grid_size < 2 or args.frames_per_point < 1:
        raise SystemExit(
            "grid-size must be at least 2 and frames-per-point positive"
        )
    capture = cv2.VideoCapture(args.camera_index)
    if not capture.isOpened():
        raise SystemExit(f"Unable to open webcam index {args.camera_index}")
    session_dir = args.output / args.session
    session_dir.mkdir(parents=True, exist_ok=True)
    labels_path = session_dir / "labels.jsonl"
    points = [
        (column / (args.grid_size - 1), row / (args.grid_size - 1))
        for row in range(args.grid_size)
        for column in range(args.grid_size)
    ]
    try:
        with labels_path.open("a", encoding="utf-8") as labels:
            frame_index = 0
            for point_index, (x, y) in enumerate(points):
                while True:
                    success, frame = capture.read()
                    if not success:
                        raise RuntimeError("webcam frame capture failed")
                    display = frame.copy()
                    cv2.putText(
                        display,
                        f"Look at target {point_index + 1}/{len(points)} "
                        f"({x:.2f}, {y:.2f}) - press SPACE",
                        (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )
                    cv2.imshow("gaze data collection", display)
                    if cv2.waitKey(1) & 0xFF == ord(" "):
                        break
                time.sleep(max(0.0, args.prepare_seconds))
                for _ in range(args.frames_per_point):
                    success, frame = capture.read()
                    if not success:
                        continue
                    image_name = f"frame_{frame_index:06d}.jpg"
                    cv2.imwrite(str(session_dir / image_name), frame)
                    labels.write(
                        json.dumps(
                            {
                                "image": image_name,
                                "target_x": x,
                                "target_y": y,
                                "session": args.session,
                                "lighting": args.lighting,
                                "head_position": "self-selected",
                                "captured_at": datetime.now(
                                    timezone.utc
                                ).isoformat(),
                            }
                        )
                        + "\n"
                    )
                    labels.flush()
                    frame_index += 1
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
