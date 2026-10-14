#!/usr/bin/env python3
"""Show the phone workspace stream and laptop webcam live, without ROS.

Opens both cameras with the same helpers the nodes use and tiles them in one
window with a measured fps overlay. Press q or Esc in the window to quit.

Example:
  python scripts/view_cameras.py --workspace-url http://192.168.1.10:8080/video
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "assistant_perception"))
sys.path.insert(0, str(REPO_ROOT / "src" / "assistant_interaction"))

from assistant_interaction.camera_device import open_webcam  # noqa: E402
from assistant_perception.camera_stream import (  # noqa: E402
    open_mjpeg_stream,
    read_frame,
)

TILE_HEIGHT = 480


class Reader(threading.Thread):
    """Keep the newest frame and fps of one capture so cameras don't stall
    each other (the phone stream is much slower than a webcam)."""

    def __init__(self, capture) -> None:
        super().__init__(daemon=True)
        self.capture = capture
        self.frame = None
        self.fps = 0.0
        self.stop = threading.Event()

    def run(self) -> None:
        count, started = 0, time.monotonic()
        while not self.stop.is_set():
            ok, frame = read_frame(self.capture)
            if not ok:
                time.sleep(0.05)
                continue
            self.frame = frame
            count += 1
            elapsed = time.monotonic() - started
            if elapsed >= 1.0:
                self.fps, count, started = count / elapsed, 0, time.monotonic()


def tile(reader: Reader | None, label: str) -> np.ndarray:
    """Scale one camera to the common height and draw its label and fps."""
    frame = None if reader is None else reader.frame
    if frame is None:
        image = np.zeros((TILE_HEIGHT, 480, 3), np.uint8)
        text = f"{label}: waiting..." if reader else f"{label}: skipped"
    else:
        scale = TILE_HEIGHT / frame.shape[0]
        image = cv2.resize(frame, None, fx=scale, fy=scale)
        text = f"{label}  {frame.shape[1]}x{frame.shape[0]}  {reader.fps:.1f} fps"
    cv2.putText(image, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
    cv2.putText(image, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return image


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace-url", default="")
    parser.add_argument("--gaze-index", type=int, default=0)
    parser.add_argument("--gaze-backend", default="auto")
    args = parser.parse_args()

    readers: dict[str, Reader | None] = {"phone": None, "webcam": None}
    if args.workspace_url:
        capture = open_mjpeg_stream(cv2, args.workspace_url)
        if capture is None:
            print("Could not open the phone stream.")
        else:
            readers["phone"] = Reader(capture)
    capture = open_webcam(cv2, args.gaze_index, args.gaze_backend)
    if capture is None:
        print("Could not open the webcam.")
    else:
        readers["webcam"] = Reader(capture)
    if not any(readers.values()):
        return 1
    for reader in readers.values():
        if reader:
            reader.start()

    while True:
        cv2.imshow(
            "cameras (q to quit)",
            np.hstack(
                [tile(readers["phone"], "phone"), tile(readers["webcam"], "webcam")]
            ),
        )
        if cv2.waitKey(30) & 0xFF in (ord("q"), 27):
            break
    for reader in readers.values():
        if reader:
            reader.stop.set()
            reader.capture.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
