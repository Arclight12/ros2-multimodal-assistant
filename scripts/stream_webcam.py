#!/usr/bin/env python3
"""Serve a local webcam as an MJPEG stream so the Pi's gaze node can read it.

Run on the laptop:  python scripts/stream_webcam.py --index 0 --port 8081
Then launch the Pi with:
  gaze_camera_device:=http://<laptop-ip>:8081/video gaze_camera_backend:=ffmpeg
Open http://<laptop-ip>:8081/video in a browser to check it first.
"""

from __future__ import annotations

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

latest_jpeg: bytes | None = None


def capture_loop(index: int, backend: str, width: int, height: int, quality: int):
    """Keep the newest JPEG in ``latest_jpeg``; clients never block the camera."""
    global latest_jpeg
    api = {"dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF}.get(backend, cv2.CAP_ANY)
    capture = cv2.VideoCapture(index, api)
    if not capture.isOpened():
        raise SystemExit(f"Cannot open webcam index {index} (backend {backend}).")
    if width > 0:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height > 0:
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    while True:
        ok, frame = capture.read()
        if not ok:
            time.sleep(0.05)
            continue
        ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok:
            latest_jpeg = jpeg.tobytes()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/video":
            self.send_error(404, "use /video")
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        last = None
        try:
            while True:
                jpeg = latest_jpeg
                if jpeg is None or jpeg is last:
                    time.sleep(0.01)
                    continue
                last = jpeg
                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode()
                    + b"\r\n\r\n"
                    + jpeg
                    + b"\r\n"
                )
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def log_message(self, *args):  # keep the console quiet
        pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--backend", default="dshow", choices=["dshow", "msmf", "any"])
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--quality", type=int, default=80)
    args = parser.parse_args()
    threading.Thread(
        target=capture_loop,
        args=(args.index, args.backend, args.width, args.height, args.quality),
        daemon=True,
    ).start()
    print(
        f"Streaming webcam {args.index} at http://0.0.0.0:{args.port}/video (Ctrl+C stops)"
    )
    ThreadingHTTPServer(("0.0.0.0", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
