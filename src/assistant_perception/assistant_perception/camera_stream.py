"""OpenCV helpers for opening the phone's HTTP MJPEG workspace stream.

This module stays free of ROS imports so timeout and reconnect behavior
remains testable on any machine, following the repository convention of
keeping camera policy in pure modules.
"""

from __future__ import annotations

from typing import Any

FFMPEG_HINT = (
    "Check the phone app server is running, that both devices share one "
    "Wi-Fi network, and that the URL is the direct MJPEG stream (for example "
    "IP Webcam: http://<phone-ip>:8080/video, DroidCam: "
    "http://<phone-ip>:4747/video). scripts/check_cameras.py tests this "
    "without ROS."
)


def retry_delay_s(attempt: int, base_delay_s: float, max_delay_s: float) -> float:
    """Return capped exponential reconnect backoff for one-based ``attempt``."""
    return min(base_delay_s * (2 ** max(0, attempt - 1)), max_delay_s)


def open_mjpeg_stream(
    cv2: Any,
    url: str,
    open_timeout_ms: int = 5000,
    read_timeout_ms: int = 10000,
) -> Any:
    """Open an HTTP/RTSP stream with the FFMPEG backend and bounded timeouts.

    The timeout properties are honored by the FFMPEG backend; on OpenCV
    builds without the parameters constructor overload this falls back to the
    two-argument call. Returns an opened capture, or ``None`` on failure.
    """
    timeout_params = [
        cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,
        int(open_timeout_ms),
        cv2.CAP_PROP_READ_TIMEOUT_MSEC,
        int(read_timeout_ms),
    ]
    try:
        capture = cv2.VideoCapture(url, cv2.CAP_FFMPEG, timeout_params)
    except TypeError:
        capture = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    if not capture.isOpened():
        capture.release()
        return None
    return capture


def read_frame(capture: Any) -> tuple[bool, Any]:
    """Read one frame, converting driver exceptions into a failed read."""
    try:
        return capture.read()
    except Exception:  # noqa: BLE001 - arbitrary codec/network failures
        return False, None
