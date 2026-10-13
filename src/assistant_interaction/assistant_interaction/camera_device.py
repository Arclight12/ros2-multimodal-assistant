"""OpenCV helpers for opening the laptop webcam used for gaze estimation.

This module stays free of ROS imports so backend selection and reconnect
behavior remain testable on any machine, following the repository convention
of keeping camera policy in pure modules.
"""

from __future__ import annotations

from typing import Any

BACKEND_ATTRIBUTES = {
    "auto": "CAP_ANY",
    "any": "CAP_ANY",
    "dshow": "CAP_DSHOW",
    "msmf": "CAP_MSMF",
    "v4l2": "CAP_V4L2",
    "ffmpeg": "CAP_FFMPEG",
}

WEBCAM_HINT = (
    "Check the webcam is not held by another application, try a different "
    "gaze_camera_index (scripts/check_cameras.py --probe scans indices), or "
    "set gaze_camera_backend ('dshow' is often fastest on Windows, 'v4l2' on "
    "Linux)."
)


def retry_delay_s(attempt: int, base_delay_s: float, max_delay_s: float) -> float:
    """Return capped exponential reconnect backoff for one-based ``attempt``."""
    return min(base_delay_s * (2 ** max(0, attempt - 1)), max_delay_s)


def resolve_backend(cv2: Any, backend_name: str) -> int:
    """Map a backend name to its OpenCV capture-API constant.

    Raises ``ValueError`` for unknown names so typos fail loudly instead of
    silently opening with the wrong API.
    """
    attribute = BACKEND_ATTRIBUTES.get(str(backend_name).lower())
    if attribute is None:
        known = ", ".join(sorted(BACKEND_ATTRIBUTES))
        raise ValueError(
            f"unknown camera backend {backend_name!r}; known backends: {known}"
        )
    return int(getattr(cv2, attribute, 0))


def open_webcam(
    cv2: Any,
    source: Any,
    backend_name: str = "auto",
    width: int = 0,
    height: int = 0,
) -> Any:
    """Open a webcam by device index or path with the requested backend.

    ``source`` is an integer device index, or a device path string such as
    ``/dev/video2``. ``width``/``height`` above zero request that capture
    size from the driver; zero leaves the device default untouched. Returns
    an opened capture, or ``None`` on failure.
    """
    if isinstance(source, str):
        if not source.strip():
            return None
        source = source.strip()
    capture = cv2.VideoCapture(source, resolve_backend(cv2, backend_name))
    if not capture.isOpened():
        capture.release()
        return None
    if width > 0:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    if height > 0:
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    return capture


def read_frame(capture: Any) -> tuple[bool, Any]:
    """Read one frame, converting driver exceptions into a failed read."""
    try:
        return capture.read()
    except Exception:  # noqa: BLE001 - arbitrary driver failures
        return False, None
