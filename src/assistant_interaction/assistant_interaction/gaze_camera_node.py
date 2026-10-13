"""Publish frames from the laptop webcam used only for gaze estimation."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .camera_device import (
    WEBCAM_HINT,
    open_webcam,
    read_frame,
    retry_delay_s,
)

try:
    import cv2
except ImportError:  # pragma: no cover - optional runtime dependency
    cv2 = None

WARNING_INTERVAL_S = 10.0


class GazeCameraNode(Node):
    """Capture a configurable webcam, reconnecting when frames stop."""

    def __init__(self) -> None:
        super().__init__("gaze_camera_node")
        self.declare_parameter("gaze_camera_index", 0)
        self.declare_parameter("gaze_camera_device", "")
        self.declare_parameter("gaze_camera_backend", "auto")
        self.declare_parameter("gaze_camera_width", 0)
        self.declare_parameter("gaze_camera_height", 0)
        self.declare_parameter("gaze_camera_frame_id", "gaze_camera")
        self.declare_parameter("gaze_camera_fps", 15.0)
        self.declare_parameter("gaze_camera_reconnect_base_delay_s", 1.0)
        self.declare_parameter("gaze_camera_reconnect_max_delay_s", 30.0)
        self.declare_parameter("mock_mode", True)
        self._camera_index = int(self.get_parameter("gaze_camera_index").value)
        self._device = str(self.get_parameter("gaze_camera_device").value)
        self._backend_name = str(self.get_parameter("gaze_camera_backend").value)
        self._width = int(self.get_parameter("gaze_camera_width").value)
        self._height = int(self.get_parameter("gaze_camera_height").value)
        self._frame_id = str(self.get_parameter("gaze_camera_frame_id").value)
        self._mock_mode = bool(self.get_parameter("mock_mode").value)
        self._reconnect_base_s = float(
            self.get_parameter("gaze_camera_reconnect_base_delay_s").value
        )
        self._reconnect_max_s = float(
            self.get_parameter("gaze_camera_reconnect_max_delay_s").value
        )
        self._publisher = self.create_publisher(Image, "/gaze/image_raw", 10)
        self._capture = None
        self._open_attempts = 0
        self._next_open_time = 0.0
        self._last_warning_time = 0.0
        self._reconnect_enabled = False

        if not self._mock_mode:
            if cv2 is None:
                self.get_logger().error(
                    "OpenCV is unavailable; gaze camera is disabled."
                )
            else:
                self._reconnect_enabled = True
                self._open_capture()
        else:
            self.get_logger().warning(
                "Gaze camera running in deterministic mock mode."
            )
        fps = max(1.0, float(self.get_parameter("gaze_camera_fps").value))
        self._timer = self.create_timer(1.0 / fps, self._publish_next_frame)

    @property
    def _source(self):
        """Return the configured device path, falling back to the index."""
        return self._device if self._device.strip() else self._camera_index

    def _describe_source(self) -> str:
        """Return a human-readable name for the configured camera source."""
        if self._device.strip():
            return f"device {self._device.strip()!r}"
        return f"index {self._camera_index}"

    def _open_capture(self) -> None:
        """Attempt one webcam open and schedule a retry when it fails."""
        self._schedule_reopen()
        capture = open_webcam(
            cv2,
            self._source,
            backend_name=self._backend_name,
            width=self._width,
            height=self._height,
        )
        if capture is None:
            self._warn_throttled(
                f"Unable to open laptop webcam {self._describe_source()} with "
                f"backend {self._backend_name!r} "
                f"(attempt {self._open_attempts}). {WEBCAM_HINT}"
            )
            return
        self._capture = capture
        self._open_attempts = 0
        self.get_logger().info(
            f"Opened laptop webcam {self._describe_source()} with backend "
            f"{self._backend_name!r}."
        )

    def _schedule_reopen(self) -> None:
        """Queue the next open attempt with capped exponential backoff."""
        self._open_attempts += 1
        self._next_open_time = time.monotonic() + retry_delay_s(
            self._open_attempts, self._reconnect_base_s, self._reconnect_max_s
        )

    def _publish_next_frame(self) -> None:
        """Capture and publish one webcam image."""
        if self._mock_mode:
            self._publish_frame(None)
            return
        if self._capture is None:
            if self._reconnect_enabled and time.monotonic() >= self._next_open_time:
                self._open_capture()
            return
        success, frame = read_frame(self._capture)
        if not success or frame is None:
            self._warn_throttled(
                "Laptop webcam stopped delivering frames; reconnecting."
            )
            self._release_capture()
            return
        self._publish_frame(frame)

    def _publish_frame(self, frame) -> None:
        """Publish a BGR frame, or the deterministic mock frame when None."""
        message = Image()
        message.header.frame_id = self._frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        if frame is None:
            message.height = 1
            message.width = 1
            message.encoding = "bgr8"
            message.step = 3
            message.data = bytes((0, 0, 0))
        else:
            message.height, message.width = frame.shape[:2]
            message.encoding = "bgr8"
            message.is_bigendian = False
            message.step = int(frame.strides[0])
            message.data = frame.tobytes()
        self._publisher.publish(message)

    def _release_capture(self) -> None:
        """Close a broken device and queue its first reconnect attempt."""
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        self._schedule_reopen()

    def _warn_throttled(self, message: str) -> None:
        """Log at most one warning every ``WARNING_INTERVAL_S`` seconds."""
        now = time.monotonic()
        if now - self._last_warning_time < WARNING_INTERVAL_S:
            return
        self._last_warning_time = now
        self.get_logger().warning(message)

    def shutdown_callback(self) -> None:
        """Release the webcam."""
        if self._capture is not None:
            self._capture.release()
            self._capture = None


def main(args: list[str] | None = None) -> None:
    """Run the gaze camera node."""
    rclpy.init(args=args)
    node = GazeCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_callback()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
