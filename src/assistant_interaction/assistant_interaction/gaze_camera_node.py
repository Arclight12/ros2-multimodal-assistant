"""Publish frames from the laptop webcam used only for gaze estimation."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import cv2
except ImportError:  # pragma: no cover - optional runtime dependency
    cv2 = None


class GazeCameraNode(Node):
    """Capture a configurable webcam index, or publish a mock frame."""

    def __init__(self) -> None:
        super().__init__("gaze_camera_node")
        self.declare_parameter("gaze_camera_index", 0)
        self.declare_parameter("gaze_camera_frame_id", "gaze_camera")
        self.declare_parameter("gaze_camera_fps", 15.0)
        self.declare_parameter("mock_mode", True)
        self._camera_index = int(self.get_parameter("gaze_camera_index").value)
        self._frame_id = str(self.get_parameter("gaze_camera_frame_id").value)
        self._mock_mode = bool(self.get_parameter("mock_mode").value)
        self._publisher = self.create_publisher(Image, "/gaze/image_raw", 10)
        self._capture = None
        if not self._mock_mode:
            if cv2 is None:
                self.get_logger().error(
                    "OpenCV is unavailable; gaze camera is disabled."
                )
            else:
                self._capture = cv2.VideoCapture(self._camera_index)
                if not self._capture.isOpened():
                    self.get_logger().error(
                        f"Unable to open laptop webcam index {self._camera_index}."
                    )
                    self._capture.release()
                    self._capture = None
        else:
            self.get_logger().warning(
                "Gaze camera running in deterministic mock mode."
            )
        fps = max(1.0, float(self.get_parameter("gaze_camera_fps").value))
        self._timer = self.create_timer(1.0 / fps, self._publish_next_frame)

    def _publish_next_frame(self) -> None:
        """Capture and publish one webcam image."""
        if self._mock_mode:
            message = Image()
            message.header.frame_id = self._frame_id
            message.height = 1
            message.width = 1
            message.encoding = "bgr8"
            message.step = 3
            message.data = bytes((0, 0, 0))
            self._publisher.publish(message)
            return
        if self._capture is None:
            return
        success, frame = self._capture.read()
        if not success or frame is None:
            self.get_logger().warning("Laptop webcam returned no frame.")
            return
        message = Image()
        message.header.frame_id = self._frame_id
        message.height, message.width = frame.shape[:2]
        message.encoding = "bgr8"
        message.step = int(frame.strides[0])
        message.data = frame.tobytes()
        self._publisher.publish(message)

    def shutdown_callback(self) -> None:
        """Release the webcam."""
        if self._capture is not None:
            self._capture.release()


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
