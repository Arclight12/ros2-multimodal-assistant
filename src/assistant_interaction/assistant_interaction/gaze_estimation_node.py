"""Estimate direct normalized tabletop gaze coordinates."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from assistant_msgs.msg import GazePoint

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - optional runtime dependency
    CvBridge = None

try:
    import mediapipe as mp
except ImportError:  # pragma: no cover - optional runtime dependency
    mp = None

try:
    import torch
except ImportError:  # pragma: no cover - optional runtime dependency
    torch = None


_FEATURE_INDICES = (1, 33, 133, 159, 145, 263, 362, 386, 374, 152)


class GazeEstimationNode(Node):
    """Run the custom gaze regressor without interpreting object identity."""

    def __init__(self) -> None:
        super().__init__("gaze_estimation_node")
        self.declare_parameter("gaze_model_path", "")
        self.declare_parameter("minimum_gaze_confidence", 0.6)
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("mock_gaze_fps", 5.0)
        self._model_path = str(self.get_parameter("gaze_model_path").value)
        self._minimum_confidence = float(
            self.get_parameter("minimum_gaze_confidence").value
        )
        self._mock_mode = bool(self.get_parameter("mock_mode").value)
        self._publisher = self.create_publisher(GazePoint, "/gaze/point", 10)
        self._bridge = CvBridge() if CvBridge is not None else None
        self._model = self._load_model()
        self._face_mesh = self._load_face_mesh()
        self._subscription = self.create_subscription(
            Image, "/gaze/image_raw", self._on_image, 10
        )
        self._timer = None
        if self._mock_mode:
            fps = max(1.0, float(self.get_parameter("mock_gaze_fps").value))
            self._timer = self.create_timer(1.0 / fps, self._publish_mock_gaze)

    def _load_model(self):
        """Load a TorchScript custom regressor if configured."""
        if self._mock_mode:
            return None
        if torch is None:
            self.get_logger().error(
                "PyTorch is unavailable; gaze inference is disabled."
            )
            return None
        if not self._model_path or not Path(self._model_path).is_file():
            self.get_logger().error(
                f"Gaze model not found at {self._model_path!r}; inference disabled."
            )
            return None
        try:
            model = torch.jit.load(self._model_path, map_location="cpu")
            model.eval()
            return model
        except Exception as exc:  # noqa: BLE001 - optional model boundary
            self.get_logger().error(f"Failed to load gaze model: {exc}")
            return None

    def _load_face_mesh(self):
        """Create the optional landmark extractor used by the regressor."""
        if self._mock_mode or mp is None:
            if not self._mock_mode and mp is None:
                self.get_logger().error(
                    "MediaPipe is unavailable; gaze inference is disabled."
                )
            return None
        try:
            return mp.solutions.face_mesh.FaceMesh(
                static_image_mode=False,
                max_num_faces=1,
                refine_landmarks=True,
            )
        except Exception as exc:  # noqa: BLE001 - optional model boundary
            self.get_logger().error(
                f"Failed to initialize face landmark model: {exc}"
            )
            return None

    def _publish_mock_gaze(self) -> None:
        """Publish the fixed mock target used by deterministic graph tests."""
        message = GazePoint()
        message.header.stamp = self.get_clock().now().to_msg()
        message.x = 0.68
        message.y = 0.34
        message.confidence = 0.95
        message.valid = True
        self._publisher.publish(message)

    def _on_image(self, image: Image) -> None:
        """Infer one gaze point from one laptop frame."""
        if self._mock_mode or self._model is None or self._bridge is None:
            return
        if self._face_mesh is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")
            result = self._face_mesh.process(frame)
            if not result.multi_face_landmarks:
                self._publish_invalid(image)
                return
            landmarks = result.multi_face_landmarks[0].landmark
            features = []
            for index in _FEATURE_INDICES:
                landmark = landmarks[index]
                features.extend((landmark.x, landmark.y, landmark.z))
            prediction = self._model(
                torch.tensor([features], dtype=torch.float32)
            )[0].tolist()
            x, y = (
                max(0.0, min(1.0, float(value)))
                for value in prediction[:2]
            )
            message = GazePoint()
            message.header = image.header
            message.x = x
            message.y = y
            message.confidence = max(self._minimum_confidence, 0.9)
            message.valid = all(math.isfinite(value) for value in (x, y))
            self._publisher.publish(message)
        except Exception as exc:  # noqa: BLE001 - camera/model boundary
            self.get_logger().error(f"Gaze inference failed: {exc}")
            self._publish_invalid(image)

    def _publish_invalid(self, image: Image) -> None:
        """Tell fusion that the current frame has no usable gaze estimate."""
        message = GazePoint()
        message.header = image.header
        message.valid = False
        message.confidence = 0.0
        self._publisher.publish(message)

    def shutdown_callback(self) -> None:
        """Close the optional face landmark session."""
        if self._face_mesh is not None:
            self._face_mesh.close()


def main(args: list[str] | None = None) -> None:
    """Run the gaze estimation node."""
    rclpy.init(args=args)
    node = GazeEstimationNode()
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
