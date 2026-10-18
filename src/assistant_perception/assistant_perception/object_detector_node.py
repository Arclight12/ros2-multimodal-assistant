"""Continuous object detection for workspace camera frames."""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from assistant_msgs.msg import DetectedObject, DetectedObjectArray

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - optional runtime dependency
    CvBridge = None

try:
    import cv2
except ImportError:  # pragma: no cover - optional runtime dependency
    cv2 = None

try:
    from ultralytics import YOLO
except ImportError:
    import sys
    _venv_site = Path.home() / "robot-venv" / "lib" / "python3.12" / "site-packages"
    if _venv_site.is_dir() and str(_venv_site) not in sys.path:
        sys.path.append(str(_venv_site))
    try:
        from ultralytics import YOLO
    except ImportError:  # pragma: no cover - optional runtime dependency
        YOLO = None


DARK_MEAN = 40.0
BRIGHT_MEAN = 215.0
TARGET_MEAN = 110.0


class ObjectDetectorNode(Node):
    """Publish live bounding-box detections or deterministic mock detections."""

    def __init__(self) -> None:
        super().__init__("object_detector_node")
        self.declare_parameter("object_model_path", "")
        self.declare_parameter("object_detection_confidence", 0.55)
        self.declare_parameter("max_detections", 20)
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("mock_detection_fps", 5.0)
        self._model_path = str(self.get_parameter("object_model_path").value)
        self._confidence = float(
            self.get_parameter("object_detection_confidence").value
        )
        self._max_detections = int(self.get_parameter("max_detections").value)
        self._mock_mode = bool(self.get_parameter("mock_mode").value)
        self._busy = False
        self._publisher = self.create_publisher(
            DetectedObjectArray, "/object_detections_raw", 10
        )
        self._bridge = CvBridge() if CvBridge is not None else None
        self._model = self._load_model()
        self._subscription = self.create_subscription(
            Image, "/workspace/image_raw", self._on_image, 1
        )
        self._timer = None
        if self._mock_mode:
            fps = max(1.0, float(self.get_parameter("mock_detection_fps").value))
            self._timer = self.create_timer(1.0 / fps, self._publish_mock_detections)

    def _load_model(self):
        """Load configured weights without taking down the ROS graph."""
        if self._mock_mode:
            return None
        if YOLO is None:
            self.get_logger().error(
                "Ultralytics is unavailable; object inference is disabled."
            )
            return None
        path = Path(self._model_path).expanduser() if self._model_path else None
        if path is not None and not path.is_file():
            repo_candidate = Path(__file__).resolve().parents[3] / self._model_path
            if repo_candidate.is_file():
                path = repo_candidate
        if path is None or not path.is_file():
            self.get_logger().error(
                f"Object model not found at {self._model_path!r}; inference disabled."
            )
            return None
        try:
            return YOLO(str(path))
        except Exception as exc:  # noqa: BLE001 - optional model boundary
            self.get_logger().error(f"Failed to load object model: {exc}")
            return None

    def _publish_mock_detections(self) -> None:
        """Publish stable detections so the complete graph runs without cameras."""
        message = DetectedObjectArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "workspace_camera"
        message.image_width = 1000
        message.image_height = 1000
        message.calibrated = False
        message.objects = [
            self._mock_object("cup_0", "cup", 680.0, 340.0, 0.96),
            self._mock_object("ball_0", "ball", 180.0, 720.0, 0.91),
        ]
        self._publisher.publish(message)

    @staticmethod
    def _mock_object(object_id, class_name, image_x, image_y, confidence):
        object_message = DetectedObject()
        object_message.object_id = object_id
        object_message.class_name = class_name
        object_message.confidence = confidence
        object_message.image_center.x = image_x
        object_message.image_center.y = image_y
        return object_message

    def _on_image(self, message: Image) -> None:
        """Run the optional detector on one camera frame."""
        if self._mock_mode or self._model is None or self._bridge is None:
            return
        if self._busy:
            return
        self._busy = True
        try:
            frame = self._check_exposure(
                self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            )
            results = self._model(
                frame,
                conf=self._confidence,
                max_det=self._max_detections,
                verbose=False,
            )
            self._publish_results(message, results[0])
        except Exception as exc:  # noqa: BLE001 - third-party inference boundary
            self.get_logger().error(f"Object inference failed: {exc}")
        finally:
            self._busy = False

    def _check_exposure(self, frame):
        """Warn about unusable lighting and brighten dim frames before inference."""
        if cv2 is None:
            return frame
        mean = float(frame.mean())
        if mean < DARK_MEAN or mean > BRIGHT_MEAN:
            problem = "dark" if mean < DARK_MEAN else "overexposed"
            self.get_logger().warning(
                f"Workspace frame is {problem} (mean {mean:.0f}/255): light the "
                "table, uncover the lens, and aim away from windows and lamps.",
                throttle_duration_sec=5.0,
            )
        if 1.0 < mean < DARK_MEAN:
            return cv2.convertScaleAbs(frame, alpha=min(TARGET_MEAN / mean, 4.0))
        return frame

    def _publish_results(self, image: Image, result) -> None:
        """Translate one Ultralytics result to the shared detection message."""
        message = DetectedObjectArray()
        message.header = image.header
        message.image_width = image.width
        message.image_height = image.height
        message.calibrated = False
        names = getattr(result, "names", {})
        boxes = getattr(result, "boxes", None)
        if boxes is not None:
            for index, (box, confidence, class_id) in enumerate(
                zip(boxes.xyxy.tolist(), boxes.conf.tolist(), boxes.cls.tolist())
            ):
                x1, y1, x2, y2 = box
                object_message = DetectedObject()
                class_index = int(class_id)
                if isinstance(names, (list, tuple)):
                    class_name = str(names[class_index])
                else:
                    class_name = str(names.get(class_index, class_index))
                object_message.object_id = f"{class_name}_{index}"
                object_message.class_name = class_name
                object_message.confidence = float(confidence)
                object_message.image_center.x = (x1 + x2) / 2.0
                object_message.image_center.y = (y1 + y2) / 2.0
                object_message.bbox_width = x2 - x1
                object_message.bbox_height = y2 - y1
                message.objects.append(object_message)
        self._publisher.publish(message)

    def shutdown_callback(self) -> None:
        """Release optional detector resources."""


def main(args: list[str] | None = None) -> None:
    """Run the object detector node."""
    rclpy.init(args=args)
    node = ObjectDetectorNode()
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
