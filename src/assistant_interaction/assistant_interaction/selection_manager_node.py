"""Fuse normalized gaze and live detections into a stable target selection."""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from assistant_msgs.msg import DetectedObjectArray, GazePoint, ObjectSelection

from .selection_logic import (
    DetectionObservation,
    GazeObservation,
    SelectionConfig,
    TemporalSelector,
    nearest_valid_detection,
)


class SelectionManagerNode(Node):
    """Confirm the object closest to the user's gaze over multiple frames."""

    def __init__(self) -> None:
        super().__init__("selection_manager_node")
        self.declare_parameter("selection_hold_seconds", 1.0)
        self.declare_parameter("minimum_gaze_confidence", 0.8)
        self.declare_parameter("maximum_gaze_object_distance", 0.15)
        self.declare_parameter("minimum_detection_confidence", 0.7)
        self._config = SelectionConfig(
            minimum_gaze_confidence=float(
                self.get_parameter("minimum_gaze_confidence").value
            ),
            minimum_detection_confidence=float(
                self.get_parameter("minimum_detection_confidence").value
            ),
            maximum_gaze_object_distance=float(
                self.get_parameter("maximum_gaze_object_distance").value
            ),
        )
        self._selector = TemporalSelector(
            float(self.get_parameter("selection_hold_seconds").value)
        )
        self._gaze: GazePoint | None = None
        self._detections: DetectedObjectArray | None = None
        self._publisher = self.create_publisher(
            ObjectSelection, "/final_selection", 10
        )
        self._gaze_subscription = self.create_subscription(
            GazePoint, "/gaze/point", self._on_gaze, 10
        )
        self._detection_subscription = self.create_subscription(
            DetectedObjectArray,
            "/object_detections",
            self._on_detections,
            10,
        )

    def _on_gaze(self, message: GazePoint) -> None:
        """Update gaze and attempt a new stable selection."""
        self._gaze = message
        self._try_select()

    def _on_detections(self, message: DetectedObjectArray) -> None:
        """Update the live scene and attempt a new stable selection."""
        self._detections = message
        self._try_select()

    def _try_select(self) -> None:
        """Run confidence, distance, and temporal selection gates."""
        if self._gaze is None or self._detections is None:
            return
        gaze = GazeObservation(
            x=float(self._gaze.x),
            y=float(self._gaze.y),
            confidence=float(self._gaze.confidence),
            valid=bool(self._gaze.valid),
        )
        observations = [
            self._to_observation(
                item,
                self._detections.image_width,
                self._detections.image_height,
            )
            for item in self._detections.objects
        ]
        candidate = nearest_valid_detection(gaze, observations, self._config)
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        confirmed = self._selector.update(candidate, now)
        if confirmed is not None:
            self._publish_selection(confirmed.detection, gaze)

    @staticmethod
    def _to_observation(
        message, image_width: int, image_height: int
    ) -> DetectionObservation:
        """Convert a ROS detection into the pure fusion representation."""
        workspace_xy = None
        if message.has_workspace_pose:
            workspace_xy = (
                float(message.workspace_center.x),
                float(message.workspace_center.y),
            )
        robot_pose = None
        if message.has_robot_pose:
            robot_pose = (
                float(message.robot_pose.position.x),
                float(message.robot_pose.position.y),
                float(message.robot_pose.position.z),
            )
        return DetectionObservation(
            object_id=message.object_id,
            class_name=message.class_name,
            confidence=float(message.confidence),
            image_x=float(message.image_center.x),
            image_y=float(message.image_center.y),
            image_width=float(image_width),
            image_height=float(image_height),
            workspace_xy=workspace_xy,
            robot_pose=robot_pose,
        )

    def _publish_selection(
        self, detection: DetectionObservation, gaze: GazeObservation
    ) -> None:
        """Publish the authoritative confirmed selection."""
        message = ObjectSelection()
        message.header.stamp = self.get_clock().now().to_msg()
        message.object_id = detection.object_id
        message.source = "gaze"
        message.confidence = min(gaze.confidence, detection.confidence)
        if detection.robot_pose is not None:
            message.target_pose.position.x = detection.robot_pose[0]
            message.target_pose.position.y = detection.robot_pose[1]
            message.target_pose.position.z = detection.robot_pose[2]
            message.target_pose.orientation.w = 1.0
            message.has_target_pose = True
        self._publisher.publish(message)
        self.get_logger().info(
            f"Confirmed gaze selection: {detection.object_id} "
            f"(confidence={message.confidence:.2f})"
        )

    def shutdown_callback(self) -> None:
        """Provide a consistent node shutdown hook."""


def main(args: list[str] | None = None) -> None:
    """Run the selection manager node."""
    rclpy.init(args=args)
    node = SelectionManagerNode()
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
