"""Fuse normalized gaze and live detections into a stable target selection."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from assistant_msgs.msg import DetectedObjectArray, GazePoint, ObjectSelection

from .selection_logic import (
    DetectionObservation,
    GazeObservation,
    SelectionConfig,
    TemporalSelector,
    nearest_valid_detection,
    select_by_class,
)


class SelectionManagerNode(Node):
    """Confirm the object closest to the user's gaze over multiple frames."""

    def __init__(self) -> None:
        super().__init__("selection_manager_node")
        self.declare_parameter("selection_hold_seconds", 1.0)
        self.declare_parameter("minimum_gaze_confidence", 0.8)
        self.declare_parameter("maximum_gaze_object_distance", 0.2)
        # Equal to the detector threshold so every drawn box can be selected.
        self.declare_parameter("minimum_detection_confidence", 0.10)
        self.declare_parameter("selection_grace_seconds", 0.5)
        self.declare_parameter("voice_priority_seconds", 10.0)
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
            float(self.get_parameter("selection_hold_seconds").value),
            float(self.get_parameter("selection_grace_seconds").value),
        )
        self._voice_priority_s = float(
            self.get_parameter("voice_priority_seconds").value
        )
        self._voice_selected_at: float | None = None
        self._pending_voice_class: str | None = None
        self._last_voice_warning: float | None = None
        self._gaze: GazePoint | None = None
        self._detections: DetectedObjectArray | None = None
        self._publisher = self.create_publisher(ObjectSelection, "/final_selection", 10)
        self._gaze_subscription = self.create_subscription(
            GazePoint, "/gaze/point", self._on_gaze, 10
        )
        self._detection_subscription = self.create_subscription(
            DetectedObjectArray,
            "/object_detections",
            self._on_detections,
            10,
        )
        self._voice_subscription = self.create_subscription(
            String, "/voice/object_id", self._on_voice, 10
        )

    def _on_voice(self, message: String) -> None:
        """Select a spoken class immediately; voice outranks gaze."""
        class_name = message.data.strip().lower()
        now = self._now()
        self._voice_selected_at = now
        self._pending_voice_class = class_name
        if self._detections is None:
            self.get_logger().warning(
                f"Voice asked for '{class_name}' but nothing is detected yet."
            )
            return
        match = select_by_class(
            class_name,
            [
                self._to_observation(
                    item,
                    self._detections.image_width,
                    self._detections.image_height,
                )
                for item in self._detections.objects
            ],
            self._config,
        )
        if match is None:
            self._log_voice_unavailable(class_name)
            return
        self._pending_voice_class = None
        self._publish_selection(match, "voice", match.confidence)

    def _log_voice_unavailable(self, class_name: str) -> None:
        """Rate-limited warning when a voiced class has no live detection."""
        now = self._now()
        if (
            self._last_voice_warning is not None
            and now - self._last_voice_warning < 5.0
        ):
            return
        self._last_voice_warning = now
        if self._detections is None:
            visible = []
        else:
            visible = sorted(
                {
                    item.class_name
                    for item in self._detections.objects
                    if item.class_name
                }
            )
        names = ", ".join(visible) if visible else "none"
        self.get_logger().warning(
            f"Voice asked for '{class_name}' but it is not visible. "
            f"Visible classes: {names}."
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

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
        if self._detections is None:
            return
        now = self._now()
        observations = [
            self._to_observation(
                item,
                self._detections.image_width,
                self._detections.image_height,
            )
            for item in self._detections.objects
        ]
        if self._pending_voice_class is not None:
            if (
                self._voice_selected_at is None
                or now - self._voice_selected_at >= self._voice_priority_s
            ):
                self._pending_voice_class = None
            else:
                match = select_by_class(
                    self._pending_voice_class,
                    observations,
                    self._config,
                )
                if match is not None:
                    self._pending_voice_class = None
                    self._publish_selection(match, "voice", match.confidence)
                    return
        if self._gaze is None:
            return
        gaze = GazeObservation(
            x=float(self._gaze.x),
            y=float(self._gaze.y),
            confidence=float(self._gaze.confidence),
            valid=bool(self._gaze.valid),
        )
        candidate = nearest_valid_detection(gaze, observations, self._config)
        confirmed = self._selector.update(candidate, now)
        if confirmed is None:
            return
        if (
            self._voice_selected_at is not None
            and now - self._voice_selected_at < self._voice_priority_s
        ):
            self.get_logger().info(
                "Ignoring gaze selection: a voice command has priority."
            )
            return
        self._publish_selection(
            confirmed.detection,
            "gaze",
            min(gaze.confidence, confirmed.detection.confidence),
        )

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
        self, detection: DetectionObservation, source: str, confidence: float
    ) -> None:
        """Publish the authoritative confirmed selection."""
        message = ObjectSelection()
        message.header.stamp = self.get_clock().now().to_msg()
        message.object_id = detection.object_id
        message.source = source
        message.confidence = confidence
        message.class_name = detection.class_name
        message.image_center.x = detection.image_x
        message.image_center.y = detection.image_y
        if detection.workspace_xy is not None:
            message.workspace_center.x = detection.workspace_xy[0]
            message.workspace_center.y = detection.workspace_xy[1]
            message.has_workspace_pose = True
        if detection.robot_pose is not None:
            message.target_pose.position.x = detection.robot_pose[0]
            message.target_pose.position.y = detection.robot_pose[1]
            message.target_pose.position.z = detection.robot_pose[2]
            message.target_pose.orientation.w = 1.0
            message.has_target_pose = True
        self._publisher.publish(message)
        if source == "voice":
            workspace_text = (
                f"({detection.workspace_xy[0]:.1f},{detection.workspace_xy[1]:.1f})"
                if detection.workspace_xy is not None
                else "None"
            )
            self.get_logger().info(
                f"Voice selected {detection.class_name} {detection.object_id} "
                f"image=({detection.image_x:.1f},{detection.image_y:.1f}) "
                f"workspace={workspace_text} "
                f"has_workspace_pose={detection.workspace_xy is not None}"
            )
        self.get_logger().info(
            f"Confirmed {source} selection: {detection.object_id} "
            f"[{detection.class_name}] (confidence={message.confidence:.2f})"
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
