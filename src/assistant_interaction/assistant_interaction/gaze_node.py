"""Gaze input node for the multimodal assistant.

Tracks the user's gaze using MediaPipe FaceMesh and infers which object the user
is looking at. The inferred object is published on the ``/gaze/object_id`` topic
for the selection manager to consume.

This node contains no voice, motion, or planning logic.

Topics
------
* Publishes ``/gaze/object_id`` (``assistant_msgs/ObjectSelection``).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from assistant_msgs.msg import ObjectSelection


class GazeNode(Node):
    """Infer a target object from the user's gaze and publish it."""

    def __init__(self) -> None:
        """Initialize the gaze input node."""
        super().__init__("gaze_node")

        self.declare_parameter("user_camera_url", "0")
        self.declare_parameter("facemesh_model_path", "models/face_mesh.tflite")
        self.declare_parameter("gaze_timeout_seconds", 2.0)

        self._user_camera_url: str = (
            self.get_parameter("user_camera_url").get_parameter_value().string_value
        )
        self._facemesh_model_path: str = (
            self.get_parameter("facemesh_model_path").get_parameter_value().string_value
        )
        self._gaze_timeout_seconds: float = (
            self.get_parameter("gaze_timeout_seconds")
            .get_parameter_value()
            .double_value
        )

        self._publisher = self.create_publisher(
            ObjectSelection, "/gaze/object_id", 10
        )

        # Placeholder for the MediaPipe FaceMesh model.
        self._face_mesh = None

        self.get_logger().info(
            f"GazeNode initialized | camera={self._user_camera_url} | "
            f"model={self._facemesh_model_path}"
        )

        self._setup_gaze_loop()

    def _setup_gaze_loop(self) -> None:
        """Configure the gaze tracking loop.

        TODO(interaction-team): open the user camera, run FaceMesh, map gaze
        vector to a target object, and call :meth:`_publish_object`.
        """
        self.get_logger().info("Gaze loop configured (placeholder).")

    def _publish_object(self, object_id: str) -> None:
        """Publish an inferred gaze selection on ``/gaze/object_id``.

        :param object_id: The inferred object identifier.
        """
        msg = ObjectSelection()
        msg.object_id = object_id
        msg.source = "gaze"
        self._publisher.publish(msg)
        self.get_logger().info(f"Gaze selection: {object_id}")

    def shutdown_callback(self) -> None:
        """Perform clean shutdown of the camera stream and node."""
        self.get_logger().info("Shutting down gaze node.")
        # TODO(interaction-team): release the user camera stream here.


def main(args: list[str] | None = None) -> None:
    """Entry point for the gaze node executable."""
    rclpy.init(args=args)
    node = GazeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user (Ctrl+C).")
    finally:
        node.shutdown_callback()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
