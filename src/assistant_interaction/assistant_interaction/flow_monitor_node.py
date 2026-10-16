"""Watch the whole gaze -> detection -> selection flow and save a picture of it.

Perform next to the launched system:  ros2 run assistant_interaction flow_monitor_node
Prints the confirmed target (class, image px, workspace m, robot m) and keeps
overwriting ``output_path`` with the workspace frame, detection boxes, gaze
dot (red) and selected target (green, magenta for a voice pick). Every pick is
also written once to ``selected_output_path``, which keeps it for copying.
"""

from __future__ import annotations

from typing import Any

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from assistant_msgs.msg import DetectedObjectArray, GazePoint, ObjectSelection

from .flow_overlay import describe_selection, render_overlay

try:
    import cv2
except ImportError:  # pragma: no cover - exercised only on missing runtime deps
    cv2 = None


class FlowMonitorNode(Node):
    """Print selections and write an annotated frame of the live flow."""

    def __init__(self) -> None:
        super().__init__("flow_monitor_node")
        self.declare_parameter("output_path", "/tmp/flow.jpg")
        self.declare_parameter("selected_output_path", "/tmp/flow_selected.jpg")
        self.declare_parameter("output_period_s", 1.0)
        self.declare_parameter("selection_hold_display_s", 10.0)
        self._output_path = str(self.get_parameter("output_path").value)
        self._selected_output_path = str(
            self.get_parameter("selected_output_path").value
        )
        self._hold_s = float(self.get_parameter("selection_hold_display_s").value)
        self._bridge = CvBridge()
        self._frame = None
        self._detections: DetectedObjectArray | None = None
        self._gaze: GazePoint | None = None
        self._selected_id: str | None = None
        self._selection: dict[str, Any] | None = None
        self._selected_at = 0.0
        self.create_subscription(Image, "/workspace/image_raw", self._on_frame, 1)
        self.create_subscription(
            DetectedObjectArray, "/object_detections", self._on_detections, 10
        )
        self.create_subscription(GazePoint, "/gaze/point", self._on_gaze, 10)
        self.create_subscription(
            ObjectSelection, "/final_selection", self._on_selection, 10
        )
        self.create_timer(
            float(self.get_parameter("output_period_s").value), self._write_frame
        )
        self.get_logger().info(
            f"Writing flow picture to {self._output_path}, "
            f"each selection to {self._selected_output_path}"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_frame(self, message: Image) -> None:
        self._frame = self._bridge.imgmsg_to_cv2(message, "bgr8")

    def _on_detections(self, message: DetectedObjectArray) -> None:
        self._detections = message

    def _on_gaze(self, message: GazePoint) -> None:
        self._gaze = message

    def _on_selection(self, message: ObjectSelection) -> None:
        robot = message.target_pose.position
        self._selection = {
            "object_id": message.object_id,
            "class_name": message.class_name,
            "source": message.source,
            "confidence": message.confidence,
            "image_x": message.image_center.x,
            "image_y": message.image_center.y,
            "workspace_xy": (
                (message.workspace_center.x, message.workspace_center.y)
                if message.has_workspace_pose
                else None
            ),
            "robot_xyz": (
                (robot.x, robot.y, robot.z) if message.has_target_pose else None
            ),
        }
        self._selected_id = message.object_id
        self._selected_at = self._now()
        self.get_logger().info("\n" + describe_selection(self._selection))
        if self._frame is None or cv2 is None:
            return
        cv2.imwrite(self._selected_output_path, self._render(True))
        self.get_logger().info(
            f"Wrote selected frame to {self._selected_output_path} - copy it any time"
        )

    def _boxes(self) -> list[dict[str, Any]]:
        if self._detections is None:
            return []
        return [
            {
                "object_id": item.object_id,
                "class_name": item.class_name,
                "confidence": float(item.confidence),
                "x": float(item.image_center.x),
                "y": float(item.image_center.y),
                "w": float(item.bbox_width),
                "h": float(item.bbox_height),
            }
            for item in self._detections.objects
        ]

    def _gaze_point(self) -> tuple[float, float, bool] | None:
        if self._gaze is None:
            return None
        return (float(self._gaze.x), float(self._gaze.y), bool(self._gaze.valid))

    def _render(self, selected: bool) -> Any:
        return render_overlay(
            cv2,
            self._frame,
            self._boxes(),
            self._gaze_point(),
            self._selected_id if selected else None,
            self._selection if selected else None,
        )

    def _write_frame(self) -> None:
        if self._frame is None or cv2 is None:
            return
        cv2.imwrite(
            self._output_path,
            self._render(self._now() - self._selected_at < self._hold_s),
        )


def main(args: list[str] | None = None) -> None:
    """Run the flow monitor node."""
    rclpy.init(args=args)
    node = FlowMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
