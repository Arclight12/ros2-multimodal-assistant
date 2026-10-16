"""Map continuous image detections into the robot workspace."""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from assistant_msgs.msg import DetectedObject, DetectedObjectArray
from assistant_msgs.srv import ScanWorkspace

from .workspace_mapping import (
    load_calibration,
    map_pixel_to_base,
    map_pixel_to_workspace,
)


class WorkspaceMapperNode(Node):
    """Add workspace and robot coordinates to live detector output."""

    def __init__(self) -> None:
        super().__init__("workspace_mapper_node")
        self.declare_parameter("calibration_path", "")
        self.declare_parameter(
            "object_map_path", "~/.ros/ros2_multimodal_assistant/object_map.json"
        )
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("mock_mode", True)
        self._calibration_path = os.path.expanduser(
            str(self.get_parameter("calibration_path").value)
        )
        self._object_map_path = os.path.expanduser(
            str(self.get_parameter("object_map_path").value)
        )
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._mock_mode = bool(self.get_parameter("mock_mode").value)
        self._last_array: DetectedObjectArray | None = None
        self._warned_uncalibrated = False
        self._publisher = self.create_publisher(
            DetectedObjectArray, "/object_detections", 10
        )
        self._status_publisher = self.create_publisher(String, "/workspace_status", 10)
        self._subscription = self.create_subscription(
            DetectedObjectArray,
            "/object_detections_raw",
            self._on_raw_detections,
            10,
        )
        self._scan_service = self.create_service(
            ScanWorkspace, "/scan_workspace", self._scan_workspace_callback
        )

    def _on_raw_detections(self, raw: DetectedObjectArray) -> None:
        """Map and republish one live detector frame."""
        calibration = load_calibration(self._calibration_path)
        mapped = DetectedObjectArray()
        mapped.header = raw.header
        mapped.image_width = raw.image_width
        mapped.image_height = raw.image_height
        mapped.calibrated = calibration is not None and calibration.is_valid
        for raw_object in raw.objects:
            mapped_object = self._copy_object(raw_object)
            if calibration is not None and calibration.is_valid:
                workspace = map_pixel_to_workspace(
                    raw_object.image_center.x,
                    raw_object.image_center.y,
                    calibration,
                )
                robot = map_pixel_to_base(
                    raw_object.image_center.x,
                    raw_object.image_center.y,
                    calibration,
                )
                if workspace is not None and robot is not None:
                    mapped_object.workspace_center.x = workspace[0]
                    mapped_object.workspace_center.y = workspace[1]
                    mapped_object.robot_pose.position.x = robot[0]
                    mapped_object.robot_pose.position.y = robot[1]
                    mapped_object.robot_pose.position.z = robot[2]
                    mapped_object.robot_pose.orientation.w = 1.0
                    mapped_object.has_workspace_pose = True
                    mapped_object.has_robot_pose = True
                    mapped_object.header.frame_id = self._base_frame
            mapped.objects.append(mapped_object)
        self._last_array = mapped
        self._publisher.publish(mapped)
        if not mapped.calibrated and not self._warned_uncalibrated:
            self._warned_uncalibrated = True
            self.get_logger().warning(
                "Workspace calibration is invalid; detections remain image-only."
            )

    @staticmethod
    def _copy_object(raw_object: DetectedObject) -> DetectedObject:
        """Copy raw fields while avoiding mutation of the detector message."""
        mapped = DetectedObject()
        mapped.header = raw_object.header
        mapped.object_id = raw_object.object_id
        mapped.class_name = raw_object.class_name
        mapped.confidence = raw_object.confidence
        mapped.image_center = raw_object.image_center
        mapped.bbox_width = raw_object.bbox_width
        mapped.bbox_height = raw_object.bbox_height
        return mapped

    def _scan_workspace_callback(
        self,
        request: ScanWorkspace.Request,
        response: ScanWorkspace.Response,
    ) -> ScanWorkspace.Response:
        """Export the most recent live frame for inspection or legacy tooling."""
        if not request.start_scan:
            response.success = False
            response.message = "No scan requested (start_scan=False)."
            return response
        self._publish_status("SCANNING")
        if self._last_array is None:
            response.success = False
            response.message = "No live detections available yet."
            self._publish_status("SCAN_ERROR")
            return response
        try:
            self._write_object_map(self._last_array)
        except OSError as exc:
            response.success = False
            response.message = f"Scan export failed: {exc}"
            self._publish_status("SCAN_ERROR")
            return response
        self._publish_status("SCAN_COMPLETE")
        mode = " (MOCK)" if self._mock_mode else ""
        response.success = True
        response.message = (
            f"Live scan export complete{mode}. "
            f"Found {len(self._last_array.objects)} objects."
        )
        return response

    def _write_object_map(self, detections: DetectedObjectArray) -> None:
        """Write a provenance-bearing snapshot outside the repository."""
        objects = {}
        for object_message in detections.objects:
            item = {
                "class_name": object_message.class_name,
                "confidence": float(object_message.confidence),
                "image_x": float(object_message.image_center.x),
                "image_y": float(object_message.image_center.y),
            }
            if object_message.has_workspace_pose:
                item["workspace_x_m"] = float(object_message.workspace_center.x)
                item["workspace_y_m"] = float(object_message.workspace_center.y)
            if object_message.has_robot_pose:
                item["robot_pose"] = {
                    "x": float(object_message.robot_pose.position.x),
                    "y": float(object_message.robot_pose.position.y),
                    "z": float(object_message.robot_pose.position.z),
                }
            objects[object_message.object_id] = item
        document = {
            "_meta": {
                "mock": self._mock_mode,
                "calibrated": detections.calibrated,
                "source": "continuous_snapshot",
                "units": "m",
                "coordinate_frame": self._base_frame,
                "last_scan_time": datetime.now(timezone.utc).isoformat(
                    timespec="seconds"
                ),
            },
            "objects": objects,
        }
        path = Path(self._object_map_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(document, indent=2), encoding="utf-8")

    def _publish_status(self, value: str) -> None:
        """Publish a human-readable workspace status."""
        self._status_publisher.publish(String(data=value))

    def shutdown_callback(self) -> None:
        """Provide a consistent node shutdown hook."""


def main(args: list[str] | None = None) -> None:
    """Run the workspace mapper node."""
    rclpy.init(args=args)
    node = WorkspaceMapperNode()
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
