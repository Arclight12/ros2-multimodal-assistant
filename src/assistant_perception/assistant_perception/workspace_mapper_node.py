"""Perception node for scanning the tabletop workspace and generating the object map.

This node is responsible for workspace understanding only. It abstracts the
camera input, provides structural integration points for ArUco marker detection
and YOLO object detection, and writes the resulting object coordinates to a JSON
map consumed by the motion planner.

No motion planning or interaction logic lives in this package.

Responsibilities
----------------
* Camera abstraction (placeholder for a phone IP camera stream).
* ArUco marker detection integration point (TODO).
* YOLO object detection integration point (TODO).
* Workspace frame estimation from ArUco markers (TODO).
* Object coordinate mapping and runtime ``object_map.json`` generation.
* Providing a ``/scan_workspace`` service.

Topics
------
* Publishes ``/object_detections`` with raw detection results.
* Publishes ``/workspace_status`` with textual status.

Services
--------
* ``/scan_workspace`` : triggers a workspace scan and map generation.
"""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

import rclpy
from rclpy.node import Node

from assistant_msgs.srv import ScanWorkspace

from std_msgs.msg import String


class WorkspaceMapperNode(Node):
    """Orchestrates a tabletop scan and maintains the object map."""

    def __init__(self) -> None:
        """Initialize the workspace mapper node and its resources."""
        super().__init__("workspace_mapper_node")

        # Declare parameters with sensible defaults. These are overridable
        # through the YAML configuration file and the launch file.
        self.declare_parameter("camera_url", "http://192.168.1.100:8080/video")
        self.declare_parameter(
            "object_map_path",
            "~/.ros/ros2_multimodal_assistant/object_map.json",
        )
        self.declare_parameter(
            "example_map_path",
            "",
        )
        self.declare_parameter("workspace_width", 40.0)
        self.declare_parameter("workspace_height", 30.0)
        self.declare_parameter("default_object_height", 5.0)
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("debug_mode", False)

        self._camera_url: str = (
            self.get_parameter("camera_url").get_parameter_value().string_value
        )
        self._object_map_path: str = os.path.expanduser(
            self.get_parameter("object_map_path").get_parameter_value().string_value
        )
        self._example_map_path: str = os.path.expanduser(
            self.get_parameter("example_map_path").get_parameter_value().string_value
        )
        self._workspace_width: float = (
            self.get_parameter("workspace_width").get_parameter_value().double_value
        )
        self._workspace_height: float = (
            self.get_parameter("workspace_height").get_parameter_value().double_value
        )
        self._default_object_height: float = (
            self.get_parameter("default_object_height")
            .get_parameter_value()
            .double_value
        )
        self._mock_mode: bool = (
            self.get_parameter("mock_mode").get_parameter_value().bool_value
        )
        self._debug_mode: bool = (
            self.get_parameter("debug_mode").get_parameter_value().bool_value
        )

        self._object_map: Dict[str, Any] = {}

        self._seed_runtime_map_from_example()

        # Publishers for external visibility into the perception state.
        self._detections_pub = self.create_publisher(String, "/object_detections", 10)
        self._status_pub = self.create_publisher(String, "/workspace_status", 10)

        # Service server to trigger scans.
        self._scan_service = self.create_service(
            ScanWorkspace, "/scan_workspace", self._scan_workspace_callback
        )

        self.get_logger().info(
            "WorkspaceMapperNode initialized | "
            f"camera={self._camera_url} | object_map={self._object_map_path} | "
            f"mock_mode={self._mock_mode}"
        )

    def _seed_runtime_map_from_example(self) -> None:
        """Copy the packaged example map to the runtime location if missing.

        The runtime object map lives outside the repository (see
        ``object_map_path``) so scans never modify version-controlled files.
        On first boot it is seeded from the example map shipped by
        ``assistant_bringup`` so downstream nodes always find a valid map.
        """
        runtime_path = Path(self._object_map_path)
        if runtime_path.exists():
            return
        if not self._example_map_path:
            self.get_logger().warn(
                "No example_map_path configured and no runtime map present; "
                f"leaving object map unseeded at {runtime_path}"
            )
            return
        example_path = Path(self._example_map_path)
        if not example_path.exists():
            self.get_logger().warn(f"Example map not found at {example_path}")
            return
        runtime_path.parent.mkdir(parents=True, exist_ok=True)
        runtime_path.write_text(
            example_path.read_text(encoding="utf-8"), encoding="utf-8"
        )
        self.get_logger().info(f"Seeded runtime object map from {example_path}")

    def _scan_workspace_callback(
        self,
        request: ScanWorkspace.Request,
        response: ScanWorkspace.Response,
    ) -> ScanWorkspace.Response:
        """Handle an incoming ``/scan_workspace`` service request.

        :param request: The service request containing the scan flag.
        :param response: The response to populate with success and message.
        :return: The populated service response.
        """
        self.get_logger().info("Received scan request.")

        if not request.start_scan:
            response.success = False
            response.message = "No scan requested (start_scan=False)."
            return response

        try:
            self._publish_status("SCANNING")
            self._perform_scan()
            self._object_map = self._generate_object_map(self._capture_detections())
            self._write_object_map(self._object_map)
            self._publish_status("SCAN_COMPLETE")
            response.success = True
            if self._mock_mode:
                mode_note = " (MOCK)"
                self.get_logger().warn(
                    "mock_mode=True: scan detections are simulated. "
                    "No camera or inference is used."
                )
            else:
                mode_note = ""
            response.message = (
                f"Scan complete{mode_note}. Found {len(self._object_map)} objects."
            )
        except Exception as exc:  # noqa: BLE001 - surface any scan failure to client
            self.get_logger().error(f"Scan failed: {exc}")
            self._publish_status("SCAN_ERROR")
            response.success = False
            response.message = f"Scan failed: {exc}"

        return response

    def _perform_scan(self) -> None:
        """Execute the full scan pipeline.

        Currently a skeleton. This is the primary extension point for:
        * camera frame capture,
        * ArUco marker detection,
        * YOLO object detection.

        TODO(perception-team): integrate camera capture from ``self._camera_url``.
        TODO(perception-team): integrate ArUco detection to build workspace frame.
        TODO(perception-team): integrate YOLOv8n inference for object detection.
        """
        self.get_logger().info("Performing workspace scan (skeleton).")
        # Delay placeholder to simulate a synchronous scan.
        self._scan_simulated()

    def _scan_simulated(self) -> None:
        """Simulate the timing of a real scan without doing any inference."""
        self.get_logger().debug("Simulating scan...")

    def _capture_detections(self) -> Dict[str, Any]:
        """Capture or generate raw detections.

        :return: A mapping of object name to pixel/frame detection data.
        """
        # TODO(perception-team): replace with real YOLO detections.
        detections = {
            "cup": {"x": 12.0, "y": 8.0},
            "eraser": {"x": 5.0, "y": 14.0},
        }
        if self._mock_mode:
            self.get_logger().warn(
                "mock_mode=True: emitting simulated detections (cup, eraser)."
            )
        self._detections_pub.publish(String(data=json.dumps(detections)))
        return detections

    def _generate_object_map(self, detections: Dict[str, Any]) -> Dict[str, Any]:
        """Map detected objects into workspace coordinates.

        :param detections: Raw detections keyed by object name.
        :return: A mapping of object name to workspace coordinates.
        """
        object_map: Dict[str, Any] = {}
        for name, data in detections.items():
            object_map[name] = {
                "x": float(data.get("x", 0.0)),
                "y": float(data.get("y", 0.0)),
                "z": float(self._default_object_height),
            }
        return object_map

    def _write_object_map(self, object_map: Dict[str, Any]) -> None:
        """Persist the generated object map to the configured JSON path.

        The file includes a ``_meta`` provenance block that records whether
        the data is simulated (mock) and whether it has been calibrated, so
        placeholder coordinates can never be mistaken for real robot
        coordinates by downstream consumers.

        :param object_map: The object map (name -> pose) to write to disk.
        """
        now = datetime.now(timezone.utc).isoformat(timespec="seconds")
        document = {
            "_meta": {
                "mock": self._mock_mode,
                "calibrated": False,
                "source": "runtime_scan" if not self._mock_mode else "mock_scan",
                "units": "cm",
                "coordinate_frame": "workspace_base",
                "last_scan_time": now if not self._mock_mode else None,
            },
            "objects": object_map,
        }
        path = Path(self._object_map_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as handle:
            json.dump(document, handle, indent=4)
        self.get_logger().info(f"Object map written to {path}")

    def _publish_status(self, status: str) -> None:
        """Publish a textual status update on ``/workspace_status``.

        :param status: The status string to publish.
        """
        self._status_pub.publish(String(data=status))
        self.get_logger().info(f"Workspace status: {status}")

    def shutdown_callback(self) -> None:
        """Perform clean shutdown tasks.

        Ensures the node unregisters cleanly and destroys its logger state.
        """
        self.get_logger().info("Shutting down workspace mapper node.")


def main(args: list[str] | None = None) -> None:
    """Entry point for the workspace mapper node executable."""
    rclpy.init(args=args)
    node = WorkspaceMapperNode()

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
