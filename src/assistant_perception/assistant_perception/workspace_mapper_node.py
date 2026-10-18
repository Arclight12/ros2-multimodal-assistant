"""Map continuous image detections into the robot workspace."""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from assistant_msgs.msg import DetectedObject, DetectedObjectArray
from assistant_msgs.srv import ScanWorkspace

try:
    import cv2
    import cv2.aruco as aruco
    import numpy as np
except ImportError:
    cv2 = None
    aruco = None
    np = None

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

from .workspace_mapping import (
    load_calibration,
    map_pixel_to_base,
    map_pixel_to_workspace,
    object_contact_pixel,
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
        self.declare_parameter("workspace_width_m", 0.3115)
        self.declare_parameter("workspace_height_m", 0.1500)
        self._calibration_path = os.path.expanduser(
            str(self.get_parameter("calibration_path").value)
        )
        self._object_map_path = os.path.expanduser(
            str(self.get_parameter("object_map_path").value)
        )
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._mock_mode = bool(self.get_parameter("mock_mode").value)
        self._workspace_width = float(self.get_parameter("workspace_width_m").value)
        self._workspace_height = float(self.get_parameter("workspace_height_m").value)
        self._last_array: DetectedObjectArray | None = None
        self._warned_uncalibrated = False
        self._homography_logged = False
        self._live_homography: Any = None
        self._bridge = CvBridge() if CvBridge is not None else None
        self._aruco_dict = (
            aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            if aruco is not None
            else None
        )
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
        self._image_subscription = self.create_subscription(
            Image,
            "/workspace/image_raw",
            self._on_image,
            1,
        )
        self._scan_service = self.create_service(
            ScanWorkspace, "/scan_workspace", self._scan_workspace_callback
        )

    def _on_image(self, message: Image) -> None:
        """Track ArUco markers on the live frame to maintain an active homography."""
        if self._mock_mode or self._bridge is None or aruco is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            corners, ids, _ = aruco.detectMarkers(frame, self._aruco_dict)
            if ids is None or len(ids) == 0:
                return
            id_map = {
                int(mid): tuple(c[0].mean(axis=0))
                for mid, c in zip(ids.flatten(), corners)
            }
            found = set(id_map.keys()).intersection({0, 1, 2, 3})
            src_pts = None
            if len(found) == 4:
                src_pts = np.array(
                    [id_map[0], id_map[1], id_map[2], id_map[3]], dtype=np.float32
                )
            elif len(found) == 3:
                p = {k: np.array(id_map[k], dtype=np.float32) for k in found}
                if 2 not in found and {0, 1, 3}.issubset(found):
                    p[2] = p[1] + p[3] - p[0]
                elif 1 not in found and {0, 2, 3}.issubset(found):
                    p[1] = p[0] + p[2] - p[3]
                elif 3 not in found and {0, 1, 2}.issubset(found):
                    p[3] = p[0] + p[2] - p[1]
                elif 0 not in found and {1, 2, 3}.issubset(found):
                    p[0] = p[1] + p[3] - p[2]
                if len(p) == 4:
                    src_pts = np.array([p[0], p[1], p[2], p[3]], dtype=np.float32)

            if src_pts is not None:
                dst_pts = np.array(
                    [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
                    dtype=np.float32,
                )
                self._live_homography = cv2.getPerspectiveTransform(src_pts, dst_pts)
                if not self._homography_logged:
                    self._homography_logged = True
                    self.get_logger().info(
                        f"Active dynamic ArUco homography established from {len(found)} markers."
                    )
        except Exception:
            pass

    def _on_raw_detections(self, raw: DetectedObjectArray) -> None:
        """Map and republish one live detector frame."""
        calibration = (
            None if self._mock_mode else load_calibration(self._calibration_path)
        )
        has_planar = (self._live_homography is not None) or (
            calibration is not None and calibration.has_planar_calibration
        )
        has_robot = calibration is not None and calibration.has_robot_transform

        mapped = DetectedObjectArray()
        mapped.header = raw.header
        mapped.image_width = raw.image_width
        mapped.image_height = raw.image_height
        mapped.calibrated = has_planar
        for raw_object in raw.objects:
            mapped_object = self._copy_object(raw_object)
            if self._live_homography is not None:
                p = np.array(
                    [raw_object.image_center.x, raw_object.image_center.y, 1.0],
                    dtype=np.float32,
                )
                p_norm = self._live_homography @ p
                if abs(p_norm[2]) > 1e-12:
                    p_norm /= p_norm[2]
                    nx, ny = float(p_norm[0]), float(p_norm[1])
                    if -0.15 <= nx <= 1.15 and -0.15 <= ny <= 1.15:
                        mapped_object.workspace_center.x = nx * self._workspace_width
                        mapped_object.workspace_center.y = ny * self._workspace_height
                        mapped_object.workspace_center.z = 0.0
                        mapped_object.has_workspace_pose = True
            elif has_planar and calibration is not None:
                contact_x, contact_y = object_contact_pixel(
                    raw_object.image_center.x,
                    raw_object.image_center.y,
                    raw_object.bbox_height,
                )
                workspace = map_pixel_to_workspace(
                    contact_x,
                    contact_y,
                    calibration,
                )
                if workspace is not None:
                    mapped_object.workspace_center.x = workspace[0]
                    mapped_object.workspace_center.y = workspace[1]
                    mapped_object.workspace_center.z = 0.0
                    mapped_object.has_workspace_pose = True

                if has_robot:
                    robot = map_pixel_to_base(
                        contact_x,
                        contact_y,
                        calibration,
                    )
                    if robot is not None:
                        mapped_object.robot_pose.position.x = robot[0]
                        mapped_object.robot_pose.position.y = robot[1]
                        mapped_object.robot_pose.position.z = robot[2]
                        mapped_object.robot_pose.orientation.w = 1.0
                        mapped_object.has_robot_pose = True
            mapped.objects.append(mapped_object)
        self._last_array = mapped
        self._publisher.publish(mapped)
        if not has_planar and not self._warned_uncalibrated:
            self._warned_uncalibrated = True
            self.get_logger().warning(
                "Workspace planar calibration is invalid; detections remain image-only."
            )
        elif has_planar and not has_robot and not self._warned_uncalibrated:
            self._warned_uncalibrated = True
            self.get_logger().info(
                "Planar workspace calibration is active, but robot-base 3D transform is unverified; robot poses remain unavailable."
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
