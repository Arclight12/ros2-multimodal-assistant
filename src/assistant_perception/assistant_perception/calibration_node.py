"""ArUco-based planar workspace calibration service."""

from __future__ import annotations

import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from assistant_msgs.srv import CalibrateWorkspace

from .workspace_mapping import WorkspaceCalibration, save_calibration

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - optional runtime dependency
    cv2 = None
    np = None

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - optional runtime dependency
    CvBridge = None


class CalibrationNode(Node):
    """Detect configured table markers and persist their planar homography."""

    def __init__(self) -> None:
        super().__init__("calibration_node")
        self.declare_parameter("calibration_path", "")
        self.declare_parameter("workspace_width_m", 0.4)
        self.declare_parameter("workspace_height_m", 0.3)
        self.declare_parameter("table_z_m", 0.0)
        self.declare_parameter("base_origin_x_m", 0.0)
        self.declare_parameter("base_origin_y_m", 0.0)
        self.declare_parameter("base_origin_z_m", 0.0)
        self.declare_parameter("base_yaw_rad", 0.0)
        self.declare_parameter("robot_transform_configured", False)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("marker_ids", [0, 1, 2, 3])
        self.declare_parameter("mock_mode", True)
        self._path = os.path.expanduser(
            str(self.get_parameter("calibration_path").value)
        )
        self._width = float(self.get_parameter("workspace_width_m").value)
        self._height = float(self.get_parameter("workspace_height_m").value)
        self._table_z = float(self.get_parameter("table_z_m").value)
        self._origin = (
            float(self.get_parameter("base_origin_x_m").value),
            float(self.get_parameter("base_origin_y_m").value),
            float(self.get_parameter("base_origin_z_m").value),
        )
        self._yaw = float(self.get_parameter("base_yaw_rad").value)
        self._transform_configured = bool(
            self.get_parameter("robot_transform_configured").value
        )
        self._dictionary_name = str(self.get_parameter("aruco_dictionary").value)
        self._marker_ids = [
            int(item) for item in self.get_parameter("marker_ids").value
        ]
        self._mock_mode = bool(self.get_parameter("mock_mode").value)
        self._latest_image: Image | None = None
        self._bridge = CvBridge() if CvBridge is not None else None
        self._subscription = self.create_subscription(
            Image, "/workspace/image_raw", self._on_image, 10
        )
        self._service = self.create_service(
            CalibrateWorkspace,
            "/calibrate_workspace",
            self._calibrate_callback,
        )

    def _on_image(self, image: Image) -> None:
        """Retain only the newest frame for an operator-triggered calibration."""
        self._latest_image = image

    def _calibrate_callback(
        self,
        request: CalibrateWorkspace.Request,
        response: CalibrateWorkspace.Response,
    ) -> CalibrateWorkspace.Response:
        """Run one marker capture and persist only a fully valid calibration."""
        if not request.start_calibration:
            return self._refuse(
                response,
                "No calibration requested (start_calibration=False).",
            )
        if self._mock_mode:
            response.success = True
            response.calibrated = False
            response.message = (
                "Mock calibration is intentionally not valid for motion."
            )
            return response
        if not self._transform_configured:
            return self._refuse(
                response, "workspace-to-base transform is not configured"
            )
        if cv2 is None or np is None or self._bridge is None:
            return self._refuse(
                response, "OpenCV, NumPy, or cv_bridge is unavailable"
            )
        if self._latest_image is None:
            return self._refuse(response, "no workspace camera frame is available")
        try:
            frame = self._bridge.imgmsg_to_cv2(
                self._latest_image, desired_encoding="bgr8"
            )
            homography = self._find_homography(frame)
        except Exception as exc:  # noqa: BLE001 - camera boundary
            return self._refuse(response, f"marker detection failed: {exc}")
        if homography is None:
            return self._refuse(
                response, "all configured ArUco markers were not found"
            )
        calibration = WorkspaceCalibration(
            calibrated=True,
            homography=tuple(
                tuple(float(value) for value in row) for row in homography
            ),
            workspace_width_m=self._width,
            workspace_height_m=self._height,
            table_z_m=self._table_z,
            base_origin=self._origin,
            base_yaw_rad=self._yaw,
            robot_transform_configured=True,
        )
        try:
            save_calibration(self._path, calibration)
        except OSError as exc:
            return self._refuse(response, f"failed to save calibration: {exc}")
        response.success = True
        response.calibrated = True
        response.message = f"Workspace calibration saved to {self._path}."
        return response

    def _find_homography(self, frame):
        """Return pixel-to-normalized-workspace homography from marker centers."""
        aruco = cv2.aruco
        dictionary_id = getattr(aruco, self._dictionary_name, None)
        if dictionary_id is None:
            raise ValueError(f"unknown ArUco dictionary {self._dictionary_name!r}")
        dictionary = aruco.getPredefinedDictionary(dictionary_id)
        if hasattr(aruco, "ArucoDetector"):
            detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
            corners, ids, _ = detector.detectMarkers(frame)
        else:  # pragma: no cover - older OpenCV compatibility
            corners, ids, _ = aruco.detectMarkers(frame, dictionary)
        if ids is None:
            return None
        centers = {}
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            centers[int(marker_id)] = marker_corners[0].mean(axis=0)
        if any(marker_id not in centers for marker_id in self._marker_ids):
            return None
        source = np.float32(
            [centers[marker_id] for marker_id in self._marker_ids]
        )
        destination = np.float32(
            [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
        )
        homography, _ = cv2.findHomography(source, destination)
        return homography

    @staticmethod
    def _refuse(response, message):
        response.success = False
        response.calibrated = False
        response.message = message
        return response

    def shutdown_callback(self) -> None:
        """Provide a consistent node shutdown hook."""


def main(args: list[str] | None = None) -> None:
    """Run the calibration node."""
    rclpy.init(args=args)
    node = CalibrationNode()
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
