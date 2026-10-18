# Workspace calibration

Place four known ArUco markers (`DICT_4X4_50`, IDs `[0, 1, 2, 3]`) at the board corners:
- **Marker 0**: Top-Left
- **Marker 1**: Top-Right
- **Marker 2**: Bottom-Right
- **Marker 3**: Bottom-Left

The calibration node maps their image centers to normalized rectangle corners (`[0..1] x [0..1]`),
then maps normalized coordinates to configured workspace metres ($0.3115\text{ m} \times 0.1500\text{ m}$).

### Planar Calibration vs Full Robot Transform Validity

- **Planar camera-to-workspace calibration** (2D homography) can be calibrated and tested independently of the robot arm. When planar calibration is valid, detections produce workspace-local $(X, Y)$ coordinates (`has_workspace_pose = True`).
- **3D robot-base transform**: The vertical tabletop level (`table_z_m`) and base origin height (`base_origin_z_m`) remain unverified nominal values ($0.0\text{ m}$). Therefore, `robot_transform_configured` is set to `false` in `workspace.yaml`. Robot-frame target poses (`robot_pose`, `has_robot_pose`) remain unavailable and real grasp execution is refused until the 3D transform is physically measured and verified on the robot.

### Marker ID Verification & Limitations

Geometric checks (convexity, clockwise winding, condition number) reject degenerate, non-convex, or inverted marker configurations, but **cannot detect cyclic permutations** (`[1, 2, 3, 0]`, `[2, 3, 0, 1]`, `[3, 0, 1, 2]`), which remain convex and clockwise. An operator must visually confirm on the live camera stream that each physical marker ID is located at its designated corner before capturing calibration.

### Object Contact-Point Approximation

Tabletop mapping uses `object_contact_pixel()` to estimate the ground-contact footpoint at the bottom-center of the 2D bounding box (`center_x, center_y + bbox_height / 2`). Gaze selection continues to use the visual bounding-box centroid (`image_center`). The bottom-center estimate is an approximation that requires physical validation on real objects.

### Running Calibration

1. Ensure physical marker placement matches `ID 0 = TL, ID 1 = TR, ID 2 = BR, ID 3 = BL`.
2. Run with `mock_mode: false`, `arm_mock_mode: true`, and the phone camera stream available:
   ```bash
   ros2 launch assistant_bringup system.launch.py mock_mode:=false arm_mock_mode:=true workspace_camera_url:=http://<PHONE_IP>:8080/video
   ```
3. Trigger calibration capture:
   ```bash
   ros2 service call /calibrate_workspace assistant_msgs/srv/CalibrateWorkspace "{start_calibration: true}"
   ```

Calibration is saved under `~/.ros/ros2_multimodal_assistant/workspace_calibration.json`.
Recalibrate whenever the phone, table, or marker arrangement changes.
