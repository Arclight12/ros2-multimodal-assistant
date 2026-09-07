# Workspace calibration

Place four known ArUco markers around the tabletop in the configured order.
The calibration node maps their image centers to normalized rectangle corners,
then maps normalized coordinates to configured workspace metres and applies the
configured workspace-to-`base_link` transform. The first version assumes a
planar surface with fixed `table_z_m`.

1. Set marker IDs and physical workspace dimensions in `workspace.yaml`.
2. Set the measured robot-base transform in runtime configuration.
3. Run with `mock_mode: false` and the phone stream available.
4. Call:

```bash
ros2 service call /calibrate_workspace assistant_msgs/srv/CalibrateWorkspace "{start_calibration: true}"
```

Calibration is saved under `~/.ros/ros2_multimodal_assistant` by default, not
committed into this repository. Invalid or missing calibration leaves detection
poses unset and causes real grasp refusal. Recalibrate whenever the phone,
table, arm, or marker arrangement changes.
