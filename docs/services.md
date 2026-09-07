# Services

| Service | Type | Behavior |
|---|---|---|
| `/scan_workspace` | `assistant_msgs/ScanWorkspace` | requests a one-shot snapshot for testing/calibration; normal perception remains continuous |
| `/calibrate_workspace` | `assistant_msgs/CalibrateWorkspace` | detects configured ArUco markers, computes the homography, and saves valid runtime calibration |

Mock calibration reports that it is simulated and deliberately does not become
valid for real motion. Real calibration refuses to save a motion-capable result
until the workspace-to-robot transform is configured.
