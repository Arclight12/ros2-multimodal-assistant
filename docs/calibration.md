# Calibration Guide

Calibration maps the phone camera's pixel coordinates into a stable **workspace
coordinate frame** so the motion planner can operate in physical (centimeter)
units.

## Overview

The workspace mapper uses **ArUco markers** to define the workspace frame. A
printed marker (we recommend a 6x6 dict, id 0, 5 cm square) is placed inside
the camera's field of view and establishes the origin and axes of the tabletop.

## Physical Setup

1. Place the printed ArUco marker flat on the table.
2. Position the phone camera overhead at a fixed height and angle.
3. Keep the workspace within the camera's undistorted view.
4. Record the camera URL (IP stream) into `config/system.yaml`:

```yaml
camera_url: "http://192.168.1.100:8080/video"
```

## Workspace Parameters

These parameters define the physical table dimensions stored in the map:

| Parameter               | Default | Meaning                      |
|-------------------------|---------|------------------------------|
| `workspace_width`       | 40.0    | Table width in cm            |
| `workspace_height`      | 30.0    | Table depth in cm            |
| `default_object_height` | 5.0     | Assumed object height in cm  |

## Procedure

### Step 1 - Camera Check

Confirm the IP stream is reachable:

```bash
curl -I http://192.168.1.100:8080/video
```

### Step 2 - Detect the Marker Frame

`workspace_mapper_node` currently provides the ArUco integration point in
`_perform_scan()`. Implement marker detection using
`cv2.aruco.ArucoDetector` from the contrib package. The detected corners define
a homography from image pixels to workspace coordinates.

### Step 3 - Map Pixel to Workspace Coordinates

Transform each detected object's bounding-box center using the homography:

```python
# Pseudocode - implement inside workspace_mapper_node
homography, _ = cv2.findHomography(marker_corners, workspace_corners)
x, y, _ = cv2.decomposeHomographyMat(homography ...)
```

### Step 4 - Write the Object Map

A real (non-mock) scan writes the **runtime** object map at
`~/.ros/ros2_multimodal_assistant/object_map.json`. Repository files are never
modified by scans. The file uses a `{_meta, objects}` schema with provenance
metadata:

```json
{
  "_meta": {
    "mock": false,
    "calibrated": true,
    "source": "runtime_scan",
    "units": "cm",
    "coordinate_frame": "workspace_base",
    "last_scan_time": "2026-01-01T12:00:00+00:00"
  },
  "objects": {
    "cup": { "x": 12.0, "y": 8.0, "z": 5.0 }
  }
}
```

Coordinates are in centimeters relative to the workspace origin. A scan with
`mock_mode: true` writes `_meta.mock: true` and `_meta.calibrated: false`;
while `calibrated` is `false` the motion planner refuses any real arm motion.

The version-controlled example at `assistant_bringup/config/object_map_example.json`
is sample data only and is marked `_meta.calibrated: false`.

## Validation

- The ArUco frame must stay fixed during a scan.
- Place a known object at a measured position and verify the map matches within
  ~1-2 cm.
- Confirm the written map has `_meta.calibrated: true` and `_meta.mock: false`
  before any real motion is attempted.
- Recalibrate whenever the camera is moved.

## Troubleshooting

| Symptom            | Likely Cause / Fix                                      |
|--------------------|---------------------------------------------------------|
| No detections      | Camera URL unreachable; check IP stream with `curl`.    |
| Skewed coordinates | Marker not flat, or camera height/angle changed.        |
| Persistent errors  | Enable `debug_mode: true` for verbose logs.             |

See `PROJECT_SPEC.md` (Workspace Scanning Pipeline) for the upstream context.