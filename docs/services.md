# Services

This document describes the ROS 2 services exposed by the system.

## Service Summary

| Service            | Type                             | Server            | Client        |
|--------------------|----------------------------------|-------------------|---------------|
| `/scan_workspace`  | `assistant_msgs/ScanWorkspace`   | `workspace_mapper`| CLI / UI      |

## `/scan_workspace`

Triggers a fresh scan of the tabletop workspace. The workspace mapper detects
objects and rewrites the **runtime object map** (default
`~/.ros/ros2_multimodal_assistant/object_map.json`, outside the repository).

### Mock mode

When `mock_mode: true` (the default), no camera or inference is used. The node
emits simulated detections (`cup`, `eraser`), writes a map marked
`_meta.mock: true`, and the response message is labeled `(MOCK)`. The scan
never touches version-controlled files in the repository.

### Request

```
bool start_scan
```

Set `start_scan` to `true` to begin a scan. Set to `false` to request a status
check (which returns without scanning).

### Response

```
bool success
string message
```

`success` indicates whether the scan completed without error. `message`
provides a human-readable outcome, including the number of objects detected.

### Semantics

- The service is **synchronous**; the client blocks until the scan finishes.
- On failure, `success` is `false` and the `message` explains the error.
- On success, the runtime object map file is updated and `/workspace_status` is
  published with `SCAN_COMPLETE`.
- In mock mode the success message contains a `(MOCK)` marker so consumers can
  tell simulated data from real scan data.

### Example (CLI)

```bash
ros2 service call /scan_workspace assistant_msgs/srv/ScanWorkspace "{start_scan: true}"
```

Expected output (mock mode enabled):

```
requester: making request: assistant_msgs.srv.ScanWorkspace_Request(start_scan=True)
response:
assistant_msgs.srv.ScanWorkspace_Response(success=True, message='Scan complete (MOCK). Found 2 objects.')
```
