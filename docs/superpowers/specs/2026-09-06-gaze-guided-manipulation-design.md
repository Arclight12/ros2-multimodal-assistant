# Gaze-Guided Manipulation Design

## Status

Approved for implementation on 2026-09-06.

## Goal

Allow a user to look at an object on a fixed tabletop and have the system
identify that object, map its phone-camera pixel location into the robot base
frame, and execute a safe grasp through the existing five-package ROS 2
architecture.

## Decisions

### Package boundaries

- `assistant_msgs` owns only ROS interfaces.
- `assistant_perception` owns the phone stream, object inference, marker
  calibration, and pixel-to-workspace mapping. It never interprets gaze or
  commands the arm.
- `assistant_interaction` owns the laptop stream, direct normalized gaze
  regression, and temporal gaze/object fusion. Voice remains available as a
  legacy optional node but is not part of the default startup path.
- `assistant_motion` owns fresh-target validation, grasp orchestration, MoveIt
  integration points, and the hardware adapter. It never consumes image data.
- `assistant_bringup` owns split runtime configuration and launch orchestration.

### Detection topics

The detector publishes raw image detections on `/object_detections_raw`.
`workspace_mapper_node` republishes mapped detections on `/object_detections`.
This avoids a same-topic feedback loop while keeping `/object_detections` as
the stable topic consumed by selection and motion.

Each detection contains a stable-in-frame object ID, class name, confidence,
pixel center, and an optional `geometry_msgs/Pose` in `base_link`. The message
shape leaves segmentation as a future additive field/topic rather than
coupling the first version to a mask format.

### Calibration

`calibration_node` detects four configured ArUco markers in the fixed phone
view and saves a homography plus workspace-to-robot transform to a runtime
file under `~/.ros/ros2_multimodal_assistant/`. Real calibration requires an
explicitly configured robot transform; marker detection alone never creates
production motion coordinates. Missing, stale, malformed, or uncalibrated
data leaves detections without robot poses and blocks real grasp execution.

### Selection

Gaze is published as normalized workspace coordinates with confidence. The
selection manager normalizes detection pixel centers using the detection array
dimensions, rejects low-confidence or distant candidates, and confirms the
nearest object only after it remains stable for the configured hold duration.
The confirmed object is published as `/final_selection` with its latest target
pose when one is available.

### Motion and safety

`/execute_grasp` remains an action. A goal identifies the object and may carry
an explicit pose; the planner prefers a fresh live detection and refuses real
motion unless calibration, bounds, reachability, stable selection, hardware
availability, and MoveIt/hardware enablement all pass. Mock mode has a
deterministic simulated sequence and never emits hardware commands.

The action is explicit rather than automatically triggered by selection. This
keeps a visual confirmation/UI in the loop and avoids an accidental movement
from a transient gaze confirmation.

## Runtime flow

```text
phone URL -> workspace_camera_node -> /workspace/image_raw
          -> object_detector_node -> /object_detections_raw
          -> workspace_mapper_node -> /object_detections

laptop webcam -> gaze_camera_node -> /gaze/image_raw
              -> gaze_estimation_node -> /gaze/point

/gaze/point + /object_detections -> selection_manager_node
                                  -> /final_selection
                                  -> motion_planner_node
                                  -> /execute_grasp -> /arm/command
                                                    -> arm_controller_node
```

Normal perception is continuous. `/scan_workspace` remains a synchronous
snapshot/export service for testing and operators; the runtime action does not
depend on the example JSON map.

## Interfaces

- `GazePoint.msg`: header, normalized `x/y`, confidence, validity.
- `DetectedObject.msg`: header, object ID, class, confidence, pixel center,
  optional workspace pose.
- `DetectedObjectArray.msg`: header, image dimensions, calibration flag, and
  detections.
- `ObjectSelection.msg`: header, object ID/source, confidence, optional target
  pose.
- `ScanWorkspace.srv`: retained scan request/response contract.
- `CalibrateWorkspace.srv`: request calibration capture/save and report valid
  state.
- `ExecuteGrasp.action`: object ID plus optional target pose, staged feedback,
  and success/message result.

## Failure behavior

External camera frames, model outputs, calibration files, and action goals are
validated at their owning boundary. Missing OpenCV/model/serial/MoveIt
dependencies log a clear error and disable only that capability. The ROS graph
continues to run, and mock mode remains hardware-free.

## Verification

Pure unit tests cover nearest-target selection, temporal confirmation,
confidence/bounds rejection, homography mapping, calibration validity, missing
model behavior, and grasp refusal. CI runs these tests, Python lint, manifest
and YAML/interface checks, launch syntax validation, and a ROS 2 Jazzy build;
model training is excluded from CI.
