# Project specification

## Goal

Identify the tabletop object a user is looking at, map its phone-camera pixel
location into the robot base frame, and execute a safe grasp when explicitly
requested.

## Fixed setup

- Ubuntu 24.04, ROS 2 Jazzy, Python, `rclpy`, and `ament_python`.
- The laptop webcam faces the user and is used only for gaze estimation.
- A phone camera is fixed above the tabletop and is used for object detection.
- The arm and phone remain fixed relative to the workspace.
- The first mapping version assumes a planar table at configured `Z`.

## Runtime rules

Perception continuously publishes detections. Gaze is direct normalized
workspace regression, not arbitrary monocular depth. Selection chooses the
nearest valid detection and requires confidence, distance, and temporal hold
criteria. Motion uses the live selected object and refuses real execution when
calibration, pose, bounds, reachability, hardware, MoveIt, or selection state is
unsafe. Mock mode is deterministic and never moves hardware.

## Package boundaries

The five existing packages remain separate. Shared contracts are in
`assistant_msgs`; camera/model/calibration work is in `assistant_perception`;
human intent and fusion are in `assistant_interaction`; manipulation and arm
communication are in `assistant_motion`; launch/configuration are in
`assistant_bringup`. Training remains under top-level `training/`.

## Calibration and data

ArUco markers establish a pixel-to-normalized-workspace homography. A
configured workspace-to-base transform is required for real motion. Calibration
is persisted under `~/.ros/ros2_multimodal_assistant` by default. Datasets and
weights are local, git-ignored runtime artifacts.

## Acceptance criteria

The workspace builds on ROS 2 Jazzy, starts fully in mock mode, publishes live
mock gaze/detection/selection data, executes a simulated guarded grasp, and has
hardware-free tests for mapping, selection stabilization, confidence rejection,
bounds, calibration refusal, missing models, and grasp safety.
