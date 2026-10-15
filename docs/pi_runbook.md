# Raspberry Pi runbook and project status

Everything needed to resume work on the gaze-guided detection flow without
redoing setup. Credentials are deliberately not in this file.

## What the system does

Look at an object -> the robot knows which one.

```
phone camera  -> /workspace/image_raw -> object_detector_node -> /object_detections_raw
                                       -> workspace_mapper_node -> /object_detections
laptop webcam -> /gaze/image_raw -> gaze_estimation_node -> /gaze/point
/object_detections + /gaze/point -> selection_manager_node -> /final_selection -> motion_planner_node -> arm
```

- Detector: YOLO, classes `cup`, `water_bottle`, `book`, `pen`, `phone`, `ball`.
- Gaze: MediaPipe FaceMesh + a small TorchScript regressor -> `x, y` in 0..1 of the screen.
- Selection: the gaze point is compared with each object's centre as a fraction
  of the phone image. Within `maximum_gaze_object_distance` (0.2) for
  `selection_hold_seconds` (1.0 s, 0.5 s dropout grace) confirms the object.
- `/final_selection` (`ObjectSelection`) carries `object_id`, `class_name`,
  `image_center` (px), `workspace_center`, `has_workspace_pose` and the robot
  `target_pose` (only valid after calibration).
- Voice input was removed completely.

## Hardware and network (this setup)

| Device | Role | Address |
| --- | --- | --- |
| Raspberry Pi 4, Ubuntu 24.04, ROS 2 Jazzy | runs all nodes | `nishanth@10.71.243.194` (hostname `rpi4`; `rpi4.local` does NOT resolve - find it with `arp -a`) |
| Android phone, IP Webcam app | workspace camera | `http://10.71.243.68:8080/video` |
| Windows laptop webcam | gaze camera, served by `scripts/stream_webcam.py` | `http://10.71.243.106:8081/video` |

All three are on the phone hotspot (`10.71.243.x`); addresses can change.
Repo on the Pi: `~/ros2-multimodal-assistant`, venv `~/robot-venv`.

## Pi software state (already installed)

- colcon packages built with `--symlink-install` (Python/yaml edits apply after a
  copy; rebuild only for `.msg` changes or new entry points).
- venv needs: `setuptools<80`, `torch==2.5.1`, `torchvision==0.20.1`, `numpy<2`,
  `opencv-contrib-python<5` (4.11), ultralytics, mediapipe. Newer torch crashes
  on the Pi 4 (SIGILL).
- ROS node scripts use `/usr/bin/python3`, so every shell exports
  `PYTHONPATH=$VIRTUAL_ENV/lib/python3.12/site-packages`.
- `scripts/setup_pi.sh` reproduces the install on a fresh Pi.

## Run it

1. Phone: start the IP Webcam server. Aim the back camera at a lit table with
   one object in view (a dark frame means no detections).
2. Laptop, from the repo (run exactly one copy):
   ```powershell
   python scripts/stream_webcam.py
   ```
   Check `http://localhost:8081/video` in a browser; it must be live.
3. Copy code to the Pi after local changes (PuTTY `pscp`; pin the host key with
   `-hostkey` or accept it once):
   ```powershell
   pscp -r src scripts nishanth@10.71.243.194:ros2-multimodal-assistant/
   ```
4. Pi terminal 1 - launch (build first only if a `.msg`/entry point changed):
   ```bash
   cd ~/ros2-multimodal-assistant
   source /opt/ros/jazzy/setup.bash && source ~/robot-venv/bin/activate
   colcon build --symlink-install --packages-select assistant_msgs assistant_interaction assistant_bringup
   bash scripts/pi_launch.sh 10.71.243.68 10.71.243.106
   ```
   Detector and gaze nodes need ~60 s to start. Expected harmless messages:
   `serial_port is empty` (no arm yet), `Workspace calibration is invalid`,
   `mjpeg overread`.
5. Pi terminal 2 - every new shell needs the setup lines first:
   ```bash
   cd ~/ros2-multimodal-assistant
   source /opt/ros/jazzy/setup.bash && source ~/robot-venv/bin/activate && source install/setup.bash
   export PYTHONPATH=$VIRTUAL_ENV/lib/python3.12/site-packages:$PYTHONPATH
   ros2 topic echo /gaze/point --field valid     # true with your face in view
   ros2 topic echo /object_detections --once     # class, confidence, pixels
   ros2 run assistant_interaction flow_monitor_node
   ```
   Hold your gaze on the on-screen spot matching the object's position in the
   phone image for ~1 s -> `SELECTED <id> (<class>)` with coordinates.
6. Laptop: see what the Pi sees (yellow = detection, green = selected, red ring = gaze):
   ```powershell
   pscp nishanth@10.71.243.194:/tmp/flow.jpg .
   ```
7. Stop: Ctrl+C everywhere, then `sudo shutdown now` on the Pi.

## Lessons learned (do not rediscover)

- A node-specific key in a yaml file overrides launch args; that is why
  `camera_url` came out empty. `cameras.yaml` no longer sets camera values.
- `ros2` not found in a new SSH shell = the setup lines were not run.
- Two copies of `stream_webcam.py` fight over the camera and port 8081 and
  produce a frozen, slow feed. Keep exactly one.
- Gaze `valid: false` with confidence 0 = no face in the webcam frame (or a
  frozen webcam feed), not a Pi fault.
- No yellow box = the phone frame is dark, blurry or the object is not one of
  the six classes. The Pi frame is 1080x1920; lowering the phone resolution
  speeds up the ~1-2 Hz pipeline.
- `ros2 topic echo` without `--once` never exits, so commands pasted after it do
  not run.

## Status

Done and verified on the Pi: both camera streams into ROS, detector (a phone at
0.90 confidence, a water bottle at 0.77), gaze node publishing valid points,
smoothing/selection changes pushed to `origin/dev`.

Not yet verified:
- the smoothing/threshold changes on the Pi (need copy + relaunch),
- gaze accuracy against real screen positions,
- an end-to-end `SELECTED` block with a real object (last phone frame was black),
- the workspace/robot coordinates.

Next steps:
1. Copy code, relaunch, get a lit object in the phone view, confirm `SELECTED`.
2. Calibrate: 4 ArUco markers (`DICT_4X4_50`, ids 0-3, see
   `config/perception.yaml` and `config/workspace.yaml`), set the robot base
   transform, then
   `ros2 service call /calibrate_workspace assistant_msgs/srv/CalibrateWorkspace "{start_calibration: true}"`
   (see `docs/calibration.md`).
3. Connect the arm (`serial_port` in the motion config, `docs/arm.md`) and test
   grasps.
4. Optional: more gaze training data if accuracy is poor (`docs/model_training.md`).

## Repo hygiene

Formatter: `python -m ruff format src scripts training` (config in
`pyproject.toml`, line length 88). Lint: `python -m ruff check src scripts training`.
The pytest suite was removed on request.
