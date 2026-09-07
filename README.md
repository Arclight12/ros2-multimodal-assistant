# Gaze-Guided ROS 2 Manipulation Assistant

This ROS 2 Jazzy workspace lets a user look at a tabletop object and select it
for robotic grasping. A fixed phone camera detects and localizes objects; a
laptop webcam estimates gaze only. Real motion is disabled until calibration,
validation, reachability, hardware, and stable selection checks all pass.

The default launch is deterministic mock mode, so the complete graph can be
developed without cameras, trained weights, MoveIt, Arduino, or an arm.

## Quick start

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -r requirements.txt
colcon build --symlink-install
source install/setup.bash
ros2 launch assistant_bringup system.launch.py
```

Inspect `/gaze/point`, `/object_detections`, and `/final_selection`. Mock mode
publishes a stable simulated target; `/execute_grasp` is an explicit action and
does not automatically move an arm.

## Real cameras

Set values in `src/assistant_bringup/config/cameras.yaml` or override them:

```bash
ros2 launch assistant_bringup system.launch.py \
  workspace_camera_url:=http://PHONE:8080/video \
  gaze_camera_index:=0 mock_mode:=false
```

The phone URL and laptop device are parameters, not source-code constants. See
[docs/setup.md](docs/setup.md) and [docs/calibration.md](docs/calibration.md).

## Training

Training is separate from ROS runtime code. See
[docs/model_training.md](docs/model_training.md) for collection, preparation,
training, and evaluation commands.

## Packages

`assistant_msgs` owns interfaces; `assistant_perception` owns camera, detection,
mapping, and calibration; `assistant_interaction` owns gaze and temporal target
selection; `assistant_motion` owns guarded grasp execution and arm I/O;
`assistant_bringup` owns launch and configuration.

## Verification

```bash
PYTHONPATH=src/assistant_perception:src/assistant_interaction:src/assistant_motion python3 -m pytest -q tests
python3 -m flake8 src/assistant_perception/assistant_perception src/assistant_interaction/assistant_interaction src/assistant_motion/assistant_motion training
```

See [docs/development.md](docs/development.md) for the CI and hardware-free
workflow.
