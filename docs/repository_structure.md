# Repository structure

```text
src/
  assistant_msgs/          shared messages, service, action
  assistant_perception/    phone camera, detector, mapping, calibration
  assistant_interaction/   laptop camera, gaze, temporal selection
  assistant_motion/        guarded grasp action and arm adapter
  assistant_bringup/       launch files and YAML configuration
training/
  gaze/                    collection, preprocessing, model, train, evaluate
  object_detection/        collection, preparation, train, evaluate
datasets/                   git-ignored collected data
models/                    git-ignored exported weights
tests/                     hardware-free unit tests
docs/                      contracts, setup, calibration, training
```

Runtime calibration defaults to `~/.ros/ros2_multimodal_assistant`. The tracked
`object_map_example.json` is documentation/test data only; normal operation
uses continuous detections.
