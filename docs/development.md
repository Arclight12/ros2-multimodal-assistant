# Development

Run the smallest checks first:

```bash
python3 -m flake8 src/assistant_perception/assistant_perception src/assistant_interaction/assistant_interaction src/assistant_motion/assistant_motion training
colcon build --symlink-install
```

Before pushing, run `flake8` on the packages and `training`, `colcon build`,
None of these train models or require hardware. Keep ROS callbacks thin and put geometry, selection, and safety rules
in pure modules so they remain testable on any machine.

Use feature branches and small commits. Never commit datasets, model weights,
runtime calibration, serial credentials, or absolute machine paths.
