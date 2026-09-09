# Setup

Install ROS 2 Jazzy on Ubuntu 24.04, source it, and install workspace dependencies:

```bash
source /opt/ros/jazzy/setup.bash
sudo rosdep init  # once per machine; ignore the error if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install -r requirements.txt
colcon build --symlink-install
source install/setup.bash
```

The phone must expose an OpenCV-readable HTTP/MJPEG stream. Put that URL in
`config/cameras.yaml`; put the laptop device index there too. Model paths are
configured in `perception.yaml` and `gaze.yaml`.

Start mock mode:

```bash
ros2 launch assistant_bringup system.launch.py
```

Start real camera inference without enabling motion:

```bash
ros2 launch assistant_bringup system.launch.py mock_mode:=false \
  workspace_camera_url:=http://PHONE:8080/video gaze_camera_index:=0
```

Real manipulation additionally requires a valid saved calibration, trained
weights, configured transform, MoveIt planning setup, and arm controller.
