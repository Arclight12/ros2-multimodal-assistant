# Setup Guide

This guide walks through a clean installation of the multimodal assistant on
**Ubuntu 24.04** with **ROS 2 Jazzy**.

## 1. Prerequisites

- Ubuntu 24.04 (desktop or server).
- A GitHub account (for cloning).
- ROS 2 Jazzy installed.
- Python 3.10+.

## 2. Install ROS 2 Jazzy

If ROS 2 is not already installed, follow the official instructions:

```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade -y
sudo apt install ros-jazzy-ros-base python3-colcon-common-extensions \
  python3-rosdep python3-vcstool -y
```

Source the environment in every terminal:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 3. Install rosdep Dependencies

```bash
sudo rosdep init
rosdep update
```

Note: if `rosdep init` reports that it is already initialized, skip it.

> **Portability note:** `rosdep` was not runnable on the development machine
> (not installed, and no system `pip` overrides). These steps follow the
> standard Ubuntu 24.04 / Jazzy flow and the CI job runs `rosdep install`, but
> the exact `rosdep` output was not captured on a fresh machine. In practice
> the skeleton's ROS packages only need `rclpy`, `std_msgs`, `launch`,
> `launch_ros`, and `ament_index_python`, which ship with `ros-jazzy-ros-base`.

## 4. Install Python Dependencies

From the workspace root:

```bash
python3 -m pip install --upgrade pip
pip install -r requirements.txt
```

This installs NumPy, OpenCV (contrib, a superset that includes the ArUco
modules), MediaPipe, Ultralytics, Vosk, pyserial, and PyYAML.

> The skeleton itself does **not** import any of these packages; they are
> needed only by the future inference features. The running workspace needs
> only the ROS 2 system packages. `opencv-python` is intentionally not listed,
> since `opencv-contrib-python` is a superset.

## 5. Clone the Repository

```bash
git clone https://github.com/Arclight12/ros2-multimodal-assistant.git
cd ros2-multimodal-assistant
```

## 6. Build the Workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

Use `--symlink-install` during active development so Python edits apply without
rebuilding.

## 7. Launch the System

```bash
ros2 launch assistant_bringup system.launch.py
```

## Optional: Missing AI Models

The skeleton runs **without** any AI models. To later enable inference:

- **YOLOv8n**: download `yolov8n.pt` into `models/`.
- **MediaPipe**: the FaceMesh model is fetched by the MediaPipe runtime.
- **Vosk**: place a small Vosk model (e.g. `vosk-model-small-en-us`) in `models/`.

Models are git-ignored to keep the repository small.

## Verification

| Check | Command |
|-------|---------|
| Nodes running | `ros2 node list` |
| Graph introspection | `ros2 topic list` / `ros2 service list` |
| Trigger a scan | `ros2 service call /scan_workspace ...` |
| Grasp (skeleton) | `ros2 action send_goal /execute_grasp ...` |