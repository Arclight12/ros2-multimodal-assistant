# Multimodal Hands-Free Robotic Assistant - Run Commands Guide

This document contains all the commands needed to build, test hardware, verify cameras, and run the ROS 2 system.

---

## 1. Environment & Serial Setup

Before running anything, always open a terminal inside the project directory:

```bash
cd ~/ros2-multimodal-assistant
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Grant Arduino Serial Port Permissions
Run this whenever you plug the Arduino Uno into USB (`/dev/ttyACM0`):

```bash
sudo chmod 666 /dev/ttyACM0
```
*(Or permanently add your user to the group once: `sudo usermod -aG dialout $USER` and log out/in).*

---

## 2. Hardware Testing (No ROS Required)

### A. Test 4-DOF Servos via PCA9685
Run the interactive tester to verify all 4 servos (Base, Shoulder, Elbow, Gripper):

```bash
python3 scripts/test_arm_servos.py --port /dev/ttyACM0
```
* Key options inside the tester:
  * `a` - Run automated sweep across all 4 servos
  * `h` - Move all servos to Home position (90°, 90°, 90°)
  * `o` - Open Gripper (Channel 3)
  * `c` - Close Gripper (Channel 3)
  * `0` - Test Base rotation (Channel 0)
  * `1` - Test Left Shoulder (Channel 1)
  * `2` - Test Right Elbow (Channel 2)
  * `q` - Quit

### B. Verify Laptop Gaze Camera (Webcam)
```bash
python3 scripts/check_cameras.py --skip-workspace --gaze-index 0 --frames 10
```

### C. Verify Mobile Phone Workspace Camera (IP Webcam)
Replace `<PHONE_IP>` with the IP displayed in your phone's IP Webcam app:

```bash
python3 scripts/check_cameras.py --workspace-url http://<PHONE_IP>:8080/video --skip-gaze
```

---

## 3. Launching the System

### Option A: Mock Mode (Safe Software Simulation)
Runs all 10 nodes with simulated camera, gaze, detections, and robot motion. Use this to verify software logic without physical movement:

```bash
ros2 launch assistant_bringup system.launch.py mock_mode:=true
```

### Option B: Real Hardware Mode (Live Cameras & Physical Arm)
Connect phone camera, laptop webcam, and the Arduino arm:

```bash
ros2 launch assistant_bringup system.launch.py \
  mock_mode:=false \
  workspace_camera_url:=http://<PHONE_IP>:8080/video \
  gaze_camera_index:=0 \
  serial_port:=/dev/ttyACM0
```

---

## 4. Monitoring & Diagnostics (In a Separate Terminal)

Open a new terminal and source the setup files first:
```bash
cd ~/ros2-multimodal-assistant && source install/setup.bash
```

### Check Active Nodes and Topics
```bash
ros2 node list
ros2 topic list
```

### View Live Gaze Coordinates
```bash
ros2 topic echo /gaze/point
```

### View Live Detected Objects
```bash
ros2 topic echo /object_detections
```

### View Final Authoritative Object Selection
```bash
ros2 topic echo /final_selection
```

### View Arm Controller Status
```bash
ros2 topic echo /arm/status
```

---

## 5. Triggering Actions & Services

### Trigger a Workspace Scan
```bash
ros2 service call /scan_workspace assistant_msgs/srv/ScanWorkspace "{start_scan: true}"
```

### Trigger a Grasp Action Manually
```bash
ros2 action send_goal /execute_grasp assistant_msgs/action/ExecuteGrasp "{object_id: 'cup_0'}" --feedback
```

---

## 6. Workspace Build & Clean

### Rebuild Workspace (after making code or config changes)
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Clean Rebuild (if build artifacts get corrupted)
```bash
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 7. Run Automated Test Suite
```bash
PYTHONPATH=src/assistant_perception:src/assistant_interaction:src/assistant_motion:src/assistant_bringup python3 -m pytest -q tests
```
