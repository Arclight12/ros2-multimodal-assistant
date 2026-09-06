# Multimodal Hands-Free Robotic Assistant

A ROS 2 (Jazzy) based robotic assistant for hands-free tabletop object
acquisition. Select an object with your **voice** or your **gaze**, and a
servo-driven robotic arm plans and executes a grasp toward it.

Built as a modular, production-quality ROS 2 workspace skeleton that is fully
launchable today and ready for future integration of YOLOv8n, MediaPipe
FaceMesh, Vosk, MoveIt 2, and an Arduino Uno.

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange)](https://ubuntu.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![CI](https://github.com/Arclight12/ros2-multimodal-assistant/actions/workflows/ci.yml/badge.svg)](https://github.com/Arclight12/ros2-multimodal-assistant/actions)

---

## 1. Project Title

**Multimodal Hands-Free Robotic Assistant**

## 2. Overview

This project is a ROS 2 workspace that coordinates five packages to:

1. Scan a tabletop workspace and build an object database.
2. Accept user intent through voice and gaze.
3. Resolve conflicts between modalities (voice wins).
4. Plan and execute arm motion toward the selected object.

The system runs fully on Ubuntu 24.04 with ROS 2 Jazzy and is implemented in
Python with `rclpy` and `ament_python`.

## 3. Motivation

Hands-free assistive manipulation is valuable for people who cannot physically
interact with devices. This term project demonstrates a clean, modular ROS 2
architecture that a four-person team can develop in parallel: each member owns
an isolated package with well-defined interfaces.

## 4. System Architecture Diagram

```
User
 ├── Voice ──► voice_node ────────────┐
 └── Gaze  ──► gaze_node ─────────────┤
                                      ▼
                              selection_manager_node
                                      │  (voice wins conflicts)
                                      ▼
                              /final_selection
                                      │
                                      ▼
                              motion_planner_node
                                      │
                          ┌───────────┴───────────┐
                          ▼                       ▼
                       MoveIt 2              arm_controller_node
                  (planning placeholder)           │
                                            (serial protocol)
                                          Arduino Uno │
                                                      ▼
                                               Robot Arm + Gripper

phone camera ──► workspace_mapper_node ──► runtime object map ──► motion_planner
                       (mock mode simulates detections)
```

## 5. Package Architecture Diagram

```
src/
├── assistant_msgs        Shared interfaces (msg / srv / action) - no logic
├── assistant_perception  Workspace understanding (camera, ArUco, YOLO, map)
├── assistant_interaction Voice, gaze, selection arbitration
├── assistant_motion      MoveIt integration, planning, Arduino control
└── assistant_bringup     Launch files, config, system startup
```

## 6. ROS Graph Overview

| Communication | Name           | Type |
|---------------|----------------|------|
| Topics        | `/voice/object_id`, `/gaze/object_id`, `/final_selection`, `/object_detections`, `/workspace_status` | `assistant_msgs/ObjectSelection` / `std_msgs/String` |
| Services      | `/scan_workspace` | `assistant_msgs/ScanWorkspace` |
| Actions       | `/execute_grasp`  | `assistant_msgs/ExecuteGrasp` |

## 7. Repository Structure

```
.
├── src/                # 5 ROS 2 packages (colcon root)
├── docs/               # Technical documentation
├── models/             # AI model weights (git-ignored)
├── scripts/            # Developer helpers
├── .github/workflows/  # CI
├── requirements.txt    # Python dependencies
├── PROJECT_SPEC.md
├── README.md
└── LICENSE
```

See [docs/repository_structure.md](docs/repository_structure.md) for detail.

## 8. Installation

```bash
git clone https://github.com/Arclight12/ros2-multimodal-assistant.git
cd ros2-multimodal-assistant
```

## 9. Dependency Installation

### ROS dependencies

See [docs/setup.md](docs/setup.md) for full ROS 2 Jazzy installation steps.

```bash
sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Python dependencies

```bash
pip install -r requirements.txt
```

Installs `numpy`, `opencv-contrib-python`, `mediapipe`, `ultralytics`,
`vosk`, `pyserial`, and `pyyaml`.

> The skeleton itself does **not** import any of these: the running workspace
> only needs the ROS 2 system packages. These dependencies are for the future
> inference/motion features. `opencv-python` is intentionally absent because
> `opencv-contrib-python` (which also provides the ArUco modules) is a superset.

## 10. ROS Setup

| Item    | Value                                         |
|---------|-----------------------------------------------|
| Distro  | ROS 2 Jazzy                                    |
| OS      | Ubuntu 24.04                                   |
| Library | `rclpy`                                        |
| Build   | `ament_python` (Python packages)               |

```bash
source /opt/ros/jazzy/setup.bash
```

## 11. Build Instructions

```bash
colcon build --symlink-install
source install/setup.bash
```

Use `--symlink-install` for Python development so edits apply immediately.

## 12. Running The System

Two ways to start:

### Full system

```bash
ros2 launch assistant_bringup system.launch.py
```

### Individual nodes (debugging)

```bash
ros2 run assistant_perception workspace_mapper_node
ros2 run assistant_interaction voice_node
ros2 run assistant_interaction gaze_node
ros2 run assistant_interaction selection_manager_node
ros2 run assistant_motion motion_planner_node
ros2 run assistant_motion arm_controller_node
```

## 13. Launch Instructions

The launch file loads `config/system.yaml` into every node and starts all six
nodes:

```bash
ros2 launch assistant_bringup system.launch.py
```

Verify the graph:

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

### Trigger a scan

```bash
ros2 service call /scan_workspace assistant_msgs/srv/ScanWorkspace "{start_scan: true}"
```

In mock mode the response is `success=True, message='Scan complete (MOCK). Found 2 objects.'`,
and the scan writes only to the runtime map
(`~/.ros/ros2_multimodal_assistant/object_map.json`).

### Execute a grasp (mock)

```bash
ros2 action send_goal /execute_grasp assistant_msgs/action/ExecuteGrasp "{object_id: 'cup'}" --feedback
```

In mock mode the result says `Mock grasp of 'cup' completed (SIMULATED, no robot motion).`
No arm motion is performed.

## 14. Interfaces

All shared interfaces live in `assistant_msgs`:

| Interface | Kind     | Definition |
|-----------|----------|------------|
| `ObjectSelection.msg` | Message | `string object_id`, `string source` |
| `ScanWorkspace.srv`   | Service | `bool start_scan` → `bool success`, `string message` |
| `ExecuteGrasp.action` | Action  | Goal: `object_id`; Result: `success`, `message`; Feedback: `current_state` |

## 15. Topics

| Topic                | Type                          | Description                     |
|----------------------|-------------------------------|---------------------------------|
| `/voice/object_id`   | `ObjectSelection`             | Recognized voice selection      |
| `/gaze/object_id`    | `ObjectSelection`             | Inferred gaze selection         |
| `/final_selection`   | `ObjectSelection`             | Authoritative resolved selection|
| `/object_detections` | `std_msgs/String` (JSON)      | Raw detections (observability)  |
| `/workspace_status`  | `std_msgs/String`             | Scan/status messages            |

## 16. Services

| Service           | Type                  | Purpose                       |
|-------------------|-----------------------|-------------------------------|
| `/scan_workspace` | `ScanWorkspace`       | Trigger a workspace scan      |

## 17. Actions

| Action           | Type               | Purpose                    |
|------------------|--------------------|----------------------------|
| `/execute_grasp` | `ExecuteGrasp`     | Plan, move, and grasp      |

## 18. Development Workflow

```bash
# Source ROS and the workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Develop in your package
# (edit Python files under src/<pkg>/<pkg>/)

# Lint
python3 -m flake8 src/<package>

# Rebuild (fast with symlink-install)
colcon build --symlink-install
```

See [docs/development.md](docs/development.md).

## 19. Team Contribution Guidelines

- One package owner per contributor (see [docs/development.md](docs/development.md)).
- Feature branches + Pull Requests; CI runs on every PR.
- Never edit interfaces owned by another contributor without coordination.
- Follow PEP 8, type hints, docstrings, and the Node conventions.
- Keep PRs small, focused, and reviewable.

## 20. Troubleshooting

| Problem | Solution |
|---------|----------|
| `colcon: command not found` | `sudo apt install python3-colcon-common-extensions` |
| Missing ROS packages | `rosdep install --from-paths src --ignore-src -r -y` |
| Node crashes on missing model | Nodes are fail-open; check `ros2 node list` for warnings in logs |
| Serial port not found | Edit `serial_port` in `config/system.yaml` (e.g. `/dev/ttyUSB0`) |
| Camera not streaming | Verify `camera_url` reachable: `curl -I <url>` |
| Scan service fails | Set `debug_mode: true` and re-run |

## 21. System Status

The distinction between what actually works today and what is scaffolding for
future work is kept explicit. **Mock mode** (`mock_mode: true`, the default)
simulates scan detections and grasps; every mock result is explicitly labeled
so it can never be mistaken for real robot motion.

### Currently working (validated)

- Five-package `colcon` workspace that builds clean
  (`colcon build --symlink-install`).
- `ros2 launch assistant_bringup system.launch.py` starts all **6 nodes**.
- Full ROS graph: 5 topics, 1 service, 1 action.
- `ScanWorkspace` service: mock scan returns `success=True` and a `(MOCK)`
  response, writes the runtime object map (outside the repo).
- Voice/gaze arbitration: `voice_node` wins conflicts in `selection_manager`.
- `ExecuteGrasp` action: mock grasp of a known object succeeds with feedback
  `[lookup_pose, planning, executing_simulated]`; unknown objects are aborted.
- Interfaces (`ObjectSelection`, `ScanWorkspace`, `ExecuteGrasp`) validate via
  `ros2 interface show`.
- flake8-clean Python sources and CI (lint, build, repo-checks) for GitHub.

### Not yet implemented (future work)

- YOLOv8n object detection and ArUco-based workspace calibration.
- MediaPipe FaceMesh gaze tracking.
- Vosk offline speech recognition.
- Motorized robot arm / Arduino Uno firmware and serial servo control.
- MoveIt 2 / URDF-SRDF planning and execution, RViz visualization.
- TF calibration between the camera and the arm base frame.

### Future Work

- Continuous workspace updates instead of one-shot scans.
- Pick-and-place and multi-object workflows.
- ESP32 migration for the arm controller.
- Robotic grasping improvements (pose refinement, tactile feedback).

## 22. License

This project is licensed under the [MIT License](LICENSE).

---

_Generated as a term-project foundation by the Robotics Course Team._