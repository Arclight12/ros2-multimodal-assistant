# Gaze-Guided Manipulation Implementation Plan

> **For agentic workers:** Inline execution is used in this session. Work is
> split into verified slices and each completed slice is committed.

**Goal:** Evolve the ROS 2 skeleton into a safe, continuously perceived,
gaze-guided tabletop grasp system with functional deterministic mock mode.

**Architecture:** Keep the five packages. Use raw detections internally,
workspace mapping at the perception boundary, normalized gaze in interaction,
and an explicit action-driven motion layer guarded by calibration and live
validation.

**Tech Stack:** ROS 2 Jazzy, Python 3, `rclpy`, ROS IDL, OpenCV/ArUco,
Ultralytics, MediaPipe-compatible custom gaze inference, MoveIt adapter,
PyYAML, pytest, flake8.

**Spec:** `docs/superpowers/specs/2026-09-06-gaze-guided-manipulation-design.md`

## Global Constraints

- Preserve `src/assistant_msgs`, `assistant_perception`,
  `assistant_interaction`, `assistant_motion`, and `assistant_bringup`.
- No hardcoded phone URL, absolute path, robot coordinates, or model weights.
- Real arm motion is disabled unless calibration, target, bounds, reachability,
  hardware, and feature flags are valid.
- Mock mode requires no cameras, models, MoveIt, Arduino, or serial device.
- Runtime inference and model training remain separate.
- No training or hardware work runs in normal CI.

---

### Task 1: Record the approved design and establish checks

**Files:**
- Create: `docs/superpowers/specs/2026-09-06-gaze-guided-manipulation-design.md`
- Create: `docs/superpowers/plans/2026-09-06-gaze-guided-manipulation.md`
- Create: `tests/conftest.py`

- [x] Write the approved design and plan.
- [ ] Add a test import path helper only if the pure package modules cannot be
  imported directly from the repository root.
- [ ] Run the existing source parse/lint baseline and record environment limits.
- [ ] Commit the design and plan before implementation.

### Task 2: Add stable ROS interface contracts

**Files:**
- Create: `src/assistant_msgs/msg/GazePoint.msg`
- Create: `src/assistant_msgs/msg/DetectedObject.msg`
- Create: `src/assistant_msgs/msg/DetectedObjectArray.msg`
- Modify: `src/assistant_msgs/msg/ObjectSelection.msg`
- Modify: `src/assistant_msgs/srv/CalibrateWorkspace.srv`
- Modify: `src/assistant_msgs/action/ExecuteGrasp.action`
- Modify: `src/assistant_msgs/CMakeLists.txt`
- Modify: `src/assistant_msgs/package.xml`

Define additive fields using `std_msgs/Header`, `geometry_msgs/Point`,
`geometry_msgs/Pose`, and explicit validity booleans. Register every interface
and its `std_msgs`, `geometry_msgs`, `sensor_msgs`, and generator dependency.
Run interface-file checks and a ROS build when ROS is available; otherwise run
IDL text validation and commit.

### Task 3: Add pure workspace mapping and safety logic tests first

**Files:**
- Create: `src/assistant_perception/assistant_perception/workspace_mapping.py`
- Create: `src/assistant_interaction/assistant_interaction/selection_logic.py`
- Create: `src/assistant_motion/assistant_motion/grasp_safety.py`
- Create: `tests/test_workspace_mapping.py`
- Create: `tests/test_selection_logic.py`
- Create: `tests/test_grasp_safety.py`

Write tests first for normalized mapping, homography mapping, invalid
calibration, nearest valid detection, temporal confirmation, confidence and
distance rejection, workspace bounds, and mock/real grasp refusal. Use only
stdlib dataclasses in pure modules so tests do not require ROS.

### Task 4: Implement continuous phone perception

**Files:**
- Create: `src/assistant_perception/assistant_perception/workspace_camera_node.py`
- Create: `src/assistant_perception/assistant_perception/object_detector_node.py`
- Replace: `src/assistant_perception/assistant_perception/workspace_mapper_node.py`
- Create: `src/assistant_perception/assistant_perception/calibration_node.py`
- Modify: `src/assistant_perception/setup.py`
- Modify: `src/assistant_perception/package.xml`

Implement configurable OpenCV stream capture, deterministic mock frames and
detections, optional Ultralytics inference, ArUco calibration service,
runtime calibration persistence, and raw-to-mapped detection flow. Missing
dependencies only disable their capability. Keep scan as a live snapshot/map
export service and stop seeding production runtime from fake example data.

### Task 5: Implement gaze camera, direct regression inference, and fusion

**Files:**
- Create: `src/assistant_interaction/assistant_interaction/gaze_camera_node.py`
- Create: `src/assistant_interaction/assistant_interaction/gaze_estimation_node.py`
- Replace: `src/assistant_interaction/assistant_interaction/selection_manager_node.py`
- Modify: `src/assistant_interaction/setup.py`
- Modify: `src/assistant_interaction/package.xml`

Use configurable laptop camera input, an optional model adapter, deterministic
mock gaze, and the pure temporal selector. Do not infer object IDs in the gaze
node. Keep the old voice executable available but out of the default launch.

### Task 6: Implement guarded grasp orchestration and arm isolation

**Files:**
- Replace: `src/assistant_motion/assistant_motion/motion_planner_node.py`
- Replace: `src/assistant_motion/assistant_motion/arm_controller_node.py`
- Modify: `src/assistant_motion/setup.py`
- Modify: `src/assistant_motion/package.xml`

Subscribe to live detections and final selections, validate fresh pose and
safety state, expose staged `ExecuteGrasp` feedback, publish abstract JSON
commands on `/arm/command`, and have the arm node own serial/mock execution.
Real mode must refuse missing hardware/MoveIt/calibration.

### Task 7: Split bringup configuration and launch

**Files:**
- Create: `src/assistant_bringup/config/cameras.yaml`
- Create: `src/assistant_bringup/config/perception.yaml`
- Create: `src/assistant_bringup/config/gaze.yaml`
- Create: `src/assistant_bringup/config/workspace.yaml`
- Create: `src/assistant_bringup/config/robot.yaml`
- Replace: `src/assistant_bringup/config/system.yaml`
- Replace: `src/assistant_bringup/launch/system.launch.py`
- Modify: `src/assistant_bringup/setup.py`
- Modify: `src/assistant_bringup/package.xml`

Launch the nine runtime nodes with role-specific parameter files and no fake
phone URL. Preserve `ros2 launch assistant_bringup system.launch.py`.

### Task 8: Add training and dataset tooling

**Files:**
- Create: `training/gaze/collect_data.py`
- Create: `training/gaze/preprocess.py`
- Create: `training/gaze/dataset.py`
- Create: `training/gaze/model.py`
- Create: `training/gaze/train.py`
- Create: `training/gaze/evaluate.py`
- Create: `training/object_detection/collect_data.py`
- Create: `training/object_detection/prepare_dataset.py`
- Create: `training/object_detection/train.py`
- Create: `training/object_detection/evaluate.py`
- Create: `datasets/gaze/.gitkeep`
- Create: `datasets/object_detection/.gitkeep`
- Create: `models/gaze/.gitkeep`
- Create: `models/object_detection/.gitkeep`
- Modify: `.gitignore`
- Modify: `requirements.txt`

Provide executable CLI programs with lazy optional ML imports. Gaze tooling
collects multi-session grid-labeled frames and trains a feature-fusion
regressor; object tooling captures phone images, prepares YOLO layout, trains,
validates, evaluates, and exports best weights.

### Task 9: Update CI and documentation

**Files:**
- Modify: `.github/workflows/ci.yml`
- Modify: `PROJECT_SPEC.md`
- Modify: `README.md`
- Replace: `docs/architecture.md`
- Replace: `docs/topics.md`
- Replace: `docs/services.md`
- Replace: `docs/actions.md`
- Replace: `docs/calibration.md`
- Replace: `docs/setup.md`
- Replace: `docs/development.md`
- Replace: `docs/repository_structure.md`
- Create: `docs/model_training.md`

Document the two-camera setup, all interfaces, calibration, mock and real
flows, training commands, and known hardware-dependent work. CI validates all
YAML, manifests, interfaces, launch syntax, pure tests, lint, and ROS build.

### Task 10: Final verification and review

Run, where supported:

```bash
python3 -m pytest -q
python3 -m flake8 src training
python3 -m compileall -q src training
colcon build --symlink-install
colcon test --event-handlers console_direct+
```

On the Windows controller environment, record ROS commands that cannot run;
the CI workflow remains the Ubuntu Jazzy verification path. Review the diff for
hardcoded URLs/coordinates, unsafe fallbacks, dead placeholder paths, missing
model behavior, and interface/documentation drift before the final report.
