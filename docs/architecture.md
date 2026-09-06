# System Architecture

## Overview

The multimodal robotic assistant is a ROS 2 (Jazzy) based system designed to
let a user select an object on a tabletop using **voice** or **gaze** input,
and have a robotic arm autonomously plan and execute a grasp toward that object.

The architecture follows a **strict separation of concerns** across four
functional domains (perception, interaction, motion, orchestration), each
isolated into its own ROS package. This keeps coupling minimal, makes the
system testable by four independent contributors, and preserves clean extension
points for future work.

## Architectural Principles

1. **Clean boundaries** - perception never moves the arm; motion never
   interprets a camera frame.
2. **Data flows one direction** - from perception (world state) through
   interaction (user intent) into motion (execution).
3. **Configuration over code** - paths, ports, and feature toggles live in
   YAML, never hardcoded.
4. **Extensible by design** - every AI integration (YOLO, MediaPipe, Vosk,
   MoveIt) is a clearly marked extension point.
5. **Fail-open startup** - missing hardware or models never crashes the ROS
   graph; the system logs and continues.

## Layered View

```
+----------------------------------------------------------------------------------+
|                          ROS 2 Middleware (DDS)                                  |
+----------------------------------------------------------------------------------+
        |                          |                           |
        v                          v                           v
+------------------+  +-----------------------------+  +-------------------------+
|  Perception      |  |  Interaction                |  |  Motion                 |
|  workspace_      |  |  voice_node       gaze_node |  |  motion_planner_node    |
|  mapper_node     |  |         \           /       |  |  arm_controller_node    |
|                  |  |          \         /        |  |                         |
| camera + ArUco + |  |         selection_manager   |  |  MoveIt + Arduino       |
| YOLO + object_map|  |  (voice wins conflicts)     |  |                         |
+------------------+  +-----------------------------+  +-------------------------+
        |                          |                           |
        +--------------------------+---------------------------+
                                   |
                         +---------------------+
                         |   assistant_bringup  |
                         |  (launch + config)   |
                         +---------------------+
```

## Data Flow

1. **Workspace Understanding** - `workspace_mapper_node` captures frames from
   the phone camera, establishes a workspace frame from ArUco markers, detects
   objects with YOLO, and writes the runtime object map. In mock mode it emits
   simulated detections instead.
2. **Intent Capture** - `voice_node` (Vosk) and `gaze_node` (MediaPipe) each
   publish a candidate object selection.
3. **Arbitration** - `selection_manager_node` resolves conflicts. If voice and
   gaze disagree, **voice wins**.
4. **Motion Generation** - `motion_planner_node` looks up the selected object's
   pose in the object map and provides a MoveIt planning/execution action.
5. **Actuation** - `arm_controller_node` translates plans into serial commands
   for the Arduino that drives the servos and gripper.

## Workspace Scanning Pipeline

```
Phone Camera
   │
   ▼
ArUco Detection ──► Workspace Frame (coordinate system)
   │
   ▼
YOLO Detection ──► Object Pixel → Workspace Coordinates
   │
   ▼
object_map.json (shared, reusable)
```

## Feature Toggles

Runtime behavior is controlled through `config/system.yaml`:

| Parameter          | Purpose                                   |
|--------------------|-------------------------------------------|
| `mock_mode`        | Simulate scan detections / grasps (default `true`) |
| `moveit_enabled`   | Enable/disable MoveIt integration         |
| `debug_mode`       | Extra logging / debug visualization       |
| `camera_url`       | IP stream source for the workspace camera |

## Data Provenance

The object map written by scans is stored **outside** the repository (default
`~/.ros/ros2_multimodal_assistant/object_map.json`) and carries a `_meta`
provenance block (`mock`, `calibrated`, `source`, `units`, coordinates frame).
The motion planner refuses real arm motion while the map is not calibrated, so
placeholder coordinates can never become real robot commands by accident.

## Future Extensions

The layer boundaries leave extension points for:
- Continuous workspace updates (streaming perception).
- Pick-and-place workflows (multi-goal actions in motion layer).
- ESP32 migration (drop-in serial abstraction in `arm_controller_node`).
- Improved grasp planning (MoveIt within `motion_planner_node`).
