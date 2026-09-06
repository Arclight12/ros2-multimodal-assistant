# Actions

This document describes the ROS 2 action interface used for grasp execution.

## Action Summary

| Action           | Type                           | Server            | Client          |
|------------------|--------------------------------|-------------------|-----------------|
| `/execute_grasp` | `assistant_msgs/ExecuteGrasp`  | `motion_planner`  | CLI / UI        |

## `/execute_grasp`

Executes a full grasp of a selected object: look up pose, plan motion
(MoveIt), move the arm, and close the gripper. This is an **action** because it
is long-running and reports periodic feedback.

### Goal

```
string object_id
```

The identifier of the object to grasp. It must exist in the runtime object map
(`~/.ros/ros2_multimodal_assistant/object_map.json` by default), otherwise the
goal is aborted.

### Result

```
bool success
string message
```

`success` states whether the grasp completed. `message` explains the outcome,
including failure reasons (e.g. "Object not found").

### Feedback

```
string current_state
```

Human-readable stages published during execution:

| State                 | Meaning                                   |
|-----------------------|-------------------------------------------|
| `lookup_pose`         | Retrieving the object pose from the map   |
| `planning`            | Computing a trajectory via MoveIt         |
| `moving`              | Executing the trajectory through Arduino  |
| `executing_simulated` | Simulated execution in mock mode          |

### Semantics

- The action server rejects goals for unknown objects by aborting.
- If the object map is marked `_meta.calibrated: false` (mock or example data),
  and `moveit_enabled` is `true`, the goal is **aborted** with a calibration
  refusal message: placeholder coordinates are never used to move a real arm.
- With `mock_mode: true` the result message is labeled `SIMULATED` so no one
  mistakes a mock "success" for real robot motion.
- If `moveit_enabled` is `false` and mock mode is off, motion is skipped and a
  skeleton result is reported.
- Clients receive feedback on every state transition.

### Example (CLI)

First, send a goal in a terminal to watch feedback:

```bash
ros2 action send_goal /execute_grasp assistant_msgs/action/ExecuteGrasp "{object_id: 'cup'}" --feedback
```

Example output (mock mode enabled):

```
Waiting for an action server to become available...
Sending goal:
    object_id: cup

Goal accepted with ID: ...
Feedback:
    current_state: lookup_pose
    current_state: planning
    current_state: executing_simulated

Result:
    success: True
    message: "Mock grasp of 'cup' completed (SIMULATED, no robot motion)."
```
