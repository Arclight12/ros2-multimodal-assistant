# Arm

The arm is a 4-DOF hobby-servo arm (base, shoulder, elbow, wrist) plus a
gripper, driven by an Arduino over USB serial. The Raspberry Pi runs the whole
ROS graph and sends joint commands to the Arduino.

## Architecture

```text
/object_detections ─┐
/final_selection   ─┴─> motion_planner_node ──> /arm/command (String, JSON)
                        (/execute_grasp action,        │
                         analytic IK, safety gate)     ▼
        ▲                                     arm_controller_node ──serial──> Arduino ──PWM──> servos
        └── /arm/status, /arm/done <──────────────────┘
```

- `motion_planner_node` runs the guarded `/execute_grasp` action. For a real
  grasp it checks `validate_real_grasp` (`grasp_safety.py`), solves inverse
  kinematics (`arm_kinematics.py`) for the pregrasp and grasp poses up front,
  then runs the stages: open gripper, move above the object, lower, close
  gripper, lift, home. Each stage waits for the arm to acknowledge it.
- `arm_controller_node` converts each command to one serial frame
  (`arm_protocol.py`, joint angle to servo degrees using the offsets and signs
  in `robot.yaml`) and reports the Arduino's reply.
- `firmware/arm_controller/arm_controller.ino` is a dumb servo driver. All
  kinematics stay on the Pi.
- Mock mode (the default) never touches hardware: the controller only logs
  frames and acknowledges them.

## ROS interface

`/arm/command` (`std_msgs/String`, JSON):

```json
{"id": 12, "command": "move_joints", "angles": [base, shoulder, elbow, wrist]}
{"id": 13, "command": "home" | "open_gripper" | "close_gripper"}
```

Angles are radians in the model convention of `arm_kinematics.py` (base 0 =
forward; shoulder 0 = upper arm horizontal, positive up; elbow and wrist are
relative to the previous link; gripper pointing down means the three pitch
angles sum to -pi/2).

- `/arm/status` is republished every second: `mock`, `unavailable`,
  `connecting` (waiting for the Arduino's `READY`) or `available`.
- `/arm/done` carries the command `id` once the Arduino replied `OK`, or
  `<id>:error` if it was rejected or the link dropped.

## Serial protocol

115200 baud, ASCII, one line per frame.

| Host sends | Meaning |
|---|---|
| `J,b,s,e,w` | servo degrees (0..180) for base, shoulder, elbow, wrist |
| `H` | home pose |
| `O` / `C` | open / close gripper |

The Arduino answers `READY` after boot, `OK` when a move has finished, or
`ERR,bad_command|bad_args|out_of_range` (nothing moves on an error). It checks
each angle against per-joint limits set in the sketch.

## Wiring and power

- Signal pins: base D3, shoulder D5, elbow D6, wrist D9, gripper D10.
- Power the servos from their own 5-6 V supply, never the Arduino 5 V pin. Stall
  current can brown out the board and the USB link.
- Join the servo supply ground to the Arduino ground.
- The Arduino is powered over USB from the Pi.

## Calibration and bring-up

All geometry and limits here are placeholders from the URDF. **Measure your
arm.**

1. Edit the constants at the top of `arm_controller.ino` (pins, per-joint
   min/max, home pose, gripper open/closed) and upload it.
2. Set the arm geometry in `src/assistant_bringup/config/robot.yaml`
   (`arm_shoulder_height_m`, `arm_upper_arm_m`, `arm_forearm_m`,
   `arm_gripper_m`). The joint limits used by IK are the `JointLimits` defaults in
   `arm_kinematics.py`; adjust them to match the arm.
3. Set `servo_offsets_deg` and `servo_signs` so that model angle 0 and the
   direction of rotation match each servo (`servo_deg = offset + sign *
   degrees(angle)`).
4. Add the Pi user to the serial group (`sudo usermod -aG dialout $USER`, log
   out and back in) and set `serial_port` (usually `/dev/ttyACM0`).
5. First power-up with the arm unloaded and clear of obstacles. The servos snap
   to the home pose at boot.
6. Test one joint at a time before a full move.
7. Real motion needs `mock_mode:=false` and `motion_enabled: true` in
   `robot.yaml`, plus a valid workspace calibration, a stable selection, and an
   IK-reachable target.

The reachable zone of a small arm is limited: with the placeholder geometry the
fingertip reaches roughly 5 to 25 cm from the base at table height.

## Why not MoveIt 2

- A 4-DOF arm cannot reach arbitrary 6D poses, and MoveIt's default KDL IK
  solver is unreliable below 6 DOF.
- Hobby servos give no joint feedback for MoveIt's execution monitoring.
- MoveIt 2 with a planning scene is heavy for a Raspberry Pi.
- Picking from a flat table needs a few waypoints, which closed-form IK covers.

MoveIt can be added later as a separate planner behind the same
`/execute_grasp` action without changing its callers.
