# Arm

The arm has four PCA9685 channels: ch0 base, ch1 + ch2 a mechanically coupled
linkage (vertical / forward), ch3 gripper. An Arduino Uno drives the PCA9685
over I2C and talks to the Raspberry Pi over USB serial. The Pi runs the whole
ROS graph.

## Architecture

```text
/object_detections ─┐
/final_selection   ─┴─> motion_planner_node ──> /arm/command (String, JSON)
                        (/execute_grasp action,        │
                         reach/base check, safety)     ▼
        ▲                                     arm_controller_node ──serial──> Uno ──I2C──> PCA9685 ──> servos
        └── /arm/status, /arm/done <──────────────────┘
```

- `motion_planner_node` runs the guarded `/execute_grasp` action. For a real
  grasp it checks `validate_real_grasp` (`grasp_safety.py`), converts the
  object's robot-frame (x, y) to a base angle and reach (`arm_target` in
  `workspace_checks.py`), then runs the stages: home, open gripper, move above
  the object (lifted), lower (table height), close gripper, lift, home. Each
  stage waits for the arm to acknowledge it. The object is assumed to be on the
  table: z is ignored.
- `arm_controller_node` turns each command into one serial frame
  (`arm_protocol.py`) and reports the Arduino's reply.
- `firmware/arm_controller/` holds the Arduino sketch. Ch1 and ch2 are never
  commanded independently: the sketch walks a grid of recorded (ch1, ch2)
  pulse pairs from `arm_calibration.h`.
- Mock mode (the default) never touches hardware: the controller only logs
  frames and acknowledges them.

## ROS interface

`/arm/command` (`std_msgs/String`, JSON):

```json
{"id": 12, "command": "move_to", "base_deg": 10.0, "reach_mm": 150.0, "level": 0}
{"id": 13, "command": "home" | "open_gripper" | "close_gripper"}
```

`level` 0 is table height (grasp), 1 is lifted. `base_deg` is 0 straight
ahead, positive to the left.

- `/arm/status` is republished every second: `mock`, `unavailable`,
  `connecting` (waiting for the Arduino's `READY`) or `available`.
- `/arm/done` carries the command `id` once the Arduino replied `OK`, or
  `<id>:error` if it was rejected or the link dropped.

## Channel limits and clamping

`robot.yaml` (`channel_limits`) is the single source of truth for the pulse
range of all four channels: ch0 base, ch1 + ch2 coupled linkage, ch3 gripper,
each 225..389 centred on 307. The same numbers are `PULSE_MIN` / `PULSE_MAX` /
`PULSE_CENTER` in `arm_protocol.py` and in `arm_calibration.h`. ROS clamps
before sending: the planner clamps the base angle through its ch0 pulse and the
reach into `arm_reach_min_m` / `arm_reach_max_m` (`clamp_command` in
`workspace_checks.py`), the arm node clamps the ch0 pulse again as the last ROS
step, and each clamp is logged once per node run. The Arduino still refuses
anything outside its own limits (`ERR,out_of_range`) as the final gate. Only
ch0's pulse follows from a command, so ROS bounds the linkage and gripper
through reach and the firmware grid instead.

## Serial protocol

115200 baud, ASCII, one line per frame.

| Host sends | Meaning |
|---|---|
| `H` | go home. The first `H` after a reset snaps every servo to home |
| `P,base_deg,reach_mm,level` | move to a base angle and reach at table (0) or lifted (1) height |
| `O` / `C` | open / close gripper |
| `x` (while moving) | freeze and keep holding; the move replies `ERR,stopped` |

The Uno answers `READY` after boot, `OK` when a move has finished, or
`ERR,bad_command|bad_args|out_of_range|not_homed|uncalibrated|stopped`
(nothing moves on an error except a freeze, which stops mid-move). It prints
`ERR,bad_calibration` and never `READY` if `arm_calibration.h` is inconsistent.

The Uno resets whenever the Pi opens the port, which leaves the servos limp.
Nothing moves until the first `H`, which the planner sends as its first stage.
**Support the arm and place it at home by hand before every launch.**

Movement is a straight-line glide of 1 pulse count per 25 ms on the fastest
channel. Base and reach only change while lifted; the arm descends and rises
through every adjacent recorded row so no straight segment spans a gap you did
not test.

## Wiring and power

- Pi USB-A to Uno USB-B. The Uno also takes its power from this cable.
- Uno A4 (SDA), A5 (SCL), 5V, GND to PCA9685 SDA, SCL, VCC, GND. Address 0x40.
- Servos on PCA9685 channels 0..3, powered from a separate 5 V supply on the
  PCA9685 V+ terminal, never from the Uno 5 V pin. Stall current can brown out
  the board and the USB link.
- Join the servo supply ground to the Uno ground.
- Install the Adafruit PWM Servo Driver library in the Arduino IDE.

## Calibration and bring-up

Only the (350, 300) pair is confirmed. Everything else in
`arm_calibration.h` is a placeholder, and real grasps are refused until you
fill it in.

1. Use the jog sketch in `arm_movement.txt` (serial monitor, 115200). Support the
   arm. Walk ch1 and ch2 outward from the reference in 2-3 count steps and
   press `s` to print each pair that moves with no buzzing or binding. Stop and
   undo at the first grinding.
2. Fill `GRID` and `REACH_MM` in `arm_calibration.h`: rows are heights (row 0 at
   table height, last row lifted and home), columns are reach in ascending mm.
   Measure each reach from the base axis to the gripper tip. Add rows or
   columns until each neighbouring pair is a safe straight line.
3. Measure `BASE_CENTER` and `BASE_COUNTS_PER_DEG` (negative reverses), and the
   `GRIP_OPEN` / `GRIP_CLOSED` pulses. Equal gripper values mean `O`/`C` are
   refused.
4. Upload `arm_controller.ino` (the sketch folder includes `arm_calibration.h`).
   Open the serial monitor and send, one at a time: `H`, `P,0,<reach>,1`,
   `P,0,<reach>,0`, `O`, `C`, back to `H`. Type `x` to freeze during a move.
5. In `robot.yaml` set `arm_reach_min_m` / `arm_reach_max_m` to the first and
   last `REACH_MM` (metres) and `arm_base_limit_deg` inside the base range.
6. On the Pi: `sudo usermod -aG dialout $USER`, log out and back in, check
   `serial_port` (default `/dev/ttyACM0`).
7. Real motion needs `mock_mode:=false` and `motion_enabled: true` in
   `robot.yaml`, plus a valid workspace calibration, a stable selection and a
   target inside the calibrated reach and base range.

Pulses are commands, not measured positions. The SG90s give no feedback, so a
stall or a hand-moved arm is invisible to the code. Cut servo power on any
grinding or buzzing.
