# Fixed-pose arm demo

Sequence: open gripper -> move to pickup -> close to fixed endpoint -> return
to home/handover while gripping -> hold 3 seconds -> release.

| Channel | Home/handover | Pickup |
|---|---:|---:|
| CH0 base | 330 | 460 |
| CH1 linkage | 430 | 490 |
| CH2 linkage | 400 | 460 |
| CH3 gripper | 300 open | 460 close |

Increasing CH0 pulse moves toward the viewer's right in this setup. CH1 and CH2
are mechanically coupled. The path between endpoints must be tested unloaded.
No sensor detects object contact or confirms successful pickup.

Install the Adafruit PWM Servo Driver library. PCA9685 address is 0x40.
Power servos from a separate regulated 5V supply on V+, with common ground to
the Uno; never use Uno 5V for servo power. Support the arm and physically place
it at home before the first command. Cut servo power immediately if buzzing,
grinding, binding, or stalling occurs.

The demo firmware supports D, H, O, C, and x; it does not support the old
generic P,base_deg,reach_mm,level command. Keep motion_planner_node real motion
disabled until a linkage grid and workspace are fully calibrated.

## Serial protocol

115200 baud, ASCII, one command at a time. Commands: `D` (demo cycle), `H`
(home, gripper open), `O`, `C`, and `x` (freeze). The Uno replies `READY` after
boot, then exactly one `OK` or `ERR,<reason>` per command (`bad_command`,
`out_of_range`, `stopped`; `ERR,bad_calibration` and no `READY` if the limits in
`arm_calibration.h` are inconsistent). `STATE,<name>` lines during `D` are
progress only. `x` mid-move makes the running command reply `ERR,stopped`; idle
`x` gets no reply. While a move runs the Uno discards everything except `x`, so
`arm_controller_node` allows one command in flight and refuses another with
`<id>:error`. ROS `stop` sends `x` and is acknowledged at once.

## Motion and limits

Every move is a straight line in pulse space from the current pose to the
target, with all four channels arriving together and a smoothstep ease in and
out. Peak speed is 1 count per 20 ms on the longest channel. All endpoints sit
on the per-channel min/max, so the path cannot leave the tested box; the
firmware also rejects any target outside it (`ERR,out_of_range`). CH1 and CH2
keep a constant 30 count offset along the path, as at both endpoints.

Not established, so not enabled: CH1 alternate heights 490..520, CH3 open limit
below 300, and CH3 closing limit between 400 and 460. The firmware has no
contact feedback, so closing on an object at 460 may stall the gripper servo.

The Uno resets when the port opens and leaves the servos unpowered; the first
command then drives them straight to the commanded pose with no ramp. Hold the
arm at home before every launch.

Trigger through the existing arm_controller_node (mock_mode false):
ros2 topic pub --once /arm/command std_msgs/msg/String \
  "{data: '{\\"id\\": 1, \\"command\\": \\"demo_cycle\\"}'}"
