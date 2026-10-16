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

Trigger through the existing arm_controller_node (mock_mode false):
ros2 topic pub --once /arm/command std_msgs/msg/String \
  "{data: '{\\"id\\": 1, \\"command\\": \\"demo_cycle\\"}'}"
