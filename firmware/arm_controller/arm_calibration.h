#pragma once
struct ArmPose { int ch0; int ch1; int ch2; int ch3; };
// User-supplied tested endpoints. Home is also the handover pose.
// Pulse counts are PCA9685 counts at 50 Hz (1 count = ~4.9 us).
const ArmPose HOME_POSE = {330, 430, 400, 300};
const ArmPose PICKUP_POSE = {460, 490, 460, 300};
const int GRIPPER_OPEN_PULSE = 300;
const int GRIPPER_CLOSE_PULSE = 460;
// Per-channel bounds only cover the supplied endpoints, so every pose and every
// point of the straight path between them is inside this box.
//   CH1 pickup 490 is the "lower-height" preset. Alternate heights 490..520 are
//   NOT enabled: raise CHANNEL_MAX[1] and PICKUP_POSE.ch1 together, one pose at
//   a time, only after that pose has been run unloaded and validated.
//   CH3: 300 as the open limit and 400..460 as the closing limit are NOT
//   established. Closing on a real object has no feedback, so 460 may stall the
//   servo against the object; if it buzzes, lower GRIPPER_CLOSE_PULSE (>= 400).
//   The old reference pair (CH1, CH2) = (350, 300) is outside these bounds and
//   unused by the demo.
const int CHANNEL_MIN[4] = {330, 430, 400, 300};
const int CHANNEL_MAX[4] = {460, 490, 460, 460};
// Delay per update. Peak speed is 1 count per step (50 counts/s); moves ease in
// and out, so a 130 count move takes ~3.9 s and the whole D cycle ~20 s.
const unsigned long STEP_DELAY_MS = 20;
const unsigned long HANDOVER_HOLD_MS = 3000;
