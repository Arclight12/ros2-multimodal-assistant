#pragma once
struct ArmPose { int ch0; int ch1; int ch2; int ch3; };
// User-supplied tested endpoints. Home is also the handover pose.
const ArmPose HOME_POSE = {330, 430, 400, 300};
const ArmPose PICKUP_POSE = {460, 490, 460, 300};
const int GRIPPER_OPEN_PULSE = 300;
const int GRIPPER_CLOSE_PULSE = 460;
// Per-channel bounds only cover supplied endpoints.
const int CHANNEL_MIN[4] = {330, 430, 400, 300};
const int CHANNEL_MAX[4] = {460, 490, 460, 460};
const unsigned long STEP_DELAY_MS = 20;
const unsigned long HANDOVER_HOLD_MS = 3000;
