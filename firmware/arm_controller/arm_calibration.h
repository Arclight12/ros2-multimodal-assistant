#pragma once
/*
 * arm_calibration.h - every measured value for the PCA9685 arm.
 *
 * !! EVERYTHING BELOW EXCEPT THE (350,300) REFERENCE PAIR IS A PLACEHOLDER !!
 * The firmware refuses to report READY if a table is inconsistent, and it
 * refuses O/C until the gripper pulses differ. Fill the grid by walking the
 * linkage in small steps with the jog sketch (see docs/arm.md), recording each
 * pair that moves without buzzing. Never fill a pair you did not command and
 * listen to.
 *
 * Pulse counts are PCA9685 counts at 50 Hz (4096 per 20 ms), not degrees.
 */

// Hard software limits for every channel (the arm's proven-safe range).
const int PULSE_MIN = 225;
const int PULSE_MAX = 389;

// ch1 (vertical-ish linkage) and ch2 (forward-ish linkage) are mechanically
// coupled, so they are only ever commanded as a recorded pair.
struct Pair {
  int p1;
  int p2;
};

// Grid of recorded pairs. Rows are heights: row 0 = gripper at table height
// (grasp), row ROWS-1 = lifted (also home). Columns are reach from the base
// axis in mm, ascending. Adjacent rows and adjacent columns must be close
// enough that the straight line between them is mechanically free; add rows or
// columns until that is true. Home = GRID[ROWS-1][0].
const int ROWS = 1;
const int COLS = 1;
const float REACH_MM[COLS] = {0};
const Pair GRID[ROWS][COLS] = {
    {{350, 300}},  // confirmed reference pair
};

// Base (ch0): pulse = BASE_CENTER + degrees * BASE_COUNTS_PER_DEG, degrees
// measured from straight ahead, positive to the robot's left. A negative
// BASE_COUNTS_PER_DEG reverses the direction.
const int BASE_CENTER = 307;
const float BASE_COUNTS_PER_DEG = 2.0;  // UNMEASURED: roughly 0.45 deg/count

// Gripper (ch3). Equal values mean "not calibrated": O and C are refused.
const int GRIP_OPEN = 307;
const int GRIP_CLOSED = 307;

// Delay between 1-count updates: ~40 counts/s on the fastest channel.
const int STEP_MS = 25;
