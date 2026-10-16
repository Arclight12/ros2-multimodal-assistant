/*
 * arm_controller.ino - PCA9685 driver for the 4-channel coupled-linkage arm.
 *   ch0 base, ch1 + ch2 coupled linkage, ch3 gripper. I2C address 0x40.
 * The Pi does the vision; this sketch only walks recorded (ch1, ch2) pairs.
 * Calibration lives in arm_calibration.h.
 *
 * !! POWER !! Servos run from their own 5 V supply into the PCA9685 V+ rail,
 * with that supply's GND tied to the Uno GND. Never use the Uno 5 V pin.
 *
 * !! HOMING !! The Uno resets whenever the Pi opens the port, which leaves the
 * servos limp. Nothing moves until the host sends H, and the first H snaps the
 * servos straight to the home pose. Support the arm and place it at home by
 * hand first.
 *
 * Protocol: 115200 baud, newline-terminated ASCII.
 *   host -> H                         go home (first H after boot snaps)
 *   host -> P,<base_deg>,<reach_mm>,<level>
 *                                     level 0 = table height, 1 = lifted
 *   host -> O | C                     open / close gripper
 *   host -> x (any time while moving) freeze: keep holding, reply ERR,stopped
 *   uno  -> READY                     once after boot and a valid calibration
 *   uno  -> OK                        command finished
 *   uno  -> ERR,<reason>              bad_command | bad_args | out_of_range |
 *                                     not_homed | uncalibrated | stopped
 */

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#include "arm_calibration.h"

Adafruit_PWMServoDriver pwm(0x40);

const uint8_t CH_BASE = 0, CH_P1 = 1, CH_P2 = 2, CH_GRIP = 3;
const int TOP = ROWS - 1;

int cur[4] = {-1, -1, -1, -1};  // last COMMANDED pulse per channel, not measured
bool homed = false;
int curRow = 0;
float curReach = 0;

void put(uint8_t ch, int value) {
  if (value == cur[ch]) return;
  pwm.setPWM(ch, 0, value);
  cur[ch] = value;
}

// Straight-line glide of all four channels, one count per STEP_MS on the
// channel with the longest way to go. False if the host sent 'x'.
bool moveTo(const int target[4]) {
  int from[4], steps = 0;
  for (uint8_t c = 0; c < 4; c++) {
    from[c] = cur[c];
    steps = max(steps, abs(target[c] - from[c]));
  }
  for (int i = 1; i <= steps; i++) {
    for (uint8_t c = 0; c < 4; c++) {
      put(c, from[c] + (int)((long)(target[c] - from[c]) * i / steps));
    }
    delay(STEP_MS);
    if (Serial.available() && Serial.peek() == 'x') return false;
  }
  return true;
}

bool moveAxes(int base, int p1, int p2) {
  int t[4] = {base, p1, p2, cur[CH_GRIP]};
  return moveTo(t);
}

bool moveGrip(int grip) {
  int t[4] = {cur[CH_BASE], cur[CH_P1], cur[CH_P2], grip};
  return moveTo(t);
}

// Pair at a height row and reach, interpolated between the two neighbouring
// recorded columns only.
Pair pairAt(int row, float reach) {
  if (COLS == 1) return GRID[row][0];
  int j = 0;
  while (j < COLS - 2 && reach > REACH_MM[j + 1]) j++;
  float f = (reach - REACH_MM[j]) / (REACH_MM[j + 1] - REACH_MM[j]);
  f = constrain(f, 0.0, 1.0);
  Pair a = GRID[row][j], b = GRID[row][j + 1];
  return {a.p1 + (int)round(f * (b.p1 - a.p1)),
          a.p2 + (int)round(f * (b.p2 - a.p2))};
}

// Step through every adjacent row so no segment spans more than one recorded gap.
bool walkRows(int toRow) {
  while (curRow != toRow) {
    int next = curRow + (toRow > curRow ? 1 : -1);
    Pair p = pairAt(next, curReach);
    if (!moveAxes(cur[CH_BASE], p.p1, p.p2)) return false;
    curRow = next;
  }
  return true;
}

// Step through every recorded column between here and toReach (at row TOP).
bool walkReach(float toReach) {
  int dir = toReach > curReach ? 1 : -1;
  for (int j = dir > 0 ? 0 : COLS - 1; j >= 0 && j < COLS; j += dir) {
    float r = REACH_MM[j];
    bool between = dir > 0 ? (r > curReach && r < toReach)
                           : (r < curReach && r > toReach);
    if (!between) continue;
    Pair p = pairAt(TOP, r);
    if (!moveAxes(cur[CH_BASE], p.p1, p.p2)) return false;
    curReach = r;
  }
  Pair p = pairAt(TOP, toReach);
  if (!moveAxes(cur[CH_BASE], p.p1, p.p2)) return false;
  curReach = toReach;
  return true;
}

// Base and reach only change while lifted, so the gripper never sweeps the table.
bool travel(int basePulse, float reach, int row) {
  // After a freeze the arm is mid-segment: return to the last recorded pose.
  Pair here = pairAt(curRow, curReach);
  if (!moveAxes(cur[CH_BASE], here.p1, here.p2)) return false;
  bool turn = basePulse != cur[CH_BASE];
  bool slide = fabs(reach - curReach) >= 0.5;
  if (turn || slide) {
    if (!walkRows(TOP)) return false;
    if (turn && !moveAxes(basePulse, cur[CH_P1], cur[CH_P2])) return false;
    if (slide && !walkReach(reach)) return false;
  }
  return walkRows(row);
}

bool goHome() {
  if (homed) return travel(BASE_CENTER, REACH_MM[0], TOP);
  Pair h = GRID[TOP][0];
  put(CH_BASE, BASE_CENTER);
  put(CH_P1, h.p1);
  put(CH_P2, h.p2);
  put(CH_GRIP, GRIP_OPEN);
  curRow = TOP;
  curReach = REACH_MM[0];
  homed = true;
  delay(500);
  return true;
}

bool calibrationValid() {
  auto inRange = [](int v) { return v >= PULSE_MIN && v <= PULSE_MAX; };
  if (COLS < 1 || ROWS < 1) return false;
  if (!inRange(BASE_CENTER) || !inRange(GRIP_OPEN) || !inRange(GRIP_CLOSED))
    return false;
  for (int j = 0; j < COLS; j++) {
    if (j > 0 && REACH_MM[j] <= REACH_MM[j - 1]) return false;
    for (int r = 0; r < ROWS; r++) {
      if (!inRange(GRID[r][j].p1) || !inRange(GRID[r][j].p2)) return false;
    }
  }
  return true;
}

void reply(bool ok, const char *err) {
  if (ok) {
    Serial.println("OK");
    return;
  }
  Serial.print("ERR,");
  Serial.println(err);
}

// Returns true on success; otherwise sets *err.
bool handle(char *line, const char **err) {
  char *cmd = strtok(line, ",");
  if (!cmd || cmd[1]) {
    *err = "bad_command";
    return false;
  }
  if (*cmd == 'H') return goHome() ? true : (*err = "stopped", false);
  if (*cmd != 'P' && *cmd != 'O' && *cmd != 'C') {
    *err = "bad_command";
    return false;
  }
  if (!homed) {
    *err = "not_homed";
    return false;
  }
  if (*cmd == 'O' || *cmd == 'C') {
    if (GRIP_OPEN == GRIP_CLOSED) {
      *err = "uncalibrated";
      return false;
    }
    return moveGrip(*cmd == 'O' ? GRIP_OPEN : GRIP_CLOSED) ? true
                                                           : (*err = "stopped", false);
  }
  char *a = strtok(NULL, ",");
  char *b = strtok(NULL, ",");
  char *c = strtok(NULL, ",");
  if (!a || !b || !c || strtok(NULL, ",") || (*c != '0' && *c != '1') || c[1]) {
    *err = "bad_args";
    return false;
  }
  float deg = atof(a), reach = atof(b);
  float pulse = BASE_CENTER + deg * BASE_COUNTS_PER_DEG;
  if (!isfinite(pulse) || !isfinite(reach) || pulse < PULSE_MIN ||
      pulse > PULSE_MAX || reach < REACH_MM[0] - 0.5 ||
      reach > REACH_MM[COLS - 1] + 0.5) {
    *err = "out_of_range";
    return false;
  }
  reach = constrain(reach, REACH_MM[0], REACH_MM[COLS - 1]);
  return travel((int)round(pulse), reach, *c == '1' ? TOP : 0)
             ? true
             : (*err = "stopped", false);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  for (uint8_t c = 0; c < 4; c++) pwm.setPWM(c, 0, 4096);  // full off: limp
  if (!calibrationValid()) {
    Serial.println("ERR,bad_calibration");  // no READY: the Pi stays unavailable
    while (true) {}
  }
  Serial.println("READY");
}

void loop() {
  static char buf[48];
  static uint8_t len = 0;
  static bool overflow = false;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') {
      if (len < sizeof(buf) - 1) buf[len++] = ch;
      else overflow = true;
      continue;
    }
    buf[len] = 0;
    const char *err = "bad_command";
    bool ok = !overflow && len > 0 && handle(buf, &err);
    len = 0;
    overflow = false;
    if (!ok && !strcmp(err, "stopped")) {
      while (Serial.available()) Serial.read();  // drop the 'x' and anything queued
    }
    reply(ok, err);
  }
}
