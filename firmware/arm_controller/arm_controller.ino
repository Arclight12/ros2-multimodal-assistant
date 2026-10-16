/*
 Fixed-pose demo: D=pickup, return home, hold 3s, release; H=home/open;
 O=open; C=close; x=stop commanded movement.
 CH0 base, CH1/CH2 coupled linkage, CH3 gripper; PCA9685 at 0x40.
 Before boot, physically support/place arm at HOME_POSE. No feedback sensors:
 pulses are commanded positions, not measured positions. Test path unloaded.
 Cut servo power immediately if any servo buzzes, grinds, or binds.
 Servo supply must be separate regulated 5V on PCA9685 V+, common GND with Uno.

 Serial protocol (115200 baud, newline-terminated ASCII, one command at a time):
   host -> D | H | O | C        run it; the Uno replies exactly ONE of
   uno  -> OK                   the command finished
   uno  -> ERR,<reason>         bad_command | out_of_range | stopped
   host -> x                    freeze while moving: the running command then
                                replies ERR,stopped. Idle x gets no reply.
   uno  -> READY                once after boot, if the calibration is valid
   uno  -> STATE,<name>         progress lines during D (informational)
 Bytes other than 'x' that arrive while a move is running are discarded, so the
 host must wait for OK/ERR before sending the next command.
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "arm_calibration.h"
Adafruit_PWMServoDriver pwm(0x40);
int cur[4] = {HOME_POSE.ch0, HOME_POSE.ch1, HOME_POSE.ch2, GRIPPER_OPEN_PULSE};
bool stopping = false;
const char *err = nullptr;  // reason for the reply to the current command

int pv(const ArmPose &p, int c) {
  if (c == 0) return p.ch0;
  if (c == 1) return p.ch1;
  if (c == 2) return p.ch2;
  return p.ch3;
}
bool valid(int c, int v) {
  return c >= 0 && c < 4 && v >= CHANNEL_MIN[c] && v <= CHANNEL_MAX[c];
}
bool fail(const char *why) {
  err = why;
  return false;
}
void writeAll() {
  for (int c = 0; c < 4; c++) pwm.setPWM(c, 0, cur[c]);
}
bool checkStop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'x' || c == 'X') stopping = true;
  }
  return stopping;
}
// Glide all four channels together to t[]. Every channel is checked against its
// limits before anything moves, and every intermediate value lies between the
// start and the target, so the path never leaves the box the endpoints span.
// Smoothstep easing: velocity ramps up from zero and back down to zero. The
// move takes 1.5x the steps of the longest channel, which keeps the PEAK speed
// at 1 count per STEP_DELAY_MS, the same as the old constant-speed glide.
bool moveTo(const int t[4]) {
  int from[4], longest = 0;
  for (int c = 0; c < 4; c++) {
    if (!valid(c, t[c])) return fail("out_of_range");
    from[c] = cur[c];
    longest = max(longest, abs(t[c] - from[c]));
  }
  if (longest > 0) {
    int n = (longest * 3 + 1) / 2;
    for (int s = 1; s <= n; s++) {
      if (checkStop()) return fail("stopped");
      float u = (float)s / n;
      float e = u * u * (3.0f - 2.0f * u);
      for (int c = 0; c < 4; c++) {
        cur[c] = from[c] + (int)round((t[c] - from[c]) * e);
      }
      writeAll();
      delay(STEP_DELAY_MS);
    }
  }
  for (int c = 0; c < 4; c++) cur[c] = t[c];
  writeAll();
  return true;
}
bool pose(const ArmPose &p, int grip) {
  int t[4] = {p.ch0, p.ch1, p.ch2, grip};
  return moveTo(t);
}
bool grip(int v) {
  int t[4] = {cur[0], cur[1], cur[2], v};
  return moveTo(t);
}
bool runDemo() {
  Serial.println("STATE,open");
  if (!grip(GRIPPER_OPEN_PULSE)) return false;
  Serial.println("STATE,pickup");
  if (!pose(PICKUP_POSE, GRIPPER_OPEN_PULSE)) return false;
  Serial.println("STATE,close");
  if (!grip(GRIPPER_CLOSE_PULSE)) return false;
  Serial.println("STATE,return_home");
  if (!pose(HOME_POSE, GRIPPER_CLOSE_PULSE)) return false;
  Serial.println("STATE,hold_3s");
  for (unsigned long t = 0; t < HANDOVER_HOLD_MS; t += 20) {
    if (checkStop()) return fail("stopped");
    delay(20);
  }
  Serial.println("STATE,release");
  if (!grip(GRIPPER_OPEN_PULSE)) return false;
  Serial.println("STATE,complete");
  return true;
}
bool home() { return pose(HOME_POSE, GRIPPER_OPEN_PULSE); }
bool calibrationOK() {
  for (int c = 0; c < 4; c++) {
    if (CHANNEL_MIN[c] > CHANNEL_MAX[c] || !valid(c, pv(HOME_POSE, c)) ||
        !valid(c, pv(PICKUP_POSE, c)))
      return false;
  }
  return valid(3, GRIPPER_OPEN_PULSE) && valid(3, GRIPPER_CLOSE_PULSE);
}
void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  for (int c = 0; c < 4; c++) pwm.setPWM(c, 0, 4096);  // outputs off until first command
  if (!calibrationOK()) {
    Serial.println("ERR,bad_calibration");
    while (true) delay(1000);
  }
  Serial.println("READY");
}
void loop() {
  static char b[24];
  static int n = 0;
  static bool overflow = false;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (n < (int)sizeof(b) - 1) b[n++] = c;
      else overflow = true;
      continue;
    }
    b[n] = '\0';
    int len = n;
    bool bad = overflow;
    n = 0;
    overflow = false;
    if (len == 0 && !bad) continue;  // blank line: no command, no reply
    if (bad || len != 1) {
      Serial.println("ERR,bad_command");
      continue;
    }
    char k = b[0];
    if (k == 'x' || k == 'X') {  // idle freeze: nothing is moving, no reply
      stopping = true;
      continue;
    }
    bool ok = false;
    err = nullptr;
    stopping = false;
    if (k == 'D') ok = runDemo();
    else if (k == 'H') ok = home();
    else if (k == 'O') ok = grip(GRIPPER_OPEN_PULSE);
    else if (k == 'C') ok = grip(GRIPPER_CLOSE_PULSE);
    else err = "bad_command";
    if (ok) {
      Serial.println("OK");
    } else {
      Serial.print("ERR,");
      Serial.println(err ? err : "stopped");
    }
  }
}
