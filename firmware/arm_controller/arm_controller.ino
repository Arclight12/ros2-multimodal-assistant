/*
 * arm_controller.ino - dumb servo driver for a 4-DOF arm + gripper.
 * All kinematics run on the host (Raspberry Pi, ROS 2). This sketch only
 * moves servos, one degree at a time, and reports back over serial.
 *
 * !! WIRING / POWER WARNING !!
 *   Servos must run from their OWN 5-6 V supply, sized for stall current,
 *   with that supply's GND tied to the Uno GND.
 *   NEVER power the servos from the Uno 5 V pin. It browns out the board
 *   and can damage it.
 *
 * Protocol: 115200 baud, newline-terminated ASCII.
 *   host -> J,<base>,<shoulder>,<elbow>,<wrist>   move arm to angles (deg)
 *   host -> H                                     move arm to home pose
 *   host -> O | C                                 open / close gripper
 *   uno  -> READY                                 once after boot
 *   uno  -> OK                                    command finished
 *   uno  -> ERR,<reason>                          bad_command | bad_args | out_of_range
 */

#include <Servo.h>

// ===== MEASURE ON YOUR ARM ================================================
// Joint order used everywhere below.
enum Joint : uint8_t { BASE, SHOULDER, ELBOW, WRIST, GRIPPER, NUM_SERVOS };
const uint8_t NUM_ARM = 4;  // BASE..WRIST

// Signal pins (Uno digital pins).
const uint8_t SERVO_PIN[NUM_SERVOS] = {3, 5, 6, 9, 10};

// Per-joint safe travel limits in degrees. Commands outside these are rejected.
const float SERVO_MIN[NUM_SERVOS] = {10, 20, 20, 10, 20};
const float SERVO_MAX[NUM_SERVOS] = {170, 160, 160, 170, 160};

// Arm home pose (base, shoulder, elbow, wrist).
const float HOME_DEG[NUM_ARM] = {90, 90, 90, 90};

// Gripper positions in degrees.
const float GRIPPER_OPEN_DEG = 40;
const float GRIPPER_CLOSED_DEG = 100;

// Motion profile: every joint moves STEP_DEG per STEP_MS milliseconds.
const float STEP_DEG = 1.0;
const unsigned long STEP_MS = 15;
// ==========================================================================

Servo servos[NUM_SERVOS];
float current[NUM_SERVOS];  // last commanded angle per joint
char line[64];             // fixed buffer: 63 chars + terminator
uint8_t lineLen = 0;
bool lineOverflow = false;

void fail(const __FlashStringHelper* reason) {
  Serial.print(F("ERR,"));
  Serial.println(reason);
}

// Plain decimal: optional '-', digits, at most one '.', at least one digit.
bool isNumber(const char* s) {
  bool digit = false, dot = false;
  if (*s == '-') s++;
  for (; *s; s++) {
    if (*s >= '0' && *s <= '9') digit = true;
    else if (*s == '.' && !dot) dot = true;
    else return false;
  }
  return digit;
}

void writeJoint(uint8_t i) {
  servos[i].write((int)(current[i] + 0.5f));
}

// Blocking move: all joints step together toward target[] until all arrive.
void moveTo(const float target[NUM_SERVOS]) {
  bool busy = true;
  while (busy) {
    busy = false;
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      float d = target[i] - current[i];
      if (d >= STEP_DEG) { current[i] += STEP_DEG; busy = true; }
      else if (d <= -STEP_DEG) { current[i] -= STEP_DEG; busy = true; }
      else current[i] = target[i];  // last sub-degree step: land exactly
      writeJoint(i);
    }
    delay(STEP_MS);
  }
}

// Parses one complete line. Nothing moves unless the whole line is valid.
void handleLine(char* s) {
  char* cmd = strtok(s, ",");
  if (cmd == NULL) return;  // empty line: ignore

  float target[NUM_SERVOS];
  for (uint8_t i = 0; i < NUM_SERVOS; i++) target[i] = current[i];

  if (strcmp(cmd, "J") == 0) {
    for (uint8_t i = 0; i < NUM_ARM; i++) {
      char* tok = strtok(NULL, ",");
      if (tok == NULL || !isNumber(tok)) { fail(F("bad_args")); return; }
      target[i] = atof(tok);
    }
    if (strtok(NULL, ",") != NULL) { fail(F("bad_args")); return; }
    // Clamp-reject: check every joint before moving any.
    for (uint8_t i = 0; i < NUM_ARM; i++) {
      if (target[i] < SERVO_MIN[i] || target[i] > SERVO_MAX[i]) {
        fail(F("out_of_range"));
        return;
      }
    }
  } else if (strcmp(cmd, "H") == 0 || strcmp(cmd, "O") == 0 || strcmp(cmd, "C") == 0) {
    if (strtok(NULL, ",") != NULL) { fail(F("bad_args")); return; }
    if (cmd[0] == 'H') {
      for (uint8_t i = 0; i < NUM_ARM; i++) target[i] = HOME_DEG[i];
    } else {
      target[GRIPPER] = (cmd[0] == 'O') ? GRIPPER_OPEN_DEG : GRIPPER_CLOSED_DEG;
    }
  } else {
    fail(F("bad_command"));
    return;
  }

  moveTo(target);
  Serial.println(F("OK"));
}

// Collects bytes into line[]. Ignores '\r'. An overlong line is discarded
// and answered with ERR,bad_command when its newline arrives.
void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (lineOverflow) {
        fail(F("bad_command"));
      } else {
        line[lineLen] = '\0';
        handleLine(line);
      }
      lineLen = 0;
      lineOverflow = false;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineOverflow = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PIN[i]);
  }

  // Boot pose: arm at home, gripper open. Host starts commanding after READY.
  for (uint8_t i = 0; i < NUM_ARM; i++) current[i] = HOME_DEG[i];
  current[GRIPPER] = GRIPPER_OPEN_DEG;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) writeJoint(i);

  Serial.println(F("READY"));
}

void loop() {
  readSerial();
}
