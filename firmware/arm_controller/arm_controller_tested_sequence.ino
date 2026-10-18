/*
 * arm_controller.ino - PCA9685 12-bit PWM servo driver for 4-DOF robot arm.
 * Designed for the Kit4Curious / MeArm 4-DOF kit with PCA9685 I2C driver.
 *
 * Servo Channels on PCA9685:
 *   Channel 0: Bottom Base servo (turntable rotation)
 *   Channel 1: Left Shoulder servo
 *   Channel 2: Right Elbow servo
 *   Channel 3: Top Gripper servo
 *
 * Wiring to Arduino Uno:
 *   PCA9685 GND -> Arduino GND & External Power Supply GND (common ground)
 *   PCA9685 VCC -> Arduino 5V pin (powers the PCA9685 logic chip)
 *   PCA9685 SDA -> Arduino A4 (or dedicated SDA pin)
 *   PCA9685 SCL -> Arduino A5 (or dedicated SCL pin)
 *   PCA9685 OE  -> Arduino GND (Recommended: forces PWM outputs ON)
 *   PCA9685 V+  -> External 5V-6V Power Supply (+) (powers the servo motors)
 *   PCA9685 GND -> External Power Supply (-)
 *
 * Protocol (115200 baud, newline-terminated ASCII):
 *   host -> J,<base>,<shoulder>,<elbow>[,<wrist>]  move arm to angles (deg)
 *   host -> H                                      move arm to home pose
 *   host -> O | C                                  open / close gripper
 *   uno  -> READY                                  once after boot
 *   uno  -> OK                                     command finished
 *   uno  -> ERR,<reason>                           bad_command | bad_args | out_of_range
 */

#include <Wire.h>

#define PCA9685_ADDR 0x40
#define MODE1_REG    0x00
#define MODE2_REG    0x01
#define PRESCALE_REG 0xFE
#define LED0_ON_L    0x06

// Joint mapping
enum Joint : uint8_t { BASE = 0, SHOULDER = 1, ELBOW = 2, GRIPPER = 3, NUM_SERVOS = 4 };

// PCA9685 Channels
const uint8_t CHANNEL[NUM_SERVOS] = {0, 1, 2, 3};

// Pulse ticks for 12-bit PWM (0..4095) at 50 Hz (~20ms period).
// SG90 micro-servos use 600us (0 deg) to 2400us (180 deg).
// At 50 Hz, 1 tick = 20000us / 4096 = 4.88us.
// 600us  -> ~123 ticks (min)
// 1500us -> ~307 ticks (center 90 deg)
// 2400us -> ~491 ticks (max)
const uint16_t PULSE_MIN = 130;  // Safe 0 degree pulse
const uint16_t PULSE_MAX = 490;  // Safe 180 degree pulse

// Per-joint safe travel limits in degrees.
const float SERVO_MIN[NUM_SERVOS] = {10, 15, 15, 20};
const float SERVO_MAX[NUM_SERVOS] = {170, 165, 165, 160};

// Arm home pose (base, shoulder, elbow, gripper)
const float HOME_DEG[NUM_SERVOS] = {90, 90, 90, 50};

// Gripper positions in degrees (Channel 3)
const float GRIPPER_OPEN_DEG = 45;
const float GRIPPER_CLOSED_DEG = 110;

// Motion profile: smooth stepping (degrees per millisecond)
const float STEP_DEG = 1.0;
const unsigned long STEP_MS = 15;

float current[NUM_SERVOS];  // last commanded angle per joint
char line[64];             // fixed buffer: 63 chars + terminator
uint8_t lineLen = 0;
bool lineOverflow = false;

uint8_t pcaWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission();
}

uint8_t pcaRead8(uint8_t reg) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)PCA9685_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

void setPWM(uint8_t ch, uint16_t on, uint16_t off) {
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(LED0_ON_L + 4 * ch);
  Wire.write(on & 0xFF);
  Wire.write((on >> 8) & 0x0F);
  Wire.write(off & 0xFF);
  Wire.write((off >> 8) & 0x0F);
  Wire.endTransmission();
}

bool initPCA9685() {
  Wire.begin();

  // 1. Check if PCA9685 acknowledges I2C address 0x40
  Wire.beginTransmission(PCA9685_ADDR);
  if (Wire.endTransmission() != 0) {
    return false; // Device not responding
  }

  // 2. Put PCA9685 to sleep to set PRESCALE frequency
  pcaWrite8(MODE1_REG, 0x11); // SLEEP = 1, ALLCALL = 1
  delay(5);

  // 3. Set prescaler for 50 Hz PWM frequency
  // Formula: round(25000000 / (4096 * 50)) - 1 = 121
  pcaWrite8(PRESCALE_REG, 121);

  // 4. Wake up chip (clear SLEEP bit)
  pcaWrite8(MODE1_REG, 0x01); // ALLCALL = 1
  delay(5);

  // 5. Enable Auto-Increment (AI = 0x20) and restart
  pcaWrite8(MODE1_REG, 0x21); // AI = 1, ALLCALL = 1

  // 6. Set MODE2 to TOTEM-POLE / PUSH-PULL output (OUTDRV = 1, bit 2 = 0x04)
  // CRITICAL: Without this, outputs are Open-Drain and cannot drive 5V servo pulses!
  pcaWrite8(MODE2_REG, 0x04);

  return true;
}

uint16_t angleToPulse(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  return PULSE_MIN + (uint16_t)((deg / 180.0f) * (PULSE_MAX - PULSE_MIN));
}

void writeJoint(uint8_t i) {
  setPWM(CHANNEL[i], 0, angleToPulse(current[i]));
}

void fail(const __FlashStringHelper* reason) {
  Serial.print(F("ERR,"));
  Serial.println(reason);
}

// Plain decimal validator
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

// Blocking smooth move
void moveTo(const float target[NUM_SERVOS]) {
  bool busy = true;
  while (busy) {
    busy = false;
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      float d = target[i] - current[i];
      if (d >= STEP_DEG) { current[i] += STEP_DEG; busy = true; }
      else if (d <= -STEP_DEG) { current[i] -= STEP_DEG; busy = true; }
      else current[i] = target[i];
      writeJoint(i);
    }
    delay(STEP_MS);
  }
}

void handleLine(char* s) {
  char* cmd = strtok(s, ",");
  if (cmd == NULL) return;

  float target[NUM_SERVOS];
  for (uint8_t i = 0; i < NUM_SERVOS; i++) target[i] = current[i];

  if (strcmp(cmd, "J") == 0) {
    char* b_str = strtok(NULL, ",");
    char* s_str = strtok(NULL, ",");
    char* e_str = strtok(NULL, ",");
    char* w_str = strtok(NULL, ",");  // optional 4th value from host

    if (!b_str || !s_str || !e_str || !isNumber(b_str) || !isNumber(s_str) || !isNumber(e_str)) {
      fail(F("bad_args"));
      return;
    }

    target[BASE] = atof(b_str);
    target[SHOULDER] = atof(s_str);
    target[ELBOW] = atof(e_str);

    // Range checks
    for (uint8_t i = 0; i < 3; i++) {
      if (target[i] < SERVO_MIN[i] || target[i] > SERVO_MAX[i]) {
        fail(F("out_of_range"));
        return;
      }
    }
  } else if (strcmp(cmd, "H") == 0) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) target[i] = HOME_DEG[i];
  } else if (strcmp(cmd, "O") == 0) {
    target[GRIPPER] = GRIPPER_OPEN_DEG;
  } else if (strcmp(cmd, "C") == 0) {
    target[GRIPPER] = GRIPPER_CLOSED_DEG;
  } else {
    fail(F("bad_command"));
    return;
  }

  moveTo(target);
  Serial.println(F("OK"));
}

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

  bool ok = initPCA9685();
  if (ok) {
    Serial.println(F("PCA9685_OK"));
  } else {
    Serial.println(F("PCA9685_NOT_FOUND_CHECK_I2C"));
  }

  // Boot pose: all joints at home, gripper open
  for (uint8_t i = 0; i < NUM_SERVOS; i++) current[i] = HOME_DEG[i];
  current[GRIPPER] = GRIPPER_OPEN_DEG;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) writeJoint(i);

  Serial.println(F("READY"));
}

void loop() {
  readSerial();
}
