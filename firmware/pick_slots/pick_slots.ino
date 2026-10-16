#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 I2C address
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo channels
const uint8_t BASE    = 0;
const uint8_t LINK1   = 1;
const uint8_t LINK2   = 2;
const uint8_t GRIPPER = 3;

// Smooth movement settings
const int STEP_SIZE = 2;
const int STEP_DELAY_MS = 20;

// Calibrated home pose
const int HOME[4] = {330, 430, 400, 300};

// Gripper endpoints
const int GRIPPER_OPEN  = 300;
const int GRIPPER_CLOSE = 455;

// Pickup: base (channel 0) differs per slot.
// TODO: replace with your 4 measured base values (use B,<value> to find them).
const int PICKUP_BASE[4] = {150, 250, 350, 490};

// Pickup: channels 1 and 2 are constant for all slots
const int PICKUP_LINK1 = 500;
const int PICKUP_LINK2 = 535;
// Gripper at pickup = GRIPPER_OPEN

// Single drop position (base only; links stay at HOME).
// TODO: set to where you want to release the object.
const int DROP_BASE = 330;

// Current commanded pulse values
int currentPulse[4] = {330, 430, 400, 300};

bool stopped = false;
bool busy = false;

// --------------------------------------------------
// Set pulse on one PCA9685 channel
// --------------------------------------------------
void setServoPulse(uint8_t channel, int pulse) {
  pwm.setPWM(channel, 0, pulse);
}

// --------------------------------------------------
// Check for emergency stop command 'x'
// --------------------------------------------------
bool checkStop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == 'x' || c == 'X') {
      stopped = true;
      Serial.println("STOPPED");
      return true;
    }
  }

  return stopped;
}

// --------------------------------------------------
// Move all four channels smoothly.
// Each channel changes by at most STEP_SIZE counts per step.
// --------------------------------------------------
bool moveToPose(const int target[4]) {
  while (true) {
    if (checkStop()) return false;

    bool finished = true;

    for (int ch = 0; ch < 4; ch++) {
      if (currentPulse[ch] < target[ch]) {
        currentPulse[ch] += STEP_SIZE;
        if (currentPulse[ch] > target[ch]) currentPulse[ch] = target[ch];
        finished = false;
      }
      else if (currentPulse[ch] > target[ch]) {
        currentPulse[ch] -= STEP_SIZE;
        if (currentPulse[ch] < target[ch]) currentPulse[ch] = target[ch];
        finished = false;
      }

      setServoPulse(ch, currentPulse[ch]);
    }

    if (finished) return true;

    delay(STEP_DELAY_MS);
  }
}

// --------------------------------------------------
// Move only the gripper, keeping other channels unchanged
// --------------------------------------------------
bool moveGripper(int target) {
  int pose[4];

  for (int i = 0; i < 4; i++) {
    pose[i] = currentPulse[i];
  }

  pose[GRIPPER] = target;
  return moveToPose(pose);
}

// --------------------------------------------------
// Return to calibrated home pose
// --------------------------------------------------
bool goHome() {
  return moveToPose(HOME);
}

// --------------------------------------------------
// Pick object from the given slot, then return home
// while holding it.
// --------------------------------------------------
bool pickObject(uint8_t slot) {
  Serial.print("PICKUP_START,");
  Serial.print(slot);
  Serial.print(",base=");
  Serial.println(PICKUP_BASE[slot]);

  // Swing base to the slot with links at home and gripper open
  int approach[4] = {
    PICKUP_BASE[slot],
    HOME[LINK1],
    HOME[LINK2],
    GRIPPER_OPEN
  };
  if (!moveToPose(approach)) return false;

  if (!moveGripper(GRIPPER_OPEN)) return false;
  delay(300);

  // Lower linkage to the pickup pose
  int pickupPose[4] = {
    PICKUP_BASE[slot],
    PICKUP_LINK1,
    PICKUP_LINK2,
    GRIPPER_OPEN
  };
  if (!moveToPose(pickupPose)) return false;
  delay(300);

  // Grasp
  if (!moveGripper(GRIPPER_CLOSE)) return false;
  delay(300);

  // Return to home while gripping
  int carryHome[4] = {
    HOME[BASE],
    HOME[LINK1],
    HOME[LINK2],
    GRIPPER_CLOSE
  };
  if (!moveToPose(carryHome)) return false;

  Serial.println("PICKUP_COMPLETE");
  return true;
}

// --------------------------------------------------
// Place object at the single drop position
// --------------------------------------------------
bool placeObject() {
  Serial.println("HOLDING_AT_HOME");
  delay(3000);

  if (checkStop()) return false;

  int destination[4] = {
    DROP_BASE,
    HOME[LINK1],
    HOME[LINK2],
    GRIPPER_CLOSE
  };

  Serial.print("MOVING_TO_DROP,base=");
  Serial.println(DROP_BASE);

  if (!moveToPose(destination)) return false;
  delay(300);

  // Release
  if (!moveGripper(GRIPPER_OPEN)) return false;

  Serial.println("OBJECT_RELEASED");
  delay(500);

  if (!goHome()) return false;

  Serial.println("CYCLE_COMPLETE");
  return true;
}

// --------------------------------------------------
// Complete cycle: pick from slot, drop at DROP_BASE
// --------------------------------------------------
void runCycle(uint8_t slot) {
  if (busy) {
    Serial.println("ERR,busy");
    return;
  }

  if (slot > 3) {
    Serial.println("ERR,invalid_slot");
    return;
  }

  busy = true;
  stopped = false;

  Serial.println("CYCLE_START");

  if (!goHome()) {
    Serial.println("CYCLE_ABORTED");
    busy = false;
    return;
  }

  if (!pickObject(slot)) {
    Serial.println("CYCLE_ABORTED");
    busy = false;
    return;
  }

  if (!placeObject()) {
    Serial.println("CYCLE_ABORTED");
    busy = false;
    return;
  }

  busy = false;
}

// --------------------------------------------------
// Arduino setup
// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(500);

  // Initialize outputs at the calibrated home pose.
  // Support the arm before powering the servos.
  for (int ch = 0; ch < 4; ch++) {
    setServoPulse(ch, HOME[ch]);
  }

  delay(1000);

  Serial.println("ARM_READY");
  Serial.println("Commands:");
  Serial.println("H     - Go home");
  Serial.println("O     - Open gripper");
  Serial.println("C     - Close gripper");
  Serial.println("D,0   - Pick from slot 0, drop at DROP_BASE");
  Serial.println("D,1   - Pick from slot 1, drop at DROP_BASE");
  Serial.println("D,2   - Pick from slot 2, drop at DROP_BASE");
  Serial.println("D,3   - Pick from slot 3, drop at DROP_BASE");
  Serial.println("B,<n> - Move base to raw pulse n (100-600)");
  Serial.println("x     - Stop movement");
}

// --------------------------------------------------
// Serial command handling
// --------------------------------------------------
void loop() {
  if (!Serial.available()) return;

  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command.length() == 0) return;

  if (command.equalsIgnoreCase("x")) {
    stopped = true;
    Serial.println("STOPPED");
    return;
  }

  if (busy) {
    Serial.println("ERR,busy");
    return;
  }

  if (command.equalsIgnoreCase("H")) {
    stopped = false;
    goHome();
  }
  else if (command.equalsIgnoreCase("O")) {
    stopped = false;
    moveGripper(GRIPPER_OPEN);
  }
  else if (command.equalsIgnoreCase("C")) {
    stopped = false;
    moveGripper(GRIPPER_CLOSE);
  }
  else if (command.startsWith("D,")) {
    int slot = command.substring(2).toInt();

    if (slot >= 0 && slot <= 3) {
      runCycle((uint8_t)slot);
    } else {
      Serial.println("ERR,invalid_slot");
    }
  }
  else if (command.startsWith("B,")) {
    int p = command.substring(2).toInt();

    if (p >= 100 && p <= 600) {   // safe-ish bounds, adjust
      stopped = false;
      int pose[4] = {p, currentPulse[1], currentPulse[2], currentPulse[3]};
      moveToPose(pose);
      Serial.print("BASE,");
      Serial.println(p);
    } else {
      Serial.println("ERR,out_of_range");
    }
  }
  else {
    Serial.println("ERR,unknown_command");
  }
}
