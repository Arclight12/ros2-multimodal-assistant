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
const int GRIPPER_CLOSE = 435; // Default ~4-5mm tighter closure


// Hard safety limits on base rotation (prevents over-rotation or wire wrapping)
const int MIN_SAFE_BASE = 230; // Far-Right reach limit
const int MAX_SAFE_BASE = 440; // Far-Left reach limit

// Predefined slots: base pulses for Slot 0 (Right) to Slot 3 (Left)
const int PICKUP_BASE[4] = {230, 300, 370, 440};

// Pickup: channels 1 and 2 are constant for all slots along the arc
const int PICKUP_LINK1 = 500;
const int PICKUP_LINK2 = 540;

// Drop position (base only; links stay at HOME).
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
// Move all four channels smoothly with synchronized proportional interpolation.
// All active servos start and finish their movements at the exact same instant,
// producing smooth, straight-line Cartesian trajectories with zero late-phase drag.
// --------------------------------------------------
bool moveToPose(const int target[4]) {
  int start[4];
  int maxDelta = 0;
  for (int ch = 0; ch < 4; ch++) {
    start[ch] = currentPulse[ch];
    int delta = abs(target[ch] - start[ch]);
    if (delta > maxDelta) maxDelta = delta;
  }

  if (maxDelta == 0) return true;

  int totalSteps = (maxDelta + STEP_SIZE - 1) / STEP_SIZE;
  if (totalSteps < 1) totalSteps = 1;

  for (int step = 1; step <= totalSteps; step++) {
    if (checkStop()) return false;

    for (int ch = 0; ch < 4; ch++) {
      long p = (long)start[ch] + ((long)(target[ch] - start[ch]) * step) / totalSteps;
      currentPulse[ch] = (int)p;
      setServoPulse(ch, currentPulse[ch]);
    }

    delay(STEP_DELAY_MS);
  }

  // Ensure exact target counts at final step
  for (int ch = 0; ch < 4; ch++) {
    currentPulse[ch] = target[ch];
    setServoPulse(ch, currentPulse[ch]);
  }

  return true;
}

// --------------------------------------------------
// Move only the gripper, keeping other channels unchanged
// --------------------------------------------------
bool moveGripper(int targetPulse) {
  int target[4] = {
    currentPulse[BASE],
    currentPulse[LINK1],
    currentPulse[LINK2],
    targetPulse
  };

  return moveToPose(target);
}

// --------------------------------------------------
// Return all channels to calibrated HOME
// --------------------------------------------------
bool goHome() {
  Serial.println("GOING_HOME");
  return moveToPose(HOME);
}

// --------------------------------------------------
// Pick object at arbitrary base and reach pulses (2D continuous workspace)
// --------------------------------------------------
bool pickObjectWithPose(int basePulse, int link1Pulse, int link2Pulse, int closePulse = GRIPPER_CLOSE) {
  // Hard safety check
  if (basePulse < MIN_SAFE_BASE || basePulse > MAX_SAFE_BASE) {
    Serial.println(F("ERR,base_out_of_range"));
    return false;
  }
  if (link1Pulse < 390 || link1Pulse > 510) {
    Serial.println(F("ERR,link1_out_of_range"));
    return false;
  }
  if (link2Pulse < 400 || link2Pulse > 550) {
    Serial.println(F("ERR,link2_out_of_range"));
    return false;
  }

  int gClose = (closePulse >= 350 && closePulse <= 480) ? closePulse : GRIPPER_CLOSE;

  Serial.print(F("PICKUP_START,base="));
  Serial.print(basePulse);
  Serial.print(F(",link1="));
  Serial.print(link1Pulse);
  Serial.print(F(",link2="));
  Serial.print(link2Pulse);
  Serial.print(F(",grip="));
  Serial.println(gClose);

  // 1. Approach: base rotates to commanded angle while links stay safely elevated at HOME
  int approach[4] = {
    basePulse,
    HOME[LINK1],
    HOME[LINK2],
    GRIPPER_OPEN
  };
  if (!moveToPose(approach)) return false;

  if (!moveGripper(GRIPPER_OPEN)) return false;
  delay(300);

  // 2. Lower linkage to tabletop pickup pose
  int pickupPose[4] = {
    basePulse,
    link1Pulse,
    link2Pulse,
    GRIPPER_OPEN
  };
  if (!moveToPose(pickupPose)) return false;
  delay(300);

  // 3. Grasp object
  if (!moveGripper(gClose)) return false;
  delay(300);

  // 4. Elevate vertically at pickup angle before rotating to drop
  int liftPose[4] = {
    basePulse,
    HOME[LINK1],
    HOME[LINK2],
    gClose
  };
  if (!moveToPose(liftPose)) return false;
  delay(200);

  // 5. Return to home base while gripping
  int carryHome[4] = {
    HOME[BASE],
    HOME[LINK1],
    HOME[LINK2],
    gClose
  };
  if (!moveToPose(carryHome)) return false;

  Serial.println(F("PICKUP_COMPLETE"));
  return true;
}

bool pickObjectWithBase(int basePulse) {
  return pickObjectWithPose(basePulse, PICKUP_LINK1, PICKUP_LINK2, GRIPPER_CLOSE);
}

// Legacy discrete slot pickup
bool pickObject(uint8_t slot) {
  if (slot > 3) return false;
  return pickObjectWithBase(PICKUP_BASE[slot]);
}

// --------------------------------------------------
// Place object at the drop position
// --------------------------------------------------
bool placeObject(int closePulse = GRIPPER_CLOSE) {
  Serial.println("HOLDING_AT_HOME");
  delay(1500);

  if (checkStop()) return false;

  int gClose = (closePulse >= 350 && closePulse <= 480) ? closePulse : GRIPPER_CLOSE;
  int destination[4] = {
    DROP_BASE,
    HOME[LINK1],
    HOME[LINK2],
    gClose
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
// Run cycle with arbitrary base and reach pulses (2D continuous workspace)
// --------------------------------------------------
void runCycleWithPose(int basePulse, int link1Pulse, int link2Pulse, int closePulse = GRIPPER_CLOSE) {
  if (busy) {
    Serial.println(F("ERR,busy"));
    return;
  }

  if (basePulse < MIN_SAFE_BASE || basePulse > MAX_SAFE_BASE) {
    Serial.println(F("ERR,base_out_of_range"));
    return;
  }
  if (link1Pulse < 390 || link1Pulse > 510) {
    Serial.println(F("ERR,link1_out_of_range"));
    return;
  }
  if (link2Pulse < 400 || link2Pulse > 550) {
    Serial.println(F("ERR,link2_out_of_range"));
    return;
  }

  busy = true;
  stopped = false;

  Serial.println(F("CYCLE_START"));

  if (!goHome()) {
    Serial.println(F("CYCLE_ABORTED"));
    busy = false;
    return;
  }

  if (!pickObjectWithPose(basePulse, link1Pulse, link2Pulse, closePulse)) {
    Serial.println(F("CYCLE_ABORTED"));
    busy = false;
    return;
  }

  if (!placeObject(closePulse)) {
    Serial.println(F("CYCLE_ABORTED"));
    busy = false;
    return;
  }

  busy = false;
}


// Run cycle with single base pulse along outer arc (backward compatible)
void runCycleWithBase(int basePulse) {
  runCycleWithPose(basePulse, PICKUP_LINK1, PICKUP_LINK2);
}

// --------------------------------------------------
// Run cycle with slot index (0..3)
// --------------------------------------------------
void runCycle(uint8_t slot) {
  if (slot > 3) {
    Serial.println(F("ERR,invalid_slot"));
    return;
  }
  runCycleWithBase(PICKUP_BASE[slot]);
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

  for (int ch = 0; ch < 4; ch++) {
    setServoPulse(ch, HOME[ch]);
  }

  delay(1000);

  Serial.println(F("ARM_READY"));
  Serial.println(F("Commands:"));
  Serial.println(F("H            - Go home"));
  Serial.println(F("O            - Open gripper"));
  Serial.println(F("C            - Close gripper"));
  Serial.println(F("D,<slot>     - Pick from slot 0..3 (230, 300, 370, 440)"));
  Serial.println(F("P,<base>     - Continuous outer arc pick (230-440)"));
  Serial.println(F("P,<b,ch1,ch2>- 2D continuous pick: base, link1, link2"));
  Serial.println(F("B,<n>        - Move base only (230-440)"));
  Serial.println(F("x            - Stop movement"));
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
    Serial.println(F("STOPPED"));
    return;
  }

  if (busy) {
    Serial.println(F("ERR,busy"));
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
      Serial.println(F("ERR,invalid_slot"));
    }
  }
  else if (command.startsWith("P,")) {
    String args = command.substring(2);
    int firstComma = args.indexOf(',');
    if (firstComma == -1) {
      int p = args.toInt();
      if (p >= MIN_SAFE_BASE && p <= MAX_SAFE_BASE) {
        runCycleWithBase(p);
      } else {
        Serial.println(F("ERR,base_out_of_range"));
      }
    } else {
      int secondComma = args.indexOf(',', firstComma + 1);
      if (secondComma != -1) {
        int baseP = args.substring(0, firstComma).toInt();
        int link1P = args.substring(firstComma + 1, secondComma).toInt();
        int thirdComma = args.indexOf(',', secondComma + 1);
        int link2P = (thirdComma == -1) ? args.substring(secondComma + 1).toInt() : args.substring(secondComma + 1, thirdComma).toInt();
        int gripP = (thirdComma == -1) ? GRIPPER_CLOSE : args.substring(thirdComma + 1).toInt();
        runCycleWithPose(baseP, link1P, link2P, gripP);
      } else {
        Serial.println(F("ERR,invalid_format"));
      }
    }

  }
  else if (command.startsWith("B,")) {
    int p = command.substring(2).toInt();
    if (p >= MIN_SAFE_BASE && p <= MAX_SAFE_BASE) {
      stopped = false;
      int pose[4] = {p, currentPulse[1], currentPulse[2], currentPulse[3]};
      moveToPose(pose);
      Serial.print(F("BASE,"));
      Serial.println(p);
    } else {
      Serial.println(F("ERR,base_out_of_range"));
    }
  }
  else {
    Serial.println(F("ERR,unknown_command"));
  }
}
