#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm(0x40);

const int MIN_PULSE = 225;
const int MAX_PULSE = 389;

int position[4] = {307, 307, 307, 307};
bool positionSet[4] = {false, false, false, false};

int selectedChannel = 1;
int stepSize = 1;
int stepDelayMs = 200;
bool active[4] = {false, false, false, false};

void disableAll() {
  for (int ch = 0; ch < 4; ch++) {
    pwm.setPWM(ch, 0, 4096);
    active[ch] = false;
  }
  Serial.println("All outputs disabled.");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  disableAll();

  Serial.println("\nSLOW SERVO CONTROLLER");
  Serial.println("0-3 : Select servo channel");
  Serial.println("r   : Enter starting pulse");
  Serial.println("e   : Enable selected servo");
  Serial.println("+   : Increase pulse slowly");
  Serial.println("-   : Decrease pulse slowly");
  Serial.println("x   : Disable all outputs");
  Serial.println("s   : Print the ch1,ch2 pair to record");
  Serial.println("p   : Print settings");
}

void loop() {
  if (!Serial.available()) return;

  char c = Serial.read();

  if (c == '\n' || c == '\r' || c == ' ') return;

  if (c == 'x') {
    disableAll();
    return;
  }

  if (c >= '0' && c <= '3') {
    selectedChannel = c - '0';
    Serial.print("Selected channel: ");
    Serial.println(selectedChannel);
    return;
  }

  if (c == 'r') {
    if (active[selectedChannel]) {
      Serial.println("Channel is live. Enter x first.");
      return;
    }

    Serial.println("Enter pulse value between 225 and 389:");

    while (!Serial.available()) {}
    int value = Serial.parseInt();

    while (Serial.available()) Serial.read();

    if (value < MIN_PULSE || value > MAX_PULSE) {
      Serial.println("Invalid value. No change made.");
      return;
    }

    position[selectedChannel] = value;
    positionSet[selectedChannel] = true;

    Serial.print("Stored pulse: ");
    Serial.println(value);
    Serial.println("Servo has NOT moved.");
    return;
  }

  if (c == 'e') {
    if (!positionSet[selectedChannel]) {
      Serial.println("Set a starting pulse with r first.");
      return;
    }

    Serial.println(
      "WARNING: This will command the selected servo to its stored pulse."
    );
    Serial.println(
      "Ensure the horn is disconnected and the servo is unloaded."
    );

    pwm.setPWM(
      selectedChannel, 0, position[selectedChannel]
    );

    active[selectedChannel] = true;
    Serial.println("Selected servo output enabled.");
    return;
  }

  if (c == '+' || c == '-') {
    if (!active[selectedChannel]) {
      Serial.println("Enable this channel with e first.");
      return;
    }

    int next = position[selectedChannel];

    if (c == '+') next += stepSize;
    else next -= stepSize;

    if (next < MIN_PULSE || next > MAX_PULSE) {
      Serial.println("Pulse limit reached.");
      return;
    }

    position[selectedChannel] = next;

    pwm.setPWM(
      selectedChannel, 0, position[selectedChannel]
    );

    Serial.print("Pulse: ");
    Serial.println(position[selectedChannel]);

    delay(stepDelayMs);
    return;
  }

  if (c == 's') {
    if (!active[1] || !active[2]) {
      Serial.println("Enable channels 1 and 2 first.");
      return;
    }
    Serial.print("PAIR,");
    Serial.print(position[1]);
    Serial.print(",");
    Serial.println(position[2]);
    return;
  }

  if (c == 'p') {
    Serial.print("Channel: ");
    Serial.println(selectedChannel);
    Serial.print("Pulse: ");
    Serial.println(position[selectedChannel]);
    Serial.print("Position set: ");
    Serial.println(positionSet[selectedChannel] ? "yes" : "no");
    Serial.print("Output enabled: ");
    Serial.println(active[selectedChannel] ? "yes" : "no");
  }
}