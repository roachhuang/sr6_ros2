#include <AccelStepper.h>

#define END_EFFECTOR_PIN 16
//enable pin for the axis 3, 2 and 1
#define EN321_PIN 32
#define EN4_PIN A8
#define EN5_PIN A2
#define EN6_PIN 38

// Pin definitions
const uint8_t PUL_PINS[6] = { 39, 43, 47, 46, A6, A0 };
const uint8_t DIR_PINS[6] = { 37, 41, 45, 48, A7, A1 };
const uint8_t EN_PINS[4] = { 32, A8, A2, 38 };  // EN321, EN4, EN5, EN6
const float GEAR_RATIOS[6] = {4.8, 4.0, 5.0, 2.8, 2.1, 1.0};
const float MICROSTEPS = 32; // Must match hardware config

// Stepper motor objects
AccelStepper steppers[6] = {
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[0], DIR_PINS[0]),
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[1], DIR_PINS[1]),
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[2], DIR_PINS[2]),
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[3], DIR_PINS[3]),
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[4], DIR_PINS[4]),
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[5], DIR_PINS[5])
};

// Conversion factors (degrees per step)
const double DEG_PER_STEP[6] = {
  360.0 / 200.0 / 32.0 / 4.8,
  360.0 / 200.0 / 32.0 / 4.0,
  360.0 / 200.0 / 32.0 / 5.0,
  360.0 / 200.0 / 32.0 / 2.8,
  360.0 / 200.0 / 32.0 / 2.1,
  360.0 / 200.0 / 32.0 / 1.0
};

// Current positions in degrees
float currentPositions[6] = { 0.0, -78.51, 73.90, 0.0, -90.0, 0.0 };

// Serial communication variables
const int MAX_INPUT_SIZE = 100;
char inputBuffer[MAX_INPUT_SIZE];
int inputIndex = 0;
bool newCommandReady = false;
const uint8_t ledPin = 13;  // the LED is connected to digital pin 13

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect needed for native USB.
  }
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  // Initialize enable pins
  pinMode(EN_PINS[0], OUTPUT);  // EN321
  pinMode(EN_PINS[1], OUTPUT);  // EN4
  pinMode(EN_PINS[2], OUTPUT);  // EN5
  pinMode(EN_PINS[3], OUTPUT);  // EN6

  // Configure steppers
  for (int i = 0; i < 6; i++) {
    steppers[i].setMaxSpeed(2000);     // steps/sec (adjust based on your requirements)
    steppers[i].setAcceleration(1000);  // steps/sec²
  }
  steppers[2].setPinsInverted(true, false, false);  // Invert direction pin
  // Set initial positions
  for (int i = 0; i < 6; i++) {
    steppers[i].setCurrentPosition(degreesToSteps(currentPositions[i], i));
  }
  disableMotors();
}

void loop() {
  // Non-blocking motor control
  for (int i = 0; i < 6; i++) {
    steppers[i].run();
  }

  // Handle serial communication
  serialEvent();
  if (newCommandReady) {
    processCommand(inputBuffer);
    newCommandReady = false;
  }
  // Periodically send current positions
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 100) {  // 10Hz update
    sendCurrentPositions();
    lastSend = millis();
  }
}

// Convert degrees to steps for a specific joint
long degreesToSteps(float degrees, int joint) {
  return round(degrees / DEG_PER_STEP[joint]);
}

// Convert steps to degrees for a specific joint
float stepsToDegrees(long steps, int joint) {
  return steps * DEG_PER_STEP[joint];
}

void enableMotors() {
  digitalWrite(EN_PINS[0], LOW);  // EN321
  digitalWrite(EN_PINS[1], LOW);  // EN4
  digitalWrite(EN_PINS[2], LOW);  // EN5
  digitalWrite(EN_PINS[3], LOW);  // EN6
}

void disableMotors() {
  digitalWrite(EN_PINS[0], HIGH);  // EN321
  digitalWrite(EN_PINS[1], HIGH);  // EN4
  digitalWrite(EN_PINS[2], HIGH);  // EN5
  digitalWrite(EN_PINS[3], HIGH);  // EN6
}

void moveToPosition(float targetPositions[6]) {
//   enableMotors();

  for (int i = 0; i < 6; i++) {
    long targetSteps = degreesToSteps(targetPositions[i], i);
    steppers[i].moveTo(targetSteps);

    // Special handling for joint 3 (index 2)
    // if (i == 2) {  // Joint 3 has inverse direction
    //   digitalWrite(DIR_PINS[i], (targetPositions[i] > currentPositions[i]) ? LOW : HIGH);
    // } else {
    //   digitalWrite(DIR_PINS[i], (targetPositions[i] > currentPositions[i]) ? HIGH : LOW);
    // }
  }
}

void serialEvent() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      inputBuffer[inputIndex] = '\0';
      inputIndex = 0;
      newCommandReady = true;
      return;
    } else if (inputIndex < MAX_INPUT_SIZE - 1) {
      inputBuffer[inputIndex++] = c;
    }
  }
}

void sendCurrentPositions() {
  Serial.print("f");
  for (int i = 0; i < 6; i++) {
    currentPositions[i] = stepsToDegrees(steppers[i].currentPosition(), i);
    Serial.print(currentPositions[i], 2);
    if (i < 5)
      Serial.print(",");
  }
  Serial.println();
}

// This function parses a string of comma-separated floats
void parseFloats(const char *str, float *out, int maxCount) {
  int idx = 0;
  bool negative = false;
  float value = 0.0f;
  float scale = 1.0f;
  bool decimal = false;

  while (*str && idx < maxCount) {
    char c = *str++;
    if (c == '-') {
      negative = true;
    } else if (c >= '0' && c <= '9') {
      value = value * 10.0f + (c - '0');
      if (decimal)
        scale *= 10.0f;
    } else if (c == '.') {
      decimal = true;
    } else if (c == ',') {
      out[idx++] = (negative ? -1 : 1) * (value / scale);
      value = 0.0f;
      scale = 1.0f;
      negative = false;
      decimal = false;
    }
  }
  if (idx < maxCount) {
    out[idx++] = (negative ? -1 : 1) * (value / scale);
  }
  while (idx < maxCount) {
    out[idx++] = 0.0f;
  }
}

// Keep your existing serialEvent() and parsing functions
// Add command processing in processCommand():
void processCommand(const char *cmd) {
  if (cmd[0] == 'g') {
    float targets[6];
    parseFloats(cmd + 1, targets, 6);
    Serial.println("a");
    // targets are in degrees
    moveToPosition(targets);
  }

  else if (strcmp(cmd, "eOn") == 0) {
    digitalWrite(END_EFFECTOR_PIN, HIGH);
  } else if (strcmp(cmd, "eOff") == 0) {
    digitalWrite(END_EFFECTOR_PIN, LOW);
  } else if (strcmp(cmd, "en") == 0) {
    enableMotors();
  } else if (strcmp(cmd, "dis") == 0) {
    disableMotors();
  }
}