#include <AccelStepper.h>
#include "define.h"

// Stepper motor objects
AccelStepper steppers[6] = {
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[0], DIR_PINS[0]),
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[1], DIR_PINS[1]),
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[2], DIR_PINS[2]),
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[3], DIR_PINS[3]),
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[4], DIR_PINS[4]),
  AccelStepper(AccelStepper::DRIVER, PUL_PINS[5], DIR_PINS[5])
};

// Current positions in degrees
float currentPositions[6] = { 0.0, -78.51, 73.90, 0.0, -90.0, 0.0 };

char inputBuffer[MAX_INPUT_SIZE];
int inputIndex = 0;
bool newCommandReady = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect needed for native USB.
  }
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // this is essential for the stepper drivers to work
  pinMode(EN_PINS[0], OUTPUT);  // EN321
  pinMode(EN_PINS[1], OUTPUT);  // EN4
  pinMode(EN_PINS[2], OUTPUT);  // EN5
  pinMode(EN_PINS[3], OUTPUT);  // EN6

  // Set pin modes for limit switches (INPUT_PULLUP for active-low switches)
  pinMode(Z_MIN_PIN, INPUT_PULLUP);
  pinMode(Z_MAX_PIN, INPUT_PULLUP);
  steppers[2].setPinsInverted(true, false, false);  // Invert direction pin for joint3

  enableMotors(true);
  delay(1000);

  // Configure steppers
  for (int i = 0; i < 6; i++) {
    steppers[i].setMaxSpeed(1000);     // steps/sec (adjust based on your requirements)
    steppers[i].setAcceleration(500);  // steps/sec²
  }
  
  
  homeJoint(Z_MAX_PIN, 1);  // Home joint 2 using limit switch  
  delay(1000);
  // Configure steppers
  for (int i = 0; i < 6; i++) {
    steppers[i].setMaxSpeed(1000);     // steps/sec (adjust based on your requirements)
    steppers[i].setAcceleration(500);  // steps/sec²
  }
  

  // Set initial positions
  for (int i = 0; i < 6; i++) {
    steppers[i].setCurrentPosition(degreesToSteps(currentPositions[i], i));
  }
  // disableMotors();
}

void loop() {
  // Handle serial communication
  serialEvent();
  if (newCommandReady) {
    processCommand(inputBuffer);
    newCommandReady = false;
  }

  // Non-blocking motor control. don't move it into processcommand!
  for (int i = 0; i < 6; i++) {
    steppers[i].run();
  }

  // Periodically send current positions (10hz). must be consistent with publish_rate in robot_arm_controller.yaml.
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 200) {  // 5Hz update
    // sendCurrentPositions();
    lastSend = millis();
    // digitalWrite(ledPin, digitalRead(Z_MAX_PIN));
  }
}

// Convert degrees to steps for a specific joint
long degreesToSteps(float degrees, int joint) {
  return round(degrees / DEG_PER_STEP[joint]);
}

// Convert steps to degrees for a specific joint
float stepsToRadians(long steps, int joint) {
  return steps * DEG_PER_STEP[joint] * PI / 180.0;
}

void enableMotors(bool enable) {
  uint8_t val = enable == true ? LOW : HIGH;
  digitalWrite(EN_PINS[0], val);  // EN321
  digitalWrite(EN_PINS[1], val);  // EN4
  digitalWrite(EN_PINS[2], val);  // EN5
  digitalWrite(EN_PINS[3], val);  // EN6
}

// non-blocking and simultaneous movement
void moveToPosition(float targetPositions[6]) {
  // Clamp each target to joint limits
  float clampedTargets[6];
  for (int i = 0; i < 6; i++) {
    if (targetPositions[i] < lower_limit[i])
      clampedTargets[i] = lower_limit[i];
    else if (targetPositions[i] > upper_limit[i])
      clampedTargets[i] = upper_limit[i];
    else
      clampedTargets[i] = targetPositions[i];
  }

  // Check compound joint constraint (joint 2 + joint 3)
  float sum23 = clampedTargets[1] + clampedTargets[2];
  if (sum23 > 262.0) {
    // Reduce joint 3 to fit the constraint
    clampedTargets[2] -= (sum23 - 262.0);
  }

  for (int i = 0; i < 6; i++) {
    long targetSteps = degreesToSteps(clampedTargets[i], i);
    steppers[i].moveTo(targetSteps);
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
  // Build the data string first
  String data = "f";
  for (int i = 0; i < 6; i++) {
    currentPositions[i] = stepsToRadians(steppers[i].currentPosition(), i);
    data += String(currentPositions[i], 2);
    if (i < 5)
      data += ",";
  }

  // Calculate checksum (simple sum of bytes modulo 256)
  uint8_t checksum = 0;
  for (size_t i = 0; i < data.length(); ++i) {
    checksum += data[i];
  }
  checksum = checksum % 256;

  // Send the data with checksum, separated by '*'
  Serial.print(data);
  Serial.print("*");
  Serial.println(checksum);
}

// void sendCurrentPositions() {
//   Serial.print("f");
//   for (int i = 0; i < 6; i++) {
//     currentPositions[i] = stepsToRadians(steppers[i].currentPosition(), i);
//     Serial.print(currentPositions[i], 2);
//     if (i < 5)
//       Serial.print(",");
//   }
//   Serial.println();
// }

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
    Serial.println("ack");
    // targets are in degrees
    moveToPosition(targets);
  }

  else if (strcmp(cmd, "eOn") == 0) {
    digitalWrite(END_EFFECTOR_PIN, HIGH);
  } else if (strcmp(cmd, "eOff") == 0) {
    digitalWrite(END_EFFECTOR_PIN, LOW);
  } else if (strcmp(cmd, "en") == 0) {
    enableMotors(true);
  } else if (strcmp(cmd, "dis") == 0) {
    enableMotors(false);
  } else if(strcmp(cmd, "home") == 0){
    homeJoint(Z_MAX_PIN, 1);
    Serial.println("ack");
  }
}

int getBackoffDirection(uint8_t joint_num) {
  if (joint_num == 1)  // joint 2
    return 1;          // Back off in +ve direction (ccw)
  if (joint_num == 2)
    return -1;
  return 1;  // Default
}

void homeJoint(uint8_t limitPin, uint8_t joint_num) {
  int dir = getBackoffDirection(joint_num);
  int homeSpeed = 800;
  AccelStepper &stepper = steppers[joint_num];
  // stepper.setMaxSpeed(1000);
  // stepper.setAcceleration(400);  // steps/sec²

  // Step 1: If switch already triggered, back off first
  if (digitalRead(limitPin) == HIGH) {
    // Serial.println("Limit already triggered. Backing off...");
    stepper.setSpeed(homeSpeed * dir);  // Move away from switch    
    // stepper.setAcceleration(600);  // steps/sec²
    unsigned long start = millis();
    while (millis() - start < 1500) {
      stepper.runSpeed();
    }
    delay(100);
  }

  // Step 2: Now move toward switch
  stepper.setSpeed(-homeSpeed / 4 * dir);  // Now move toward switch
  unsigned long start = millis();
  while (digitalRead(limitPin) == LOW && millis() - start < 20000) {
    stepper.runSpeed();
  }

  stepper.stop();                        // Decelerate and stop
  while (stepper.distanceToGo() != 0) {  // Wait for motor to fully stop
    stepper.run();
  }
}
