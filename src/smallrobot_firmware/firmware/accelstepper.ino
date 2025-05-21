#include <AccelStepper.h>
#include <cmath>
#include "define.h"

// Stepper motor objects
AccelStepper steppers[6] = {
    AccelStepper(AccelStepper::DRIVER, PUL_PINS[0], DIR_PINS[0]),
    AccelStepper(AccelStepper::DRIVER, PUL_PINS[1], DIR_PINS[1]),
    AccelStepper(AccelStepper::DRIVER, PUL_PINS[2], DIR_PINS[2]),
    AccelStepper(AccelStepper::DRIVER, PUL_PINS[3], DIR_PINS[3]),
    AccelStepper(AccelStepper::DRIVER, PUL_PINS[4], DIR_PINS[4]),
    AccelStepper(AccelStepper::DRIVER, PUL_PINS[5], DIR_PINS[5])};

// Current positions in degrees
float currentPositions[6] = {0.0, -78.51, 73.90, 0.0, -90.0, 0.0};

char inputBuffer[MAX_INPUT_SIZE];
int inputIndex = 0;
bool newCommandReady = false;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect needed for native USB.
  }
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  // this is essential for the stepper drivers to work
  pinMode(EN_PINS[0], OUTPUT); // EN321
  pinMode(EN_PINS[1], OUTPUT); // EN4
  pinMode(EN_PINS[2], OUTPUT); // EN5
  pinMode(EN_PINS[3], OUTPUT); // EN6

  // Configure steppers
  for (int i = 0; i < 6; i++)
  {
    steppers[i].setMaxSpeed(2000);     // steps/sec (adjust based on your requirements)
    steppers[i].setAcceleration(1000); // steps/sec²
  }
  steppers[2].setPinsInverted(true, false, false); // Invert direction pin for joint3
  // Set initial positions
  for (int i = 0; i < 6; i++)
  {
    steppers[i].setCurrentPosition(degreesToSteps(currentPositions[i], i));
  }
  disableMotors();
}

void loop()
{
  // Handle serial communication
  serialEvent();
  if (newCommandReady)
  {
    processCommand(inputBuffer);
    newCommandReady = false;
  }
  // Periodically send current positions (10hz)
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 100)
  { // 10Hz update
    sendCurrentPositions();
    lastSend = millis();
  }
  // Non-blocking motor control
  for (int i = 0; i < 6; i++)
  {
    steppers[i].run();
  }
}

// Convert degrees to steps for a specific joint
long degreesToSteps(float degrees, int joint)
{
  return round(degrees / DEG_PER_STEP[joint]);
}

// Convert steps to degrees for a specific joint
float stepsToRadians(long steps, int joint)
{
  return steps * DEG_PER_STEP[joint] * M_PI / 180.0;
}

void enableMotors()
{
  digitalWrite(EN_PINS[0], LOW); // EN321
  digitalWrite(EN_PINS[1], LOW); // EN4
  digitalWrite(EN_PINS[2], LOW); // EN5
  digitalWrite(EN_PINS[3], LOW); // EN6
}

void disableMotors()
{
  digitalWrite(EN_PINS[0], HIGH); // EN321
  digitalWrite(EN_PINS[1], HIGH); // EN4
  digitalWrite(EN_PINS[2], HIGH); // EN5
  digitalWrite(EN_PINS[3], HIGH); // EN6
}

void moveToPosition(float targetPositions[6])
{
  //   enableMotors();
  for (int i = 0; i < 6; i++)
  {
    long targetSteps = degreesToSteps(targetPositions[i], i);
    steppers[i].moveTo(targetSteps);
  }
}

void serialEvent()
{
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n')
    {
      inputBuffer[inputIndex] = '\0';
      inputIndex = 0;
      newCommandReady = true;
      return;
    }
    else if (inputIndex < MAX_INPUT_SIZE - 1)
    {
      inputBuffer[inputIndex++] = c;
    }
  }
}

void sendCurrentPositions()
{
  // Build the data string first
  String data = "f";
  for (int i = 0; i < 6; i++)
  {
    currentPositions[i] = stepsToRadians(steppers[i].currentPosition(), i);
    data += String(currentPositions[i], 2);
    if (i < 5)
      data += ",";
  }

  // Calculate checksum (simple sum of bytes modulo 256)
  uint8_t checksum = 0;
  for (size_t i = 0; i < data.length(); ++i)
  {
    checksum += data[i];
  }
  checksum = checksum % 256;

  // Send the data with checksum, separated by '*'
  Serial.print(data);
  Serial.print("*");
  Serial.println(checksum);
}

// This function parses a string of comma-separated floats
void parseFloats(const char *str, float *out, int maxCount)
{
  int idx = 0;
  bool negative = false;
  float value = 0.0f;
  float scale = 1.0f;
  bool decimal = false;

  while (*str && idx < maxCount)
  {
    char c = *str++;
    if (c == '-')
    {
      negative = true;
    }
    else if (c >= '0' && c <= '9')
    {
      value = value * 10.0f + (c - '0');
      if (decimal)
        scale *= 10.0f;
    }
    else if (c == '.')
    {
      decimal = true;
    }
    else if (c == ',')
    {
      out[idx++] = (negative ? -1 : 1) * (value / scale);
      value = 0.0f;
      scale = 1.0f;
      negative = false;
      decimal = false;
    }
  }
  if (idx < maxCount)
  {
    out[idx++] = (negative ? -1 : 1) * (value / scale);
  }
  while (idx < maxCount)
  {
    out[idx++] = 0.0f;
  }
}

// Keep your existing serialEvent() and parsing functions
// Add command processing in processCommand():
void processCommand(const char *cmd)
{
  if (cmd[0] == 'g')
  {
    float targets[6];
    parseFloats(cmd + 1, targets, 6);
    Serial.println("a");
    // targets are in degrees
    moveToPosition(targets);
  }

  else if (strcmp(cmd, "eOn") == 0)
  {
    digitalWrite(END_EFFECTOR_PIN, HIGH);
  }
  else if (strcmp(cmd, "eOff") == 0)
  {
    digitalWrite(END_EFFECTOR_PIN, LOW);
  }
  else if (strcmp(cmd, "en") == 0)
  {
    enableMotors();
  }
  else if (strcmp(cmd, "dis") == 0)
  {
    disableMotors();
  }
}