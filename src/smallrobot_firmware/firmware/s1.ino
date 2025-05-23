// 0.00  0.01  0.46  3.14  2.04 -1.57

// #include <math.h>
//#include <SoftwareSerial.h>

//#define PI 3.1415926535897932384626433832795

// this works with ROS2 h/w interface 

// kk-p20/15
#define END_EFFECTOR_PIN 16
//driver for the axis 1
#define PUL1_PIN 39
#define DIR1_PIN 37
//driver for the axis 2
#define PUL2_PIN 43
#define DIR2_PIN 41
//driver for the axis 3
#define PUL3_PIN 47
#define DIR3_PIN 45
//driver for the axis 4
#define PUL4_PIN 46
#define DIR4_PIN 48
//driver for the axis 5
#define PUL5_PIN A6
#define DIR5_PIN A7
//driver for the axis 6
#define PUL6_PIN A0
#define DIR6_PIN A1

//enable pin for the axis 3, 2 and 1
#define EN321_PIN 32
#define EN4_PIN A8
#define EN5_PIN A2
#define EN6_PIN 38

// rest pose (power off) of the robot arm.
float curPos1 = 0.0;
float curPos2 = -78.51;
float curPos3 = 73.90;
float curPos4 = 0.0;
float curPos5 = -90.0;
float curPos6 = 0.0;
const uint8_t dirPins[6] = { DIR1_PIN, DIR2_PIN, DIR3_PIN, DIR4_PIN, DIR5_PIN, DIR6_PIN };
const uint8_t pulPins[6] = { PUL1_PIN, PUL2_PIN, PUL3_PIN, PUL4_PIN, PUL5_PIN, PUL6_PIN };
boolean PULstat1 = 0;
boolean PULstat2 = 0;
boolean PULstat3 = 0;
boolean PULstat4 = 0;
boolean PULstat5 = 0;
boolean PULstat6 = 0;
boolean PULstat[6] = { PULstat1, PULstat2, PULstat3, PULstat4, PULstat5, PULstat6 };
//robot geometry
/*
360.0 / 200.0 / 32.0 calculates the degrees per microstep of the motor itself.
4.8: The division by 4.8 indicates that the motor's output is being further reduced by a gear system.
This means that for every 4.8 rotations of the motor, the output shaft of the gear system rotates once.
So, the 4.8 is a ratio. in essence, it calculates the degrees of rotation of the final output shaft per microstep, considering the gear reduction.
*/
// the motors are 200 steps /rev (50 teeth*4phase), microstepping=32
// step per degree = 200*32/360, dl= degree per microstep.
const double dl1 = 360.0 / 200.0 / 32.0 / 4.8;  // 4.8 is gear ratio
const double dl2 = 360.0 / 200.0 / 32.0 / 4.0;
const double dl3 = 360.0 / 200.0 / 32.0 / 5.0;
const double dl4 = 360.0 / 200.0 / 32.0 / 2.8;
const double dl5 = 360.0 / 200.0 / 32.0 / 2.1;
const double dl6 = 360.0 / 200.0 / 32.0 / 1.0;
const double degreePerStep[6] = { dl1, dl2, dl3, dl4, dl5, dl6 };

// String dataIn = ""; //variable to store the bluetooth command
// int index = 0; //index corresonding to the robot position
// float Joint1[50], Joint2[50], Joint3[50], Joint4[50], Joint5[50], Joint6[50], MaxSpeed[50], InSpeed[50], FinSpeed[50];
const uint8_t ledPin = 13;  // the LED is connected to digital pin 13

float X[6] = { 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 };
float velG = 0;
/*
  int i = 0;
  int j = 0;
  int k = 0;
  float J[4][7];  // col 0: timing
  float V[5][6];  // plus head and tail
  float A[5][6];  // plus head and tail
*/

// float* splitStringToFloatArray(char* str);
const int MAX_INPUT_SIZE = 100;  // Adjust as needed
char inputBuffer[MAX_INPUT_SIZE];
int inputIndex = 0;
bool newCommandReady = false;

float* splitStringToFloatArray(char* str) {
  static float arr[6];
  int count = 0;
  char* token = strtok(str, ",");

  while (token != NULL && count < 6) {
    arr[count++] = atof(token);
    token = strtok(NULL, ",");
  }

  // If there were less than 6 values, fill the rest with 0.0
  while (count < 6) {
    arr[count++] = 0.0;
  }

  return arr;
}
void parseFloats(const char* str, float* out, int maxCount) {
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
      if (decimal) scale *= 10.0f;
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
// #include <TimerOne.h>
volatile int motor1StepsRemaining = 0;
volatile int motor2StepsRemaining = 0;
volatile unsigned long lastStepTime = 0;  // Last time step was made
// if we want a joint rotate 60 degrees per sec, 60 * 60/360 = 10rpm.
const float gearRatio = 4.8;  // gear ratio
const unsigned int rpm = 10;  // rpm
                              // ISR timer interrupt freq to be at least 100 time higher than the pulse rate.
const int stepsPerRevolution = 200;
unsigned int totalEffectiveSteps = 200 * 32 * gearRatio;
unsigned long totalStepsPerMinute = rpm * totalEffectiveSteps;
unsigned int pulse_interval = 1 / (totalStepsPerMinute / 60.0) * 1e6;  // pulse frequency
unsigned int stepInterval = pulse_interval / 3;
unsigned long currentMillis = 0;  // To store current time for non-blocking logic
// Parsed joint commands
float parsedJoints[6];


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect needed for native USB.
  }

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  pinMode(END_EFFECTOR_PIN, OUTPUT);
  digitalWrite(END_EFFECTOR_PIN, LOW);

  pinMode(PUL1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(PUL2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(PUL3_PIN, OUTPUT);
  pinMode(DIR3_PIN, OUTPUT);
  pinMode(PUL4_PIN, OUTPUT);
  pinMode(DIR4_PIN, OUTPUT);
  pinMode(PUL5_PIN, OUTPUT);
  pinMode(DIR5_PIN, OUTPUT);
  pinMode(PUL6_PIN, OUTPUT);
  pinMode(DIR6_PIN, OUTPUT);

  pinMode(EN321_PIN, OUTPUT);
  pinMode(EN4_PIN, OUTPUT);
  pinMode(EN5_PIN, OUTPUT);
  pinMode(EN6_PIN, OUTPUT);

  digitalWrite(PUL1_PIN, LOW);  // gear ratio = 96/20 = 4.8
  digitalWrite(DIR1_PIN, LOW);  //LOW = negative direction

  digitalWrite(PUL2_PIN, LOW);  // gear ratio = 4
  digitalWrite(DIR2_PIN, LOW);  //LOW = positive direction

  digitalWrite(PUL3_PIN, LOW);  // gear ratio = 5
  digitalWrite(DIR3_PIN, LOW);  //LOW = negative direction

  digitalWrite(PUL4_PIN, LOW);  // gear ratio = 56/20 = 2.8
  digitalWrite(DIR4_PIN, LOW);  //LOW = positive direction

  digitalWrite(PUL5_PIN, LOW);  // gear ratio = 42/20 = 2.1
  digitalWrite(DIR5_PIN, LOW);  //LOW = positive direction

  digitalWrite(PUL6_PIN, LOW);  // gear ratio = 1
  digitalWrite(DIR6_PIN, LOW);  //LOW = positive direction

  // all joints disabled!
  digitalWrite(EN321_PIN, HIGH);
  digitalWrite(EN4_PIN, HIGH);
  digitalWrite(EN5_PIN, HIGH);
  digitalWrite(EN6_PIN, HIGH);

  // Set up Timer1 to trigger an interrupt every xxx microseconds
  // Timer1.initialize(stepInterval * 1000);  // 100 times higher than the pulse rate.
  // Timer1.attachInterrupt(stepMotorsISR);
 
}

void loop() {
  serialEvent();  // Always check serial

  if (newCommandReady) {
    Serial.println("a");
    processCommand(inputBuffer);
    newCommandReady = false;
  } else{ 
    // only when nothing to read
    float curPos[6] = { curPos1, curPos2, curPos3, curPos4, curPos5, curPos6 };
    send_cur_positions(curPos, 6);
  }
  // Optional: small delay to avoid overloading
  delay(10);

  // Your stepping logic...
  // if (stepping == false && lastSteppingState == true) {
  //   cli();
  //   Serial.println("ack");
  //   sei();
  // }
  // lastSteppingState = stepping;
}
void send_cur_positions(const float* positions, int num_joints) {
  Serial.print('f');
  for (int i = 0; i < num_joints; ++i) {
    if (positions[i] >= 0) Serial.print('+');
    else Serial.print('-');

    int int_part = int(abs(positions[i]));
    int frac_part = int((abs(positions[i]) - int_part) * 100);

    // Always 3 digits integer, 2 digits decimal
    if (int_part < 100) Serial.print('0');
    if (int_part < 10) Serial.print('0');
    Serial.print(int_part);
    Serial.print('.');
    if (frac_part < 10) Serial.print('0');
    Serial.print(frac_part);
  }
  Serial.println();
}

void processCommand(const char* cmd) {
  if (cmd[0] == 'm' || cmd[0] == 'g') {
    parseFloats(cmd + 1, parsedJoints, 6);

    if (cmd[0] == 'm') {
      goTrajectory(parsedJoints);
      // Serial.println("a");  // send ack back
    } else if (cmd[0] == 'g') {
      // relative posisiton
      float from[6] = { curPos1, curPos2, curPos3, curPos4, curPos5, curPos6 };
      goStrightLine(from, parsedJoints, 0.25e-4, 0.75e-10, 0.0, 0.0);
      // Serial.println("a");
    }
  } else if (strcmp(cmd, "eOn") == 0) {
    digitalWrite(END_EFFECTOR_PIN, HIGH);
  } else if (strcmp(cmd, "eOff") == 0) {
    digitalWrite(END_EFFECTOR_PIN, LOW);
  } else if (strcmp(cmd, "en") == 0) {
    digitalWrite(EN321_PIN, LOW);
    digitalWrite(EN4_PIN, LOW);
    digitalWrite(EN5_PIN, LOW);
    digitalWrite(EN6_PIN, LOW);
  } else if (strcmp(cmd, "dis") == 0) {
    digitalWrite(EN321_PIN, HIGH);
    digitalWrite(EN4_PIN, HIGH);
    digitalWrite(EN5_PIN, HIGH);
    digitalWrite(EN6_PIN, HIGH);
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

void goStrightLine(float* xfi, float* xff, float vel0, float acc0, float velini, float velfin) {
  //
  float lmax = max(abs(xff[0] - xfi[0]), abs(xff[1] - xfi[1]));
  lmax = max(lmax, abs(xff[2] - xfi[2]));
  lmax = max(lmax, abs(xff[3] - xfi[3]));
  lmax = max(lmax, abs(xff[4] - xfi[4]));
  lmax = max(lmax, abs(xff[5] - xfi[5]));
  unsigned long preMil = micros();
  double l = 0.0;
  vel0 = min(vel0, sqrt(lmax * acc0 + 0.5 * velini * velini + 0.5 * velfin * velfin));
  unsigned long curMil = micros();
  unsigned long t = 0;
  double tap = vel0 / acc0 - velini / acc0;
  double lap = velini * tap + acc0 * tap * tap / 2.0;
  double lcsp = lmax - (vel0 * vel0 / 2.0 / acc0 - velfin * velfin / 2.0 / acc0);
  double tcsp = (lcsp - lap) / vel0 + tap;
  double tfin = vel0 / acc0 - velfin / acc0 + tcsp;
  while (curMil - preMil <= tfin) {
    t = curMil - preMil;
    //acceleration phase
    if (t <= tap) {
      l = velini * t + acc0 * t * t / 2.0;
    }
    //contant maximum speed phase
    if (t > tap && t <= tcsp) {
      l = lap + vel0 * (t - tap);
    }
    //deceleration phase
    if (t > tcsp) {
      l = lcsp + vel0 * (t - tcsp) - acc0 * (t - tcsp) * (t - tcsp) / 2.0;
    }

    //trajectory x and y as a function of l
    float Xx[6];
    Xx[0] = xfi[0] + (xff[0] - xfi[0]) / lmax * l;
    Xx[1] = xfi[1] + (xff[1] - xfi[1]) / lmax * l;
    Xx[2] = xfi[2] + (xff[2] - xfi[2]) / lmax * l;
    Xx[3] = xfi[3] + (xff[3] - xfi[3]) / lmax * l;
    Xx[4] = xfi[4] + (xff[4] - xfi[4]) / lmax * l;
    Xx[5] = xfi[5] + (xff[5] - xfi[5]) / lmax * l;

    goTrajectory(Xx);
    curMil = micros();
  }
}

// low level control: this function takes care of gear ratios, we just give degrees for the joints to move from their current positions to the target positions.
void goTrajectory(float Jf[]) {
  const int delF = 2;
  if (Jf[0] - curPos1 > 0.0) {
    // positive direction of rotation - CCW
    digitalWrite(DIR1_PIN, HIGH);
    // why dl /2? coz each puls takes one microstep. there are +/- puls
    while (Jf[0] - curPos1 > dl1 / 2.0) {
      if (PULstat1 == 0) {
        digitalWrite(PUL1_PIN, HIGH);
        PULstat1 = 1;
      } else {
        digitalWrite(PUL1_PIN, LOW);
        PULstat1 = 0;
      }
      //curPos1 = Jf[0];
      curPos1 = curPos1 + dl1 / 2.0;
      if (Jf[0] - curPos1 > dl1 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    // neg direction roatiaon - CW rotation.
    digitalWrite(DIR1_PIN, LOW);
    while (-Jf[0] + curPos1 > dl1 / 2.0) {
      if (PULstat1 == 0) {
        digitalWrite(PUL1_PIN, HIGH);
        PULstat1 = 1;
      } else {
        digitalWrite(PUL1_PIN, LOW);
        PULstat1 = 0;
      }
      //curPos1 = Jf[0];
      curPos1 = curPos1 - dl1 / 2.0;
      if (-Jf[0] + curPos1 > dl1 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #2
  if (Jf[1] - curPos2 > 0.0) {  // positive direction of rotation
    digitalWrite(DIR2_PIN, HIGH);
    while (Jf[1] - curPos2 > dl2 / 2.0) {
      if (PULstat2 == 0) {
        digitalWrite(PUL2_PIN, HIGH);
        PULstat2 = 1;
      } else {
        digitalWrite(PUL2_PIN, LOW);
        PULstat2 = 0;
      }
      //curPos2 = Jf[1];
      curPos2 = curPos2 + dl2 / 2.0;
      if (Jf[1] - curPos2 > dl2 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR2_PIN, LOW);
    while (-Jf[1] + curPos2 > dl2 / 2.0) {
      if (PULstat2 == 0) {
        digitalWrite(PUL2_PIN, HIGH);
        PULstat2 = 1;
      } else {
        digitalWrite(PUL2_PIN, LOW);
        PULstat2 = 0;
      }
      //curPos2 = Jf[1];
      curPos2 = curPos2 - dl2 / 2.0;
      if (-Jf[1] + curPos2 > dl2 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #3
  if (Jf[2] - curPos3 > 0.0) {  // positive direction of rotation
    digitalWrite(DIR3_PIN, LOW);
    while (Jf[2] - curPos3 > dl3 / 2.0) {
      if (PULstat3 == 0) {
        digitalWrite(PUL3_PIN, HIGH);
        PULstat3 = 1;
      } else {
        digitalWrite(PUL3_PIN, LOW);
        PULstat3 = 0;
      }
      //curPos3 = Jf[2];
      curPos3 = curPos3 + dl3 / 2.0;
      if (Jf[2] - curPos3 > dl3 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR3_PIN, HIGH);
    while (-Jf[2] + curPos3 > dl3 / 2.0) {
      if (PULstat3 == 0) {
        digitalWrite(PUL3_PIN, HIGH);
        PULstat3 = 1;
      } else {
        digitalWrite(PUL3_PIN, LOW);
        PULstat3 = 0;
      }
      //curPos3 = Jf[2];
      curPos3 = curPos3 - dl3 / 2.0;
      if (-Jf[2] + curPos3 > dl3 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #4
  if (Jf[3] - curPos4 > 0.0) {  // positive direction of rotation
    digitalWrite(DIR4_PIN, HIGH);
    while (Jf[3] - curPos4 > dl4 / 2.0) {
      if (PULstat4 == 0) {
        digitalWrite(PUL4_PIN, HIGH);
        PULstat4 = 1;
      } else {
        digitalWrite(PUL4_PIN, LOW);
        PULstat4 = 0;
      }
      //curPos4 = Jf[3];
      curPos4 = curPos4 + dl4 / 2.0;
      if (Jf[3] - curPos4 > dl4 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR4_PIN, LOW);
    while (-Jf[3] + curPos4 > dl4 / 2.0) {
      if (PULstat4 == 0) {
        digitalWrite(PUL4_PIN, HIGH);
        PULstat4 = 1;
      } else {
        digitalWrite(PUL4_PIN, LOW);
        PULstat4 = 0;
      }
      //curPos4 = Jf[3];
      curPos4 = curPos4 - dl4 / 2.0;
      if (-Jf[3] + curPos4 > dl4 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #5
  if (Jf[4] - curPos5 > 0.0) {  // positive direction of rotation
    digitalWrite(DIR5_PIN, HIGH);
    while (Jf[4] - curPos5 > dl5 / 2.0) {
      if (PULstat5 == 0) {
        digitalWrite(PUL5_PIN, HIGH);
        PULstat5 = 1;
      } else {
        digitalWrite(PUL5_PIN, LOW);
        PULstat5 = 0;
      }
      //curPos5 = Jf[4];
      curPos5 = curPos5 + dl5 / 2.0;
      if (Jf[4] - curPos5 > dl5 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR5_PIN, LOW);
    while (-Jf[4] + curPos5 > dl5 / 2.0) {
      if (PULstat5 == 0) {
        digitalWrite(PUL5_PIN, HIGH);
        PULstat5 = 1;
      } else {
        digitalWrite(PUL5_PIN, LOW);
        PULstat5 = 0;
      }
      //curPos5 = Jf[4];
      curPos5 = curPos5 - dl5 / 2.0;
      if (-Jf[4] + curPos5 > dl5 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  }
  // joint #6
  if (Jf[5] - curPos6 > 0.0) {  // positive direction of rotation
    digitalWrite(DIR6_PIN, HIGH);
    while (Jf[5] - curPos6 > dl6 / 2.0) {
      if (PULstat6 == 0) {
        digitalWrite(PUL6_PIN, HIGH);
        PULstat6 = 1;
      } else {
        digitalWrite(PUL6_PIN, LOW);
        PULstat6 = 0;
      }
      //curPos6 = Jf[5];
      curPos6 = curPos6 + dl6 / 2.0;
      if (Jf[5] - curPos6 > dl6 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  } else {
    digitalWrite(DIR6_PIN, LOW);
    while (-Jf[5] + curPos6 > dl6 / 2.0) {
      if (PULstat6 == 0) {
        digitalWrite(PUL6_PIN, HIGH);
        PULstat6 = 1;
      } else {
        digitalWrite(PUL6_PIN, LOW);
        PULstat6 = 0;
      }
      //curPos6 = Jf[5];
      curPos6 = curPos6 - dl6 / 2.0;
      if (-Jf[5] + curPos6 > dl6 / 2.0) {
        delayMicroseconds(delF);
      }
    }
  }
}
/*
void goTrajectory1(float Jf[]) {
  const int baseDelay = 400;  // Start delay (slower = bigger value)
  const int minDelay = 100;   // Min delay (fastest speed)
  const int accelSteps = 200; // Number of steps to reach full speed
  const int delF = 2;

  bool moving[6] = {true, true, true, true, true, true};
  float targets[6] = {Jf[0], Jf[1], Jf[2], Jf[3], Jf[4], Jf[5]};
  float* curPos[6] = {&curPos1, &curPos2, &curPos3, &curPos4, &curPos5, &curPos6};
  int DIR_PIN[6] = {DIR1_PIN, DIR2_PIN, DIR3_PIN, DIR4_PIN, DIR5_PIN, DIR6_PIN};
  int PUL_PIN[6] = {PUL1_PIN, PUL2_PIN, PUL3_PIN, PUL4_PIN, PUL5_PIN, PUL6_PIN};
  int PULstat[6] = {&PULstat1, &PULstat2, &PULstat3, &PULstat4, &PULstat5, &PULstat6};
  float dl[6] = {dl1, dl2, dl3, dl4, dl5, dl6};

  int stepCount[6] = {0, 0, 0, 0, 0, 0};  // count steps per joint
  int totalSteps[6];

  // Compute how many steps each joint needs
  for (int i = 0; i < 6; i++) {
    totalSteps[i] = abs(int((targets[i] - *(curPos[i])) / (dl[i] / 2.0)));
    if (targets[i] - *(curPos[i]) > 0.0) {
      digitalWrite(DIR_PIN[i], HIGH);
    } else {
      digitalWrite(DIR_PIN[i], LOW);
    }
  }

  bool allDone = false;
  while (!allDone) {
    allDone = true;

    for (int i = 0; i < 6; i++) {
      if (!moving[i]) continue;

      float diff = targets[i] - *(curPos[i]);
      if (abs(diff) > dl[i] / 2.0) {
        // Calculate acceleration profile
        int halfSteps = totalSteps[i] / 2;
        int currentStep = stepCount[i];
        int delayNow = baseDelay;

        if (currentStep < accelSteps) {
          // Accelerating
          delayNow = baseDelay - (baseDelay - minDelay) * currentStep / accelSteps;
        } else if (currentStep > totalSteps[i] - accelSteps) {
          // Decelerating
          int decelStep = totalSteps[i] - currentStep;
          delayNow = baseDelay - (baseDelay - minDelay) * decelStep / accelSteps;
        } else {
          delayNow = minDelay;
        }

        if (*(PULstat[i]) == 0) {
          digitalWrite(PUL_PIN[i], HIGH);
          *(PULstat[i]) = 1;
        } else {
          digitalWrite(PUL_PIN[i], LOW);
          *(PULstat[i]) = 0;
          if (diff > 0) {
            *(curPos[i]) += dl[i] / 2.0;
          } else {
            *(curPos[i]) -= dl[i] / 2.0;
          }
          stepCount[i]++;
        }

        delayMicroseconds(delayNow);
        allDone = false;
      } else {
        moving[i] = false;
      }
    }
    delayMicroseconds(delF);
  }
}
*/
