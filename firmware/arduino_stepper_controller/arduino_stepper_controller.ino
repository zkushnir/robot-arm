/*******************************************************
 * 3-STEPPER ARM CONTROL (UNO-SRAM SAFE) + PYTHON API
 *
 * Commands (case-insensitive):
 *   MOVE a0 a1 a2 v        (deg, deg, deg, min_delay_us)
 *   GET                   -> STATE a0 a1 a2 s0 s1 s2
 *   STOP                  -> disables all motors
 *   SPEEDALL v
 *   ZEROALL
 *   ZERO m
 *   HOMEALL [v]
 *   HOME m [v]
 *   LIMITS m min max
 *   LIMITSALL min0 max0 min1 max1 min2 max2
 *******************************************************/

#include <Arduino.h>
#include <stdlib.h>
#include <math.h>

struct MotorPins {
  int stepPin;
  int dirPin;
  int enPin;
};
/**************** PIN LAYOUT ****************/
// Motor 1
const int STEP1_PIN = 13;
const int DIR1_PIN  = 12;
const int EN1_PIN   = 11;

// Motor 2
const int STEP2_PIN = 10;
const int DIR2_PIN  = 9;
const int EN2_PIN   = 8;

// Motor 3
const int STEP3_PIN = 2;
const int DIR3_PIN  = 3;
const int EN3_PIN   = 4;

static const uint8_t NUM_MOTORS = 3;

/**************** CONFIG (MECHANICS) ****************/
const long STEPS_PER_REV_M[NUM_MOTORS] = {
  16000,   // Motor 1: start with 2x (fix factor-of-2)
  16000,   // Motor 2
  16000    // Motor 3
};

inline float degPerStep(uint8_t i) { return 360.0f / (float)STEPS_PER_REV_M[i]; }

/**************** CONFIG (MOTION PROFILE) ****************/
const int ACCEL_STEPS_DEFAULT = 150;
const int DECEL_STEPS_DEFAULT = 0;

const unsigned int START_DELAY_US_DEFAULT = 650;
const unsigned int MIN_DELAY_US_DEFAULT   = 350;

/**************** INTERNAL STATE ****************/
long stepPos[NUM_MOTORS]         = {0, 0, 0};
float angleOffsetDeg[NUM_MOTORS] = {0.0f, 0.0f, 0.0f};
bool motorEnabled[NUM_MOTORS]    = {false, false, false};

unsigned int startDelayUs[NUM_MOTORS] = {START_DELAY_US_DEFAULT, START_DELAY_US_DEFAULT, START_DELAY_US_DEFAULT};
unsigned int minDelayUs[NUM_MOTORS]   = {MIN_DELAY_US_DEFAULT,   MIN_DELAY_US_DEFAULT,   MIN_DELAY_US_DEFAULT};
int accelSteps[NUM_MOTORS]            = {ACCEL_STEPS_DEFAULT,     ACCEL_STEPS_DEFAULT,     ACCEL_STEPS_DEFAULT};
int decelSteps[NUM_MOTORS]            = {DECEL_STEPS_DEFAULT,     DECEL_STEPS_DEFAULT,     DECEL_STEPS_DEFAULT};

/**************** SAFETY LIMITS ****************/
bool limitsEnabled[NUM_MOTORS] = {true, true, true};
float minLimitDeg[NUM_MOTORS]  = {-180.0f, -180.0f, -180.0f};
float maxLimitDeg[NUM_MOTORS]  = { 180.0f,  180.0f,  180.0f};

/**************** Helpers ****************/

MotorPins getPins(uint8_t i) {
  if (i == 0) return {STEP1_PIN, DIR1_PIN, EN1_PIN};
  if (i == 1) return {STEP2_PIN, DIR2_PIN, EN2_PIN};
  return {STEP3_PIN, DIR3_PIN, EN3_PIN};
}

bool validMotorNumber(int m1) { return (m1 >= 1 && m1 <= 3); }
uint8_t idx0(int m1) { return (uint8_t)(m1 - 1); }

/**************** Motion ramp ****************/
inline unsigned int rampDelayForIndex(int i, int total, int accel, int decel,
                                      unsigned int startDly, unsigned int minDly) {
  if (accel < 1) accel = 1;
  if (decel < 1) decel = 1;
  if (total < 1) total = 1;

  const long span = (long)startDly - (long)minDly;

  if (i < accel) {
    long num = (long)i * span;
    long div = (long)accel;
    unsigned int d = (unsigned int)(startDly - (num / div));
    if (d < minDly) d = minDly;
    return d;
  }
  if (i >= total - decel) {
    int j = (total - 1) - i;
    long num = (long)j * span;
    long div = (long)decel;
    unsigned int d = (unsigned int)(startDly - (num / div));
    if (d < minDly) d = minDly;
    return d;
  }
  return minDly;
}

/**************** Enable / Disable ****************/
void enableMotor(uint8_t i) {
  MotorPins p = getPins(i);
  digitalWrite(p.enPin, LOW);
  motorEnabled[i] = true;
}

void disableMotor(uint8_t i) {
  MotorPins p = getPins(i);
  digitalWrite(p.enPin, HIGH);
  motorEnabled[i] = false;
}

void enableAll()  { for (uint8_t i=0;i<NUM_MOTORS;i++) enableMotor(i); }
void disableAll() { for (uint8_t i=0;i<NUM_MOTORS;i++) disableMotor(i); }

/**************** Low-level stepping ****************/
inline void pulseStepPin(int stepPin, unsigned int dly) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(dly);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(dly);
}

void moveSteps(uint8_t i, long deltaSteps) {
  if (deltaSteps == 0) return;

  if (!motorEnabled[i]) {
    Serial.print(F("ERR motor disabled M"));
    Serial.println(i + 1);
    return;
  }

  MotorPins p = getPins(i);
  bool forward = (deltaSteps > 0);
  long steps = labs(deltaSteps);

  digitalWrite(p.dirPin, forward ? HIGH : LOW);

  unsigned int sD = startDelayUs[i];
  unsigned int mD = minDelayUs[i];
  int aS = accelSteps[i];
  int dS = decelSteps[i];

  if (aS > (int)steps) aS = (int)steps;
  if (dS > (int)steps) dS = (int)steps;

  for (long k = 0; k < steps; k++) {
    unsigned int dly = rampDelayForIndex((int)k, (int)steps, aS, dS, sD, mD);
    pulseStepPin(p.stepPin, dly);
  }

  stepPos[i] += (forward ? steps : -steps);
}

/**************** Angle API ****************/
float getAngleDeg(uint8_t i) {
  return (float)stepPos[i] * degPerStep(i) + angleOffsetDeg[i];
}

void setCurrentAngleDeg(uint8_t i, float realDeg) {
  angleOffsetDeg[i] = realDeg - ((float)stepPos[i] * degPerStep(i));
}

bool withinLimits(uint8_t i, float targetDeg) {
  if (!limitsEnabled[i]) return true;
  return (targetDeg >= minLimitDeg[i] && targetDeg <= maxLimitDeg[i]);
}

void goToAngleDeg(uint8_t i, float targetDeg) {
  if (!withinLimits(i, targetDeg)) {
    Serial.print(F("ERR LIMITS M")); Serial.print(i + 1);
    Serial.print(F(" target=")); Serial.print(targetDeg, 3);
    Serial.print(F(" allowed=[")); Serial.print(minLimitDeg[i], 3);
    Serial.print(F(",")); Serial.print(maxLimitDeg[i], 3);
    Serial.println(F("]"));
    return;
  }

  float desiredStepsF = (targetDeg - angleOffsetDeg[i]) / degPerStep(i);
  long desiredSteps = lroundf(desiredStepsF);
  long delta = desiredSteps - stepPos[i];
  moveSteps(i, delta);
}

/**************** Speed control ****************/
void setMotorSpeedDelay(uint8_t i, unsigned int newMinDelayUs) {
  if (newMinDelayUs < 80)   newMinDelayUs = 80;
  if (newMinDelayUs > 5000) newMinDelayUs = 5000;

  minDelayUs[i] = newMinDelayUs;
  if (startDelayUs[i] < minDelayUs[i]) startDelayUs[i] = minDelayUs[i];
}

void setAllSpeedDelay(unsigned int v) {
  for (uint8_t i=0;i<NUM_MOTORS;i++) setMotorSpeedDelay(i, v);
}

/**************** ZERO / HOME ****************/
void zeroMotor(uint8_t i) { setCurrentAngleDeg(i, 0.0f); }
void zeroAll() { for (uint8_t i=0;i<NUM_MOTORS;i++) zeroMotor(i); }

void homeMotor(uint8_t i) { goToAngleDeg(i, 0.0f); }
void homeAll() { homeMotor(0); homeMotor(1); homeMotor(2); }

/**************** State output ****************/
void printStateLine() {
  Serial.print(F("STATE "));
  Serial.print(getAngleDeg(0), 3); Serial.print(' ');
  Serial.print(getAngleDeg(1), 3); Serial.print(' ');
  Serial.print(getAngleDeg(2), 3); Serial.print(' ');
  Serial.print(stepPos[0]); Serial.print(' ');
  Serial.print(stepPos[1]); Serial.print(' ');
  Serial.print(stepPos[2]);
  Serial.println();
}

/**************** Small command parser (no String) ****************/
static char lineBuf[96];
static uint8_t lineLen = 0;

inline void toLowerInPlace(char *s) {
  for (; *s; ++s) if (*s >= 'A' && *s <= 'Z') *s = *s - 'A' + 'a';
}

int splitTokens(char *buf, char *tok[], int maxTok) {
  int n = 0;
  char *p = strtok(buf, " \t\r\n");
  while (p && n < maxTok) {
    tok[n++] = p;
    p = strtok(NULL, " \t\r\n");
  }
  return n;
}

void handleLine(char *buf) {
  toLowerInPlace(buf);

  char *tok[10];
  int n = splitTokens(buf, tok, 10);
  if (n <= 0) return;

  // Python API + menu unified
  if (!strcmp(tok[0], "move")) {
    if (n < 5) { Serial.println(F("ERR MOVE usage: MOVE a0 a1 a2 v")); return; }
    float a0 = atof(tok[1]);
    float a1 = atof(tok[2]);
    float a2 = atof(tok[3]);
    unsigned int v = (unsigned int)atoi(tok[4]);

    enableAll();
    setAllSpeedDelay(v);

    goToAngleDeg(0, a0);
    goToAngleDeg(1, a1);
    goToAngleDeg(2, a2);

    Serial.println(F("OK"));
    return;
  }

  if (!strcmp(tok[0], "get") && n == 1) {
    printStateLine();
    return;
  }

  if (!strcmp(tok[0], "stop") && n == 1) {
    disableAll();
    Serial.println(F("OK"));
    return;
  }

  if (!strcmp(tok[0], "speedall") && n == 2) {
    setAllSpeedDelay((unsigned int)atoi(tok[1]));
    Serial.println(F("OK"));
    return;
  }

  if (!strcmp(tok[0], "zeroall") && n == 1) {
    zeroAll();
    Serial.println(F("OK"));
    return;
  }

  if (!strcmp(tok[0], "homeall")) {
    if (n == 2) setAllSpeedDelay((unsigned int)atoi(tok[1]));
    enableAll();
    homeAll();
    Serial.println(F("OK"));
    return;
  }

  if (!strcmp(tok[0], "zero") && n == 2) {
    int m = atoi(tok[1]);
    if (!validMotorNumber(m)) { Serial.println(F("ERR motor must be 1..3")); return; }
    zeroMotor(idx0(m));
    Serial.println(F("OK"));
    return;
  }

  if (!strcmp(tok[0], "home")) {
    if (n < 2) { Serial.println(F("ERR usage: home <m> [v]")); return; }
    int m = atoi(tok[1]);
    if (!validMotorNumber(m)) { Serial.println(F("ERR motor must be 1..3")); return; }
    uint8_t i = idx0(m);
    if (n == 3) setMotorSpeedDelay(i, (unsigned int)atoi(tok[2]));
    enableMotor(i);
    homeMotor(i);
    Serial.println(F("OK"));
    return;
  }

  if (!strcmp(tok[0], "limits")) {
    if (n < 4) { Serial.println(F("ERR usage: limits <m> <min_deg> <max_deg>")); return; }
    int m = atoi(tok[1]);
    if (!validMotorNumber(m)) { Serial.println(F("ERR motor must be 1..3")); return; }
    uint8_t i = idx0(m);
    minLimitDeg[i] = atof(tok[2]);
    maxLimitDeg[i] = atof(tok[3]);
    limitsEnabled[i] = true;
    Serial.println(F("OK"));
    return;
  }

  if (!strcmp(tok[0], "limitsall")) {
    if (n < 7) { Serial.println(F("ERR usage: limitsall min0 max0 min1 max1 min2 max2")); return; }
    minLimitDeg[0] = atof(tok[1]); maxLimitDeg[0] = atof(tok[2]);
    minLimitDeg[1] = atof(tok[3]); maxLimitDeg[1] = atof(tok[4]);
    minLimitDeg[2] = atof(tok[5]); maxLimitDeg[2] = atof(tok[6]);
    limitsEnabled[0] = limitsEnabled[1] = limitsEnabled[2] = true;
    Serial.println(F("OK"));
    return;
  }

  // legacy single-motor commands (optional)
  if (!strcmp(tok[0], "set") && n >= 3) {
    int m = atoi(tok[1]);
    float target = atof(tok[2]);
    if (!validMotorNumber(m)) { Serial.println(F("ERR motor must be 1..3")); return; }
    enableMotor(idx0(m));
    goToAngleDeg(idx0(m), target);
    Serial.println(F("OK"));
    return;
  }

  if (!strcmp(tok[0], "status")) {
    printStateLine();
    return;
  }

  Serial.println(F("ERR unknown"));
}

/**************** SETUP / LOOP ****************/
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(STEP1_PIN, OUTPUT); pinMode(DIR1_PIN, OUTPUT); pinMode(EN1_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT); pinMode(DIR2_PIN, OUTPUT); pinMode(EN2_PIN, OUTPUT);
  pinMode(STEP3_PIN, OUTPUT); pinMode(DIR3_PIN, OUTPUT); pinMode(EN3_PIN, OUTPUT);

  disableAll();
  Serial.println(F("3-stepper arm controller ready."));
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else {
      if (lineLen < sizeof(lineBuf) - 1) {
        lineBuf[lineLen++] = c;
      } else {
        // overflow: reset line
        lineLen = 0;
      }
    }
  }
}
