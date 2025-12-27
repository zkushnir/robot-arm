#include <Dynamixel2Arduino.h>
#include <actuator.h>

/*******************************************************
 * 3-STEPPER ARM CONTROL (SERIAL MENU) + PYTHON API
 * - Serial-only control (no buttons, no toggle switch, no encoder)
 * - Adds Motor 3
 * - Keeps the same step/dir/en method of control
 * - Keeps speed control EXACTLY the same way (min_delay_us)
 *
 * Added Python API commands:
 *   MOVE a0 a1 a2 v   (angles in deg, v = min_delay_us)
 *   GET              (prints: STATE a0 a1 a2)
 *   STOP             (disables all motors)
 *   SPEEDALL v       (sets min_delay_us for all motors)
 *******************************************************/

/**************** PIN LAYOUT ****************/
// Motor 1
const int STEP1_PIN = 5;
const int DIR1_PIN  = 12;
const int EN1_PIN   = 8;

// Motor 2
const int STEP2_PIN = 9;
const int DIR2_PIN  = 10;
const int EN2_PIN   = 11;

// Motor 3
const int STEP3_PIN = 2;
const int DIR3_PIN  = 3;
const int EN3_PIN   = 4;

/**************** CONFIG (MECHANICS) ****************/
const long STEPS_PER_REV = 16000;  // adjust for your hardware
const float DEG_PER_STEP = 360.0f / (float)STEPS_PER_REV;

/**************** CONFIG (MOTION PROFILE) ****************/
const int ACCEL_STEPS_DEFAULT = 150;
const int DECEL_STEPS_DEFAULT = 0;

const unsigned int START_DELAY_US_DEFAULT = 650;
const unsigned int MIN_DELAY_US_DEFAULT   = 350;

/**************** INTERNAL STATE ****************/
static const uint8_t NUM_MOTORS = 3;

long stepPos[NUM_MOTORS] = {0, 0, 0};
float angleOffsetDeg[NUM_MOTORS] = {0.0f, 0.0f, 0.0f};
bool motorEnabled[NUM_MOTORS] = {false, false, false};

unsigned int startDelayUs[NUM_MOTORS] = {START_DELAY_US_DEFAULT, START_DELAY_US_DEFAULT, START_DELAY_US_DEFAULT};
unsigned int minDelayUs[NUM_MOTORS]   = {MIN_DELAY_US_DEFAULT,   MIN_DELAY_US_DEFAULT,   MIN_DELAY_US_DEFAULT};
int accelSteps[NUM_MOTORS]            = {ACCEL_STEPS_DEFAULT,     ACCEL_STEPS_DEFAULT,     ACCEL_STEPS_DEFAULT};
int decelSteps[NUM_MOTORS]            = {DECEL_STEPS_DEFAULT,     DECEL_STEPS_DEFAULT,     DECEL_STEPS_DEFAULT};

/**************** HELPERS: motor pin lookup ****************/
struct MotorPins {
  int stepPin;
  int dirPin;
  int enPin;
};

MotorPins getPins(uint8_t motorIdx0) {
  switch (motorIdx0) {
    case 0: return {STEP1_PIN, DIR1_PIN, EN1_PIN};
    case 1: return {STEP2_PIN, DIR2_PIN, EN2_PIN};
    default:return {STEP3_PIN, DIR3_PIN, EN3_PIN};
  }
}

bool validMotorNumber(int m1) { return (m1 >= 1 && m1 <= 3); }
uint8_t idx0(int m1) { return (uint8_t)(m1 - 1); }

/**************** INTERNAL: compute delay for trapezoid ****************/
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

/**************** ENABLE / DISABLE ****************/
void enableMotor(uint8_t motorIdx0) {
  MotorPins p = getPins(motorIdx0);
  digitalWrite(p.enPin, LOW);
  motorEnabled[motorIdx0] = true;
}

void disableMotor(uint8_t motorIdx0) {
  MotorPins p = getPins(motorIdx0);
  digitalWrite(p.enPin, HIGH);
  motorEnabled[motorIdx0] = false;
}

void enableAll()  { for (uint8_t i=0;i<NUM_MOTORS;i++) enableMotor(i); }
void disableAll() { for (uint8_t i=0;i<NUM_MOTORS;i++) disableMotor(i); }

/**************** LOW-LEVEL STEPPING ****************/
inline void pulseStepPin(int stepPin, unsigned int dly) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(dly);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(dly);
}

void moveSteps(uint8_t motorIdx0, long deltaSteps) {
  if (deltaSteps == 0) return;

  if (!motorEnabled[motorIdx0]) {
    Serial.print("Motor "); Serial.print(motorIdx0 + 1);
    Serial.println(" is disabled. Use: on <m>  (or on all)");
    return;
  }

  MotorPins p = getPins(motorIdx0);

  bool forward = (deltaSteps > 0);
  long steps = labs(deltaSteps);

  digitalWrite(p.dirPin, forward ? HIGH : LOW);

  unsigned int sD = startDelayUs[motorIdx0];
  unsigned int mD = minDelayUs[motorIdx0];
  int aS = accelSteps[motorIdx0];
  int dS = decelSteps[motorIdx0];

  if (aS > (int)steps) aS = (int)steps;
  if (dS > (int)steps) dS = (int)steps;

  for (long i = 0; i < steps; i++) {
    unsigned int dly = rampDelayForIndex((int)i, (int)steps, aS, dS, sD, mD);
    pulseStepPin(p.stepPin, dly);
  }

  stepPos[motorIdx0] += (forward ? steps : -steps);
}

/**************** ANGLE API ****************/
float getAngleDeg(uint8_t motorIdx0) {
  return (float)stepPos[motorIdx0] * DEG_PER_STEP + angleOffsetDeg[motorIdx0];
}

void setCurrentAngleDeg(uint8_t motorIdx0, float realAngleDeg) {
  angleOffsetDeg[motorIdx0] = realAngleDeg - ((float)stepPos[motorIdx0] * DEG_PER_STEP);
}

void goToAngleDeg(uint8_t motorIdx0, float targetAngleDeg) {
  float desiredStepsF = (targetAngleDeg - angleOffsetDeg[motorIdx0]) / DEG_PER_STEP;
  long desiredSteps = lroundf(desiredStepsF);
  long delta = desiredSteps - stepPos[motorIdx0];

  Serial.print("Motor "); Serial.print(motorIdx0 + 1);
  Serial.print(" target "); Serial.print(targetAngleDeg, 3);
  Serial.print(" deg -> move "); Serial.print(delta);
  Serial.println(" steps");

  moveSteps(motorIdx0, delta);
}

/**************** SPEED CONTROL ****************/
void setMotorSpeedDelay(uint8_t motorIdx0, unsigned int newMinDelayUs) {
  if (newMinDelayUs < 80)  newMinDelayUs = 80;
  if (newMinDelayUs > 5000) newMinDelayUs = 5000;

  minDelayUs[motorIdx0] = newMinDelayUs;

  if (startDelayUs[motorIdx0] < minDelayUs[motorIdx0]) {
    startDelayUs[motorIdx0] = minDelayUs[motorIdx0];
  }
}

void setAllSpeedDelay(unsigned int newMinDelayUs) {
  for (uint8_t i=0;i<NUM_MOTORS;i++) setMotorSpeedDelay(i, newMinDelayUs);
}

/**************** SERIAL MENU ****************/
void printMenu() {
  Serial.println();
  Serial.println("=== ARM SERIAL MENU ===");
  Serial.println("help");
  Serial.println("on all | on <m>");
  Serial.println("off all | off <m>");
  Serial.println("set <m> <angle_deg>          -> move motor m to angle");
  Serial.println("get <m>                      -> print motor m angle");
  Serial.println("set_current <m> <real_deg>   -> calibrate: set current reported angle = real_deg");
  Serial.println("speed <m> <min_delay_us>     -> change motor speed (lower = faster)");
  Serial.println("speedall <min_delay_us>      -> set speed for all motors");
  Serial.println("status                       -> print enable/speed/angles");
  Serial.println();
  Serial.println("Python API:");
  Serial.println("MOVE a0 a1 a2 v              -> move all motors (v=min_delay_us)");
  Serial.println("GET                          -> prints: STATE a0 a1 a2");
  Serial.println("STOP                         -> disables all motors");
  Serial.println("=======================");
  Serial.println();
}

void printStatus() {
  Serial.println();
  Serial.println("--- STATUS ---");
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    Serial.print("M"); Serial.print(i + 1);
    Serial.print(" | enabled: "); Serial.print(motorEnabled[i] ? "YES" : "NO");
    Serial.print(" | angle: "); Serial.print(getAngleDeg(i), 3); Serial.print(" deg");
    Serial.print(" | stepPos: "); Serial.print(stepPos[i]);
    Serial.print(" | minDelayUs: "); Serial.print(minDelayUs[i]);
    Serial.print(" | startDelayUs: "); Serial.print(startDelayUs[i]);
    Serial.println();
  }
  Serial.println("-------------");
  Serial.println();
}

int tokenize(const String& line, String tokens[], int maxTok) {
  int n = 0;
  int start = 0;
  while (start < (int)line.length() && n < maxTok) {
    while (start < (int)line.length() && line[start] == ' ') start++;
    if (start >= (int)line.length()) break;

    int end = line.indexOf(' ', start);
    if (end == -1) end = line.length();

    tokens[n++] = line.substring(start, end);
    start = end + 1;
  }
  return n;
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  // NOTE: We intentionally do NOT use sscanf. We parse with tokenize().
  String lower = line;
  lower.toLowerCase();

  String t[8];
  int n = tokenize(lower, t, 8);

  // === PYTHON API ===
  if (t[0] == "move") {
    // MOVE a0 a1 a2 v
    // v = min_delay_us (lower=faster)
    if (n < 5) {
      Serial.println("ERR MOVE usage: MOVE a0 a1 a2 v");
      return;
    }

    float a0 = t[1].toFloat();
    float a1 = t[2].toFloat();
    float a2 = t[3].toFloat();
    unsigned int v = (unsigned int)t[4].toInt();

    enableAll();
    setAllSpeedDelay(v);

    goToAngleDeg(0, a0);
    goToAngleDeg(1, a1);
    goToAngleDeg(2, a2);

    Serial.println("OK");
    return;
  }

  if (t[0] == "get" && n == 1) {
    // GET -> STATE a0 a1 a2
    Serial.print("STATE ");
    Serial.print(getAngleDeg(0), 3); Serial.print(" ");
    Serial.print(getAngleDeg(1), 3); Serial.print(" ");
    Serial.print(getAngleDeg(2), 3);
    Serial.println();
    return;
  }

  if (t[0] == "stop" && n == 1) {
    disableAll();
    Serial.println("OK");
    return;
  }

  // === existing menu ===
  // Use the LOWERCASED token list for comparisons, but preserve your original behavior otherwise.

  if (t[0] == "help") { printMenu(); return; }
  if (t[0] == "status") { printStatus(); return; }

  if (t[0] == "on") {
    if (n < 2) { Serial.println("Usage: on all | on <m>"); return; }
    if (t[1] == "all") { enableAll(); Serial.println("All motors enabled."); }
    else {
      int m = t[1].toInt();
      if (!validMotorNumber(m)) { Serial.println("Motor must be 1..3"); return; }
      enableMotor(idx0(m));
      Serial.print("Motor "); Serial.print(m); Serial.println(" enabled.");
    }
    return;
  }

  if (t[0] == "off") {
    if (n < 2) { Serial.println("Usage: off all | off <m>"); return; }
    if (t[1] == "all") { disableAll(); Serial.println("All motors disabled."); }
    else {
      int m = t[1].toInt();
      if (!validMotorNumber(m)) { Serial.println("Motor must be 1..3"); return; }
      disableMotor(idx0(m));
      Serial.print("Motor "); Serial.print(m); Serial.println(" disabled.");
    }
    return;
  }

  if (t[0] == "get") {
    if (n < 2) { Serial.println("Usage: get <m>"); return; }
    int m = t[1].toInt();
    if (!validMotorNumber(m)) { Serial.println("Motor must be 1..3"); return; }
    uint8_t i = idx0(m);
    Serial.print("Motor "); Serial.print(m);
    Serial.print(" angle = "); Serial.print(getAngleDeg(i), 3);
    Serial.println(" deg");
    return;
  }

  if (t[0] == "set_current") {
    if (n < 3) { Serial.println("Usage: set_current <m> <real_deg>"); return; }
    int m = t[1].toInt();
    if (!validMotorNumber(m)) { Serial.println("Motor must be 1..3"); return; }
    float realDeg = t[2].toFloat();
    setCurrentAngleDeg(idx0(m), realDeg);
    Serial.print("Motor "); Serial.print(m);
    Serial.print(" calibrated. Now angle = ");
    Serial.print(getAngleDeg(idx0(m)), 3);
    Serial.println(" deg");
    return;
  }

  if (t[0] == "set") {
    if (n < 3) { Serial.println("Usage: set <m> <angle_deg>"); return; }
    int m = t[1].toInt();
    if (!validMotorNumber(m)) { Serial.println("Motor must be 1..3"); return; }
    float targetDeg = t[2].toFloat();
    goToAngleDeg(idx0(m), targetDeg);
    Serial.print("Done. Motor "); Serial.print(m);
    Serial.print(" angle now = "); Serial.print(getAngleDeg(idx0(m)), 3);
    Serial.println(" deg");
    return;
  }

  if (t[0] == "speed") {
    if (n < 3) { Serial.println("Usage: speed <m> <min_delay_us>"); return; }
    int m = t[1].toInt();
    if (!validMotorNumber(m)) { Serial.println("Motor must be 1..3"); return; }
    unsigned int d = (unsigned int)t[2].toInt();
    setMotorSpeedDelay(idx0(m), d);
    Serial.print("Motor "); Serial.print(m);
    Serial.print(" minDelayUs set to "); Serial.println(minDelayUs[idx0(m)]);
    return;
  }

  if (t[0] == "speedall") {
    if (n < 2) { Serial.println("Usage: speedall <min_delay_us>"); return; }
    unsigned int d = (unsigned int)t[1].toInt();
    setAllSpeedDelay(d);
    Serial.print("All motors minDelayUs set to "); Serial.println(d);
    return;
  }

  Serial.println("Unknown command. Type: help");
}

/**************** SETUP ****************/
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(STEP1_PIN, OUTPUT); pinMode(DIR1_PIN, OUTPUT); pinMode(EN1_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT); pinMode(DIR2_PIN, OUTPUT); pinMode(EN2_PIN, OUTPUT);
  pinMode(STEP3_PIN, OUTPUT); pinMode(DIR3_PIN, OUTPUT); pinMode(EN3_PIN, OUTPUT);

  disableAll();

  Serial.println("3-stepper arm controller ready.");
  Serial.println("Type 'help' to see commands.");
  printMenu();
}

/**************** LOOP ****************/
void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }
}
