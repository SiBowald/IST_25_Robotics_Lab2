// Manual
// 0. Make sure the cart is in the middle
// 1. Go to the first pose with your hand
// 2. Press one of the end switches
// 3. Release the switch and wait approximately 1s in the same pose  -> Vector is saved as A
// 4. Go to the second pose
// 5. Press one of the end switches
// 6. Release the switch and wait approximately 1s in the same pose  -> Vector is saved as B
//    Angle between A and B is calculated. A = 0 / B = 1
// 7. Let the motor do the initalisation
// 8. After finishing the initalisation you should hear a sound
// 9. Now you can move your arm between the two poses and the cart should move in the same way.
//    You can also move into a other direction up to the previously calculated angle A/B

#include <Wire.h>
#include "ICM20600.h"

// pins
#define sw1Pin 10
#define sw2Pin 11

// stepper pins
#define stepPin 8
#define dirPin  9

#define DIR_TO_SW1  (+1)
#define DIR_TO_SW2  (-1)

// settings
#define CAL_SAMPLES 30
#define CAL_DELAY_MS 5
#define IDLE_TIME_MS 500

// moving average window size
#define MA_N 100

// stepper speed and timing
#define STEP_PERIOD_US 2000         // time between steps (smaller = faster)
#define STEP_PULSE_US  10           // HIGH pulse width on STEP pin

#define STEP_PERIOD_MAX_US 10000    // slowest (start speed)
#define STEP_PERIOD_MIN_US 2000     // fastest (top speed)
#define ACCEL_MULT_PERMILLE 985     // reduce the time between steps
#define DECEL_MULT_PERMILLE 1015    // increase the time between steps
#define DECEL_WINDOW_STEPS 100      // start slowing down when close to target

// stop jitter around target
#define MOTOR_DEADBAND_STEPS 15
// soft limits so motor does not press endstops during normal operation
#define ENDSTOP_MARGIN_STEPS 100
// ignore target changes smaller than this
#define TARGET_HYST_STEPS 15

// global variables
ICM20600 imu(true);

// acceleration from the sensors
int16_t ax, ay, az;

// Pose vectors
float poseA[3] = {0, 0, 0};
float poseB[3] = {0, 0, 0};
float angleAB = 0.0;  // radians between poseA and poseB

// Moving average buffer
float posBuf[MA_N];
float posSum = 0.0f;
int posIdx = 0;
int posCount = 0;

// motor state
long motorPosSteps = 0;   // current motor position in steps
long rangeSteps    = 0;   // total travel sw1 -> sw2 (in steps)
long targetSteps   = 0;   // desired motor position in steps (from pos 0..100)

// scheduler for non-blocking stepping -> the next step is allowed at this time
unsigned long nextStepUs = 0;

unsigned long stepPeriodUs = STEP_PERIOD_MAX_US; // current period between steps
int lastDirSign = 0;                              // +1 / -1 of previous step


// helper functions:

// returns 1 when no button pressed, 0 when any button pressed
int buttonsReleased() {
  return (digitalRead(sw1Pin) == 1) && (digitalRead(sw2Pin) == 1);
}

int buttonPressed() {
  return !buttonsReleased();
}

// Keep the value between min and max value
float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// calculates the dot product of two 3-D vectors
float dot3(float a[3], float b[3]) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// turns a 3-D vector into a unit vector (length = 1)
void normalize3(float v[3]) {
  float m = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  if (m < 1e-4) return; //prevents from crashing if zero
  v[0] /= m;
  v[1] /= m;
  v[2] /= m;
}

// reset the moving average
void resetPosMovingAverage() {
  posSum = 0.0f;
  posIdx = 0;
  posCount = 0;
  for (int i = 0; i < MA_N; i++) posBuf[i] = 0.0f;
}

// keeps a moving average (a rolling average) of the last MA_N values
float pushPosMovingAverage(float v) {
  if (posCount < MA_N) {
    posBuf[posIdx] = v;
    posSum += v;
    posCount++;
  } else {
    posSum -= posBuf[posIdx];
    posBuf[posIdx] = v;
    posSum += v;
  }
  posIdx = (posIdx + 1) % MA_N;
  return posSum / (float)posCount;
}

// read the sensors
void readAccel() {
  ax = imu.getAccelerationX();
  ay = imu.getAccelerationY();
  az = imu.getAccelerationZ();
}

// capture a pose = average accel direction over CAL_SAMPLES samples
void capturePose(float poseOut[3]) {
  float sum[3] = {0, 0, 0};

  for (int i = 0; i < CAL_SAMPLES; i++) {
    readAccel();
    sum[0] += (float)ax;
    sum[1] += (float)ay;
    sum[2] += (float)az;
    delay(CAL_DELAY_MS);
  }

  sum[0] /= (float)CAL_SAMPLES;
  sum[1] /= (float)CAL_SAMPLES;
  sum[2] /= (float)CAL_SAMPLES;

  normalize3(sum);

  poseOut[0] = sum[0];
  poseOut[1] = sum[1];
  poseOut[2] = sum[2];
}

// executes exactly ONE step, direction depends on dirSign (+1 or -1)
void doOneStep(int dirSign) {
  if (dirSign > 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(stepPin, LOW);
}

void homeAndMeasureRange() {
  Serial.println("Homing to sw1...");

  // move until sw1 is pressed (LOW)
  while (digitalRead(sw1Pin) == HIGH) {
    doOneStep(DIR_TO_SW1);
    delayMicroseconds(STEP_PERIOD_US);
  }

  // define "0" at the sw1 hit point for now
  motorPosSteps = 0;

  // back off until sw1 releases (or max steps)
  for (int i = 0; i < ENDSTOP_MARGIN_STEPS; i++) {
    if (digitalRead(sw1Pin) == HIGH) break;
    doOneStep(DIR_TO_SW2);
    delayMicroseconds(STEP_PERIOD_US);
    motorPosSteps++; // moved away from sw1
  }

  long backoff1 = motorPosSteps;   // remember how far we backed off

  Serial.println("Measuring travel to sw2...");
  long count = 0;

  // move until sw2 pressed (LOW)
  while (digitalRead(sw2Pin) == HIGH) {
    doOneStep(DIR_TO_SW2);
    delayMicroseconds(STEP_PERIOD_US);
    count++;
  }

  // count is hit-to-hit distance from sw1-hit to sw2-hit
  rangeSteps = backoff1 + count;

  motorPosSteps = rangeSteps;

  Serial.print("rangeSteps(hit-to-hit) = ");
  Serial.println(rangeSteps);

  // back off from sw2 until it releases (or ENDSTOP_MARGIN_STEPS)
  for (int i = 0; i < ENDSTOP_MARGIN_STEPS; i++) {
    if (digitalRead(sw2Pin) == HIGH) break;
    doOneStep(DIR_TO_SW1);
    delayMicroseconds(STEP_PERIOD_US);
    motorPosSteps--; // moved away from sw2
  }

  Serial.print("motorPosSteps after backoff = ");
  Serial.println(motorPosSteps);
}

//clamps pos to 0–1 and linearly maps it to a step position between minSteps and maxSteps
//where both ends are kept ENDSTOP_MARGIN_STEPS away from the endstops. This way pos=0 and pos=100 
//correspond to safe positions near the switches without physically pressing them
//Note: Since the electrical connections are not perfect some steps are missing and it looses
//      track of the correct position
long posToTargetSteps(float pos) {
  if (pos < 0.0f) pos = 0.0f;
  if (pos > 1.0f) pos = 1.0f;

  long minSteps = ENDSTOP_MARGIN_STEPS;
  long maxSteps = rangeSteps - ENDSTOP_MARGIN_STEPS;
  if (maxSteps < minSteps) maxSteps = minSteps;

  return minSteps + (long)((maxSteps - minSteps) * pos + 0.5f);
}

//Calculates the angle between new vector and vector A and outputs its angle 0-1
float compute01(float vUnit[3]) {
  float d = clampf(dot3(poseA, vUnit), -1.0, 1.0);
  float angleAV = acos(d);
  float t = angleAV / angleAB;
  return clampf(t, 0.0, 1.0);
}

void stepperService() {
  if (rangeSteps <= 0) return;

  long err = targetSteps - motorPosSteps;

  // reached target -> stop and reset speed
  if (abs(err) <= MOTOR_DEADBAND_STEPS) {
    stepPeriodUs = STEP_PERIOD_MAX_US;
    lastDirSign = 0;
    return;
  }

  int dirSign;
  if (err > 0) dirSign = DIR_TO_SW2;
  else         dirSign = DIR_TO_SW1;

  int safetyPin;
  if (dirSign == DIR_TO_SW2) safetyPin = sw2Pin;
  else                       safetyPin = sw1Pin;

  // endstop safety
  if (digitalRead(safetyPin) == LOW) {
    stepPeriodUs = STEP_PERIOD_MAX_US;
    lastDirSign = 0;
    return;
  }

  // timing (uses the current stepPeriodUs)
  unsigned long now = micros();
  if ((long)(now - nextStepUs) < 0) return;
  nextStepUs = now + stepPeriodUs;

  // do one step
  doOneStep(dirSign);
  if (dirSign == DIR_TO_SW2) motorPosSteps += 1;   // moved toward sw2 => position increases
  else                       motorPosSteps -= 1;

  // accelleration and deacceleration
  bool decel = (abs(err) < DECEL_WINDOW_STEPS);
  if (lastDirSign != dirSign) {
    // direction changed -> reset to slow
    stepPeriodUs = STEP_PERIOD_MAX_US;
  } else {
    if (decel) {
      stepPeriodUs = (stepPeriodUs * (unsigned long)DECEL_MULT_PERMILLE) / 1000UL;
      if (stepPeriodUs > STEP_PERIOD_MAX_US) stepPeriodUs = STEP_PERIOD_MAX_US;
    } else {
      stepPeriodUs = (stepPeriodUs * (unsigned long)ACCEL_MULT_PERMILLE) / 1000UL;
      if (stepPeriodUs < STEP_PERIOD_MIN_US) stepPeriodUs = STEP_PERIOD_MIN_US;
    }
  }

  lastDirSign = dirSign;

}


// make sound
void buzzStepper(unsigned int freqHz, unsigned int durationMs) {

  // If an endstop is currently pressed, do nothing
  if (digitalRead(sw1Pin) == LOW || digitalRead(sw2Pin) == LOW) return;

  unsigned long periodUs = 1000000UL / freqHz;

  // number of steps to execute
  unsigned long stepsTotal = (unsigned long)freqHz * durationMs / 1000UL;

  // make it so it ends where it started
  if (stepsTotal & 1UL) stepsTotal--;

  int dir = +1;
  for (unsigned long i = 0; i < stepsTotal; i++) {
    // safety: stop buzzing if an endstop gets hit
    if (digitalRead(sw1Pin) == LOW || digitalRead(sw2Pin) == LOW) break;

    doOneStep(dir);
    dir = -dir;                         // alternate direction each step
    delayMicroseconds(periodUs);
  }
}

void playStartupSound() {
  delay(500);
  buzzStepper(880,  110);  delay(35);
  buzzStepper(1109, 110);  delay(35);
  buzzStepper(1319, 140);  delay(55);
  buzzStepper(1760, 220);  delay(70);
  buzzStepper(1319,  80);  delay(20);
  buzzStepper(1760, 320);
}

// setup
void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(sw1Pin, INPUT_PULLUP);
  pinMode(sw2Pin, INPUT_PULLUP);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);


  imu.initialize();
  resetPosMovingAverage();

  Serial.println("Calibration");
  Serial.println("Hold Pose A and PRESS any end switch...");

  // wait for press
  while (buttonsReleased()) {
    // do nothing
  }
  capturePose(poseA);
  Serial.println("Pose A saved.");

  // wait for release
  while (buttonPressed()) {
    // do nothing
  }

  Serial.println("Release detected. Rotate arm now...");
  delay(IDLE_TIME_MS);

  Serial.println("Hold Pose B and PRESS any end switch...");

  // wait for press
  while (buttonsReleased()) {
    // do nothing
  }
  capturePose(poseB);
  Serial.println("Pose B saved.");

  // wait for release
  while (buttonPressed()) {
    // do nothing
  }

  // compute angle between A and B
  float d = clampf(dot3(poseA, poseB), -1.0, 1.0);
  angleAB = acos(d);

  Serial.print("Angle A->B (rad): ");
  Serial.println(angleAB, 6);

  if (angleAB < 0.2) {
    Serial.println("ERROR: Poses too similar. Reset and try again with larger rotation.");
    while (true);

  } else {
    Serial.println("Setup completed.");
  }

  homeAndMeasureRange();
  targetSteps = motorPosSteps;

  Serial.println("Homing completed.");
  //playStartupSound();
}

void loop() {
  readAccel();

  float v[3] = { (float)ax, (float)ay, (float)az };
  normalize3(v);

  float posRaw = compute01(v);          // 0..1
  float pos    = pushPosMovingAverage(posRaw);

  static unsigned long lastTargetUpdateMs = 0;

  long newTarget = posToTargetSteps(pos);

  if (millis() - lastTargetUpdateMs >= 20) {
    if (labs(newTarget - targetSteps) > TARGET_HYST_STEPS) {
      targetSteps = newTarget;
      lastTargetUpdateMs = millis();
    }
  }


  stepperService();

  // debug print
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 50) {
    lastPrint = millis();
    Serial.print("pos:"); Serial.print(pos, 3); Serial.print('\t');
    Serial.print("posRaw:"); Serial.print(posRaw, 3); Serial.print('\t');
    Serial.print("motorPos:"); Serial.print(motorPosSteps); Serial.print('\t');
    Serial.print("target:"); Serial.print(targetSteps);
    Serial.println();
  }
}

