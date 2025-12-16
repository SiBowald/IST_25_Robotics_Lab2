// This is a script to translate two positions into a value between 0 and 100
// Manual
// 1. Go to the first position
// 2. Press one of the end switches
// 3. Release the switch and wait approximately 1s in the same position
// 4. Go to the second position
// 5. Press one of the end switches
// 6. Release the switch and wait approximately 1s in the same position
// 7. Move the arm between the two positions and observe the pos value go from 0 to 100

#include <Wire.h>
#include "ICM20600.h"

// pins
#define sw1Pin 10
#define sw2Pin 11

// settings
#define CAL_SAMPLES 30
#define CAL_DELAY_MS 5
#define IDLE_TIME_MS 500

#define MA_N 5   // moving average window size (e.g. 3, 5, 10)

// global
ICM20600 imu(true);

int16_t ax, ay, az;

// Pose vectors
float poseA[3] = {0, 0, 0};
float poseB[3] = {0, 0, 0};
float angleAB = 0.0;  // radians between poseA and poseB

// Moving average buffer
int maBuf[MA_N];
long maSum = 0;
int maIdx = 0;
int maCount = 0;

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
void resetMovingAverage() {
  maSum = 0;
  maIdx = 0;
  maCount = 0;
  for (int i = 0; i < MA_N; i++) 
    maBuf[i] = 0;
}

// keeps a moving average (a rolling average) of the last MA_N values
int pushMovingAverage(int v) {
  if (maCount < MA_N) {
    maBuf[maIdx] = v;
    maSum += v;
    maCount++;
  } else {
    maSum -= maBuf[maIdx];
    maBuf[maIdx] = v;
    maSum += v;
  }
  maIdx = (maIdx + 1) % MA_N;

  // rounded average
  return (int)((maSum + (maCount / 2)) / maCount);
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

// convert current accel direction to 0-100 using angle from poseA
int computePos0to100(float vUnit[3]) {
  if (angleAB < 1e-4) return 0; //prevents from crashing if zero

  float d = clampf(dot3(poseA, vUnit), -1.0, 1.0);
  float angleAV = acos(d);      // 0 to pi
  float t = angleAV / angleAB;  // normally 0..1
  t = clampf(t, 0.0, 1.0);

  int pos = (int)(t * 100.0 + 0.5); //+ 0.5 is there to round to the nearest integer instead of always truncating down.
  if (pos < 0) pos = 0;
  if (pos > 100) pos = 100;
  return pos;
}

// ------------------------- setup ----------------------------
void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(sw1Pin, INPUT_PULLUP);
  pinMode(sw2Pin, INPUT_PULLUP);

  imu.initialize();
  resetMovingAverage();

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
}

void loop() {
  readAccel();

  // normalize current accel vector
  float v[3] = { (float)ax, (float)ay, (float)az };
  normalize3(v);

  int posRaw = -1;
  int pos = -1;

  posRaw = computePos0to100(v);
  pos = pushMovingAverage(posRaw);

  // Serial Plotter outputs
  Serial.print("pos:"); Serial.print(pos); Serial.print('\t');
  Serial.print("posRaw:"); Serial.print(posRaw); Serial.print('\t');

  Serial.print("ax:"); Serial.print(ax); Serial.print('\t');
  Serial.print("ay:"); Serial.print(ay); Serial.print('\t');
  Serial.print("az:"); Serial.print(az); Serial.print('\t');

  Serial.println();

  delay(10);
}
