// Simple stepper STEP/DIR slow test (no IMU)
// Moves slowly back and forth between endstops

#define stepPin 8
#define dirPin  9
#define sw1Pin 10   // optional endstop 1 (active LOW with INPUT_PULLUP)
#define sw2Pin 11   // optional endstop 2 (active LOW with INPUT_PULLUP)

#define STEP_DELAY_MS 15     // 10..20 ms recommended for very slow test
#define STEP_PULSE_US 5      // 2..10 us typical

// set to 1 if you have endstops connected, 0 if not
#define USE_ENDSTOPS 1
#define FIXED_STEPS 2000     // used only if USE_ENDSTOPS = 0

void doOneStep() {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(stepPin, LOW);
}

void setup() {
  Serial.begin(115200);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

#if USE_ENDSTOPS
  pinMode(sw1Pin, INPUT_PULLUP);
  pinMode(sw2Pin, INPUT_PULLUP);
#endif

  Serial.println("Slow stepper test starting...");
}

void loop() {
#if USE_ENDSTOPS
  // Move toward sw2 until it is pressed
  digitalWrite(dirPin, HIGH);
  while (digitalRead(sw2Pin) == HIGH) {   // HIGH = not pressed
    doOneStep();
    delay(STEP_DELAY_MS);
  }
  delay(300);

  // Move toward sw1 until it is pressed
  digitalWrite(dirPin, LOW);
  while (digitalRead(sw1Pin) == HIGH) {
    doOneStep();
    delay(STEP_DELAY_MS);
  }
  delay(300);

#else
  // No endstops: move FIXED_STEPS forward, then back
  digitalWrite(dirPin, HIGH);
  for (long i = 0; i < FIXED_STEPS; i++) {
    doOneStep();
    delay(STEP_DELAY_MS);
  }
  delay(500);

  digitalWrite(dirPin, LOW);
  for (long i = 0; i < FIXED_STEPS; i++) {
    doOneStep();
    delay(STEP_DELAY_MS);
  }
  delay(500);
#endif
}
