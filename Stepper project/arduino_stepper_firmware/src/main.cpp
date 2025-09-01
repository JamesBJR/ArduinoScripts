#include <Arduino.h>
#include <AccelStepper.h>

// --- Pins ---
const uint8_t STEP_PIN = 2;
const uint8_t DIR_PIN  = 3;
const uint8_t EN_PIN   = 4; // active LOW

// Sensor & encoder pins
const uint8_t SENSOR_PIN = 5; // active-LOW flag switch
const uint8_t ENC_A_PIN  = 6; // quadrature A
const uint8_t ENC_B_PIN  = 7; // quadrature B (moved from pin 8 to pin 7)

// --- Motion defaults ---
const long  STEPS_PER_REV     = 400;    // 1 rev = 400 steps (half-step on a 200-step motor)
const float DEFAULT_MAX_SPS   = 2000.0; // steps/s
const float DEFAULT_ACCEL     = 1000.0; // steps/s^2
const float HOMING_FAST_SPS   = 800.0;  // steps/s during fast approach/backoff
const float HOMING_SLOW_SPS   = 150.0;  // steps/s during final approach

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
String inBuf;

// --- State ---
volatile bool moveActive = false;
volatile long moveTarget = 0;

bool homingActive = false;
uint8_t homingStep = 0;

volatile long encoderPos = 0;

// Homing backoff step counter
long homingBackoffSteps = 0;

// Last encoder state for quadrature decoding
uint8_t lastA = 0, lastB = 0;

// --- Helpers ---
void applyDefaults() {
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true); // enable active-LOW
  stepper.enableOutputs();
  stepper.setMinPulseWidth(3);
  stepper.setMaxSpeed(DEFAULT_MAX_SPS);
  stepper.setAcceleration(DEFAULT_ACCEL);
}

inline void readEncoder() {
  // Rising-edge-of-A decode, use B for direction
  uint8_t A = digitalRead(ENC_A_PIN);
  uint8_t B = digitalRead(ENC_B_PIN);
  if (lastA == LOW && A == HIGH) {
    if (B == LOW) encoderPos++;
    else          encoderPos--;
  }
  lastA = A;
}

// Forward decl
void handleCommand(const String& s);
void updateEncoder();

void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      String tmp = inBuf; inBuf = "";
      tmp.trim();
      handleCommand(tmp);
    } else {
      inBuf += c;
      if (inBuf.length() > 96) inBuf.remove(0, inBuf.length() - 96);
    }
  }
}

// --- Setup ---
void setup() {
  // Encoder & sensor pins — use internal pull-ups unless you have externals
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  // pin 7 now used for quadrature B, index unused
  pinMode(SENSOR_PIN, INPUT_PULLUP); // active-LOW switch

  // Initialize encoder state
  uint8_t A = digitalRead(ENC_A_PIN);
  uint8_t B = digitalRead(ENC_B_PIN);
  static_cast<void>((A << 1) | B); // lastEncoded will be initialized in ISR

  // Attach interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), updateEncoder, CHANGE);

  applyDefaults();

  // One smooth rev on boot (non-blocking)
  stepper.move(STEPS_PER_REV);

  Serial.begin(115200);
  Serial.println(F("Stepper ready."));
  Serial.println(F("Commands:"));
  Serial.println(F("  G <steps>   - Move by <steps>"));
  Serial.println(F("  R           - Move +1 revolution"));
  Serial.println(F("  L           - Move -1 revolution"));
  Serial.println(F("  V <sps>     - Set max speed (steps/s)"));
  Serial.println(F("  A <sps^2>   - Set acceleration (steps/s^2)"));
  Serial.println(F("  E           - Enable driver"));
  Serial.println(F("  D           - Disable driver (sleep)"));
  Serial.println(F("  Z           - Soft stop"));
  Serial.println(F("  S           - Report sensor state"));
  Serial.println(F("  H           - Homing routine"));
}

void updateEncoder() {
  static uint8_t lastA = 0;
  uint8_t A = digitalRead(ENC_A_PIN);
  uint8_t B = digitalRead(ENC_B_PIN);
  // Only count rising edge of A
  if (lastA == LOW && A == HIGH) {
    // Invert direction logic: if B is LOW, decrement; if HIGH, increment
    if (B == LOW) encoderPos--;
    else          encoderPos++;
  }
  lastA = A;
}

void handleCommand(const String& s) {

  if (s.length() == 0) return;
  char cmd = toupper(s[0]);
  String arg = (s.length() > 1) ? s.substring(1) : "";
  arg.trim();

  // Block motion commands while homing
  if (homingActive && (cmd=='G' || cmd=='R' || cmd=='L' || cmd=='Z')) {
    Serial.println(F("Busy: homing in progress."));
    return;
  }

  switch (cmd) {
    case 'G': { // Move by <steps>
      long val = arg.toInt();
      stepper.move(val);
      moveActive = true;
      moveTarget = val;
      Serial.print(F("Moving ")); Serial.print(val); Serial.println(F(" steps."));
    } break;

    case 'R':
      stepper.move(STEPS_PER_REV);
      moveActive = true;
      moveTarget = STEPS_PER_REV;
      Serial.println(F("Moving +1 rev."));
      break;

    case 'L':
      stepper.move(-STEPS_PER_REV);
      moveActive = true;
      moveTarget = -STEPS_PER_REV;
      Serial.println(F("Moving -1 rev."));
      break;

    case 'V': { // Set max speed
      float v = arg.toFloat();
      if (v < 10) v = 10;
      stepper.setMaxSpeed(v);
      Serial.print(F("Max speed set to ")); Serial.println(v);
    } break;

    case 'A': { // Set acceleration
      float a = arg.toFloat();
      if (a < 10) a = 10;
      stepper.setAcceleration(a);
      Serial.print(F("Acceleration set to ")); Serial.println(a);
    } break;

    case 'E':
      stepper.enableOutputs();
      Serial.println(F("Driver enabled."));
      break;

    case 'D':
      stepper.disableOutputs();
      Serial.println(F("Driver disabled (sleep)."));
      break;

    case 'Z':
      stepper.stop();
      Serial.println(F("Stopping (soft)."));
      break;

    case 'S': {
      int sensorState = digitalRead(SENSOR_PIN);
      Serial.print(F("SENSOR (digital): "));
      Serial.println(sensorState);
    } break;

    case 'H': { // Non-blocking homing routine
      Serial.println(F("Starting homing routine..."));
      stepper.enableOutputs();
      stepper.stop();
      stepper.setAcceleration(DEFAULT_ACCEL); 
      stepper.setMaxSpeed(HOMING_FAST_SPS);
      homingActive = true;
      homingStep = 0;
      homingBackoffSteps = 0;
    } break;


    case 'Q': { // Report encoder position
      Serial.print(F("POSITION: "));
      Serial.println(encoderPos);
    } break;

    default:
      Serial.println(F("Unknown command."));
      break;
  }
}

// --- Sensor change reporter ---
int lastSensorValue = -1;
unsigned long lastSensorCheck = 0;
const unsigned long SENSOR_CHECK_INTERVAL = 100; // ms

void reportSensorChange() {
  unsigned long now = millis();
  if (now - lastSensorCheck >= SENSOR_CHECK_INTERVAL) {
    int sensorState = digitalRead(SENSOR_PIN);
    if (sensorState != lastSensorValue) {
      Serial.print(F("SENSOR (digital): "));
      Serial.println(sensorState);
      lastSensorValue = sensorState;
    }
    lastSensorCheck = now;
  }
}

// --- Main loop ---
void loop() {
  pollSerial();
  updateEncoder();

  // If not homing, run normal trapezoid motion
  if (!homingActive) {
    stepper.run();
  }

  reportSensorChange();

  // Move-complete report (uses encoderPos)
  static bool moveWasActive = false;
  if (!homingActive) {
    if (stepper.distanceToGo() != 0) {
      moveWasActive = true;
    } else if (moveWasActive) {
      Serial.print(F("POSITION: "));
      Serial.println(encoderPos);
      moveWasActive = false;
      moveActive = false;
    }
  }

  // --- Non-blocking homing state machine ---
  if (homingActive) {
  // active-LOW switch: HIGH = not triggered (OFF), LOW = triggered (ON)
  int sw = digitalRead(SENSOR_PIN);

    switch (homingStep) {
      case 0: { // Move toward sensor until triggered (ON)
        stepper.setMaxSpeed(HOMING_FAST_SPS);
        stepper.setAcceleration(DEFAULT_ACCEL);
        if (sw == LOW) {
          homingStep = 1;
          Serial.println(F("Sensor ON. Stepping back to find OFF edge..."));
        } else {
          stepper.move(1); // Move toward sensor (reverse direction)
        }
        stepper.run();
      } break;

      case 1: { // Step back until sensor turns OFF
        stepper.setMaxSpeed(HOMING_SLOW_SPS);
        stepper.setAcceleration(DEFAULT_ACCEL);
        if (sw == HIGH) {
          homingStep = 2;
          Serial.println(F("Sensor OFF. Stepping forward to find ON edge..."));
        } else {
          stepper.move(-1); // Step back (reverse direction)
        }
        stepper.run();
      } break;

      case 2: { // Step forward until sensor turns ON (exact edge)
        stepper.setMaxSpeed(HOMING_SLOW_SPS);
        stepper.setAcceleration(DEFAULT_ACCEL);
        if (sw == LOW) {
          homingStep = 3;
          Serial.println(F("Edge found. Zeroing position."));
        } else {
          stepper.move(1); // Step forward (reverse direction)
        }
        stepper.run();
      } break;

      case 3: { // Zero position and finish
        stepper.setCurrentPosition(0);
        encoderPos = 0;
        stepper.setAcceleration(DEFAULT_ACCEL);
        stepper.setMaxSpeed(DEFAULT_MAX_SPS);
        Serial.println(F("Stepper position set to 0."));
        Serial.print(F("POSITION: "));
        Serial.println(encoderPos);
        homingActive = false;
      } break;
    }
  }
}