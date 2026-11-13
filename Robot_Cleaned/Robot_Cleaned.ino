#include <Wire.h>
#include <math.h>

// Standalone sketch: replace Demo2.ino and remove the old receive() definition.
// Wheel 0 = right, wheel 1 = left.
// I2C: 9 bytes = flag, four distance digits, four angle digits.
// Distance: [hundreds, tens, units, tenths] cm.
// Angle: [sign (0=negative, 1=positive), tens, units, tenths] degrees.
// Flag 7 means no marker; other flag values mean marker detected.

constexpr uint8_t I2C_ADDRESS = 8;
constexpr uint8_t MOTOR_STBY_PIN = 4;
const uint8_t ENCODER_A[2] = {2, 3};
const uint8_t ENCODER_B[2] = {5, 6};
const uint8_t MOTOR_DIR[2] = {7, 8};
const uint8_t MOTOR_PWM[2] = {9, 10};

constexpr unsigned long CONTROL_PERIOD_MS = 20;
constexpr float WHEEL_RADIUS_M = 0.0747f;
constexpr float WHEEL_BASE_M = 0.358775f;
constexpr float COUNTS_PER_REV = 3200.0f;

// Preserve the existing tuning and units: the position integral accumulates error*ms.
constexpr float KI_POS = 0.25f;
constexpr float KP_POS = 15.0f;
constexpr float KP_VEL = 2.0f;
constexpr float TURN_VOLT_LIMIT = 1.7f;
constexpr float DRIVE_VOLT_LIMIT = 4.0f;
constexpr float K_SYNC = 5.0f;
constexpr float MAX_TRIM = 0.5f;

constexpr float SEARCH_STEP_RAD = 40.0f * PI / 180.0f;
constexpr uint8_t MAX_SEARCH_STEPS = 9; // 9 x 40 degrees = full revolution
constexpr unsigned long CAMERA_PAUSE_MS = 1000;
constexpr unsigned long SETTLE_HOLD_MS = 200;
constexpr unsigned long MOTION_TIMEOUT_MS = 20000;
constexpr float SETTLE_POS_TOL_RAD = 0.05f;
constexpr float SETTLE_VEL_TOL_RAD_S = 0.2f;
constexpr float ALIGN_TOL_RAD = 5.0f * PI / 180.0f;
constexpr float STOP_SHORT_CM = 25.0f;
constexpr unsigned long PACKET_TIMEOUT_MS = 3000;

// Updated inside encoder interrupt handlers.
volatile long encoderCount[2] = {0, 0};

// Data is published by the I2C callback, read atomically in loop().
volatile float rxDistanceCm = 0.0f;
volatile float rxAngleDeg = 0.0f;
volatile bool rxMarkerFound = false;
volatile uint8_t rxSequence = 0;

float theta[2] = {0, 0};
float prevTheta[2] = {0, 0};
float angularVelocity[2] = {0, 0};
float wheelTarget[2] = {0, 0};
float integrator[2] = {0, 0};
float motorVoltage[2] = {0, 0};

float markerDistanceCm = 0;
float markerAngleDeg = 0;
bool markerFound = false;
uint8_t lastRxSequence = 0;
unsigned long lastPacketTimeMs = 0;

// All state changes are made in loop(), never in the I2C interrupt.
enum RobotState : uint8_t {
  SEARCH_PAUSE,
  SEARCH_TURN,
  ALIGN_PAUSE,
  ALIGN_TURN,
  DRIVE,
  COMPLETE,
  FAULT
};
RobotState state = SEARCH_PAUSE;
uint8_t searchStepsCompleted = 0;
unsigned long stateStartMs = 0;
unsigned long settledSinceMs = 0;
bool settling = false;
uint8_t pauseStartSequence = 0;
bool sawFreshPacketInPause = false;
bool sawMarkerInPause = false;
float pauseMarkerDistanceCm = 0;
float pauseMarkerAngleDeg = 0;
unsigned long lastControlMs = 0;

void setMotor(uint8_t i, float volts) {
  if (volts == 0.0f) {
    analogWrite(MOTOR_PWM[i], 0);
    return;
  }
  const bool forward = volts > 0.0f;
  // Right motor forward: LOW. Left motor forward: HIGH.
  digitalWrite(MOTOR_DIR[i], (i == 0) ? (forward ? LOW : HIGH)
                                        : (forward ? HIGH : LOW));
  int duty = (int)(fabsf(volts) * (255.0f / 8.0f));
  analogWrite(MOTOR_PWM[i], constrain(duty, 0, 255));
}

void stopMotors() {
  setMotor(0, 0.0f);
  setMotor(1, 0.0f);
}

void encoderISR_R() {
  if (digitalRead(ENCODER_A[0]) == digitalRead(ENCODER_B[0]))
    encoderCount[0] += 2;
  else
    encoderCount[0] -= 2;
}

void encoderISR_L() {
  if (digitalRead(ENCODER_A[1]) == digitalRead(ENCODER_B[1]))
    encoderCount[1] -= 2;
  else
    encoderCount[1] += 2;
}

// This callback runs only when the Pi actually transmits an I2C packet.
// No Serial, delay, or expensive floating-point math inside the callback.
void receive(int count) {
  if (count != 9) {
    while (Wire.available()) Wire.read();
    return;
  }
  uint8_t bytes[9];
  for (uint8_t i = 0; i < 9; ++i) {
    if (!Wire.available()) return;
    bytes[i] = (uint8_t)Wire.read();
  }
  // Reject malformed digit packets; byte 5 is a sign, not a digit.
  for (uint8_t i = 1; i < 9; ++i) {
    if (bytes[i] > 9) return;
  }
  if (bytes[5] > 1) return;

  const bool seen = bytes[0] != 7;
  const float distance = 100.0f * bytes[1] + 10.0f * bytes[2]
                       + bytes[3] + 0.1f * bytes[4];
  float angle = 10.0f * bytes[6] + bytes[7] + 0.1f * bytes[8];
  if (bytes[5] == 0) angle = -angle;

  rxDistanceCm = seen ? distance : 0.0f;
  rxAngleDeg = seen ? angle : 0.0f;
  rxMarkerFound = seen;
  ++rxSequence;
}

void updateMeasurements(float dtSeconds) {
  long counts[2];
  noInterrupts();
  counts[0] = encoderCount[0];
  counts[1] = encoderCount[1];
  interrupts();
  for (uint8_t i = 0; i < 2; ++i) {
    theta[i] = (float)counts[i] * TWO_PI / COUNTS_PER_REV;
    angularVelocity[i] = (theta[i] - prevTheta[i]) / dtSeconds;
    prevTheta[i] = theta[i];
  }
}

void readLatestPacket(unsigned long now) {
  uint8_t sequence;
  float distance, angle;
  bool found;
  noInterrupts();
  sequence = rxSequence;
  distance = rxDistanceCm;
  angle = rxAngleDeg;
  found = rxMarkerFound;
  interrupts();

  if (sequence != lastRxSequence) {
    lastRxSequence = sequence;
    lastPacketTimeMs = now;
    markerDistanceCm = distance;
    markerAngleDeg = angle;
    markerFound = found;
    if (state == SEARCH_PAUSE || state == ALIGN_PAUSE) {
      sawFreshPacketInPause = true;
      if (found) {
        sawMarkerInPause = true;
        pauseMarkerDistanceCm = distance;
        pauseMarkerAngleDeg = angle;
      }
    }
  }
}

void setState(RobotState newState, unsigned long now) {
  state = newState;
  stateStartMs = now;
  settling = false;
  settledSinceMs = 0;
  if (newState == SEARCH_PAUSE || newState == ALIGN_PAUSE) {
    pauseStartSequence = lastRxSequence;
    sawFreshPacketInPause = false;
    sawMarkerInPause = false;
    // Hold the present position during the camera check.
    for (uint8_t i = 0; i < 2; ++i) wheelTarget[i] = theta[i];
  }
  if (newState == COMPLETE || newState == FAULT) stopMotors();
  Serial.print(F("State: "));
  Serial.println((int)newState);
}

// Increment targets relative to the *current* wheel positions, not encoder zero.
void setTurnTarget(float robotAngleRad) {
  const float wheelRotation = WHEEL_BASE_M * robotAngleRad /
                              (2.0f * WHEEL_RADIUS_M);
  wheelTarget[0] = theta[0] + wheelRotation;
  wheelTarget[1] = theta[1] - wheelRotation;
  integrator[0] = integrator[1] = 0.0f;
}

void setDriveTarget(float distanceM) {
  const float wheelRotation = distanceM / WHEEL_RADIUS_M;
  wheelTarget[0] = theta[0] + wheelRotation;
  wheelTarget[1] = theta[1] + wheelRotation;
  integrator[0] = integrator[1] = 0.0f;
}

bool targetSettled(unsigned long now) {
  bool inTolerance = true;
  for (uint8_t i = 0; i < 2; ++i) {
    if (fabsf(wheelTarget[i] - theta[i]) >= SETTLE_POS_TOL_RAD ||
        fabsf(angularVelocity[i]) >= SETTLE_VEL_TOL_RAD_S) {
      inTolerance = false;
    }
  }
  if (!inTolerance) {
    settling = false;
    return false;
  }
  if (!settling) {
    settledSinceMs = now;
    settling = true;
  }
  return now - settledSinceMs >= SETTLE_HOLD_MS;
}

void updateStateMachine(unsigned long now) {
  switch (state) {
    case SEARCH_PAUSE:
      // Do not advance until the camera has had time to acquire another frame.
      if (now - stateStartMs < CAMERA_PAUSE_MS) break;
      if (!sawFreshPacketInPause) break; // Need at least one fresh Pi report.
      if (sawMarkerInPause) {
        if (fabsf(pauseMarkerAngleDeg) <= ALIGN_TOL_RAD) {
          setState(ALIGN_PAUSE, now); // re-check before approaching
        } else {
          setTurnTarget(pauseMarkerAngleDeg * PI / 180.0f);
          setState(ALIGN_TURN, now);
        }
      } else if (searchStepsCompleted >= MAX_SEARCH_STEPS) {
        setState(COMPLETE, now); // Full scan with no marker.
      } else {
        setTurnTarget(SEARCH_STEP_RAD);
        setState(SEARCH_TURN, now);
      }
      break;

    case SEARCH_TURN:
      if (targetSettled(now)) {
        ++searchStepsCompleted;
        setState(SEARCH_PAUSE, now);
      } else if (now - stateStartMs > MOTION_TIMEOUT_MS) {
        setState(FAULT, now);
      }
      break;

    case ALIGN_TURN:
      if (targetSettled(now)) {
        setState(ALIGN_PAUSE, now);
      } else if (now - stateStartMs > MOTION_TIMEOUT_MS) {
        setState(FAULT, now);
      }
      break;

    case ALIGN_PAUSE:
      if (now - stateStartMs < CAMERA_PAUSE_MS) break;
      if (!sawFreshPacketInPause) break;
      if (!sawMarkerInPause) {
        // Lost marker: resume scanning from this new heading.
        setState(SEARCH_PAUSE, now);
      } else if (fabsf(pauseMarkerAngleDeg) > ALIGN_TOL_RAD) {
        setTurnTarget(pauseMarkerAngleDeg * PI / 180.0f);
        setState(ALIGN_TURN, now);
      } else {
        const float remainingM = fmaxf(0.0f, pauseMarkerDistanceCm - STOP_SHORT_CM) / 100.0f;
        setDriveTarget(remainingM);
        setState(DRIVE, now);
      }
      break;

    case DRIVE:
      if (targetSettled(now)) setState(COMPLETE, now);
      else if (now - stateStartMs > MOTION_TIMEOUT_MS) setState(FAULT, now);
      break;

    case COMPLETE:
    case FAULT:
      break;
  }
}

void updateMotorControl(float dtMs) {
  if (state == COMPLETE || state == FAULT) {
    stopMotors();
    return;
  }
  const float limit = (state == DRIVE) ? DRIVE_VOLT_LIMIT : TURN_VOLT_LIMIT;
  for (uint8_t i = 0; i < 2; ++i) {
    const float error = wheelTarget[i] - theta[i];
    integrator[i] += error * dtMs;
    const float velocityTarget = KI_POS * integrator[i] + KP_POS * error;
    const float command = KP_VEL * (velocityTarget - angularVelocity[i]);
    motorVoltage[i] = constrain(command, -limit, limit);
    if (command > limit || command < -limit) integrator[i] -= error * dtMs;
  }

  // Apply sync only to active motion, using progress *since this motion began*.
  if (state == SEARCH_TURN || state == ALIGN_TURN || state == DRIVE) {
    const float signR = (wheelTarget[0] >= theta[0]) ? 1.0f : -1.0f;
    const float signL = (wheelTarget[1] >= theta[1]) ? 1.0f : -1.0f;
    // Avoid cross-coupling after reaching a wheel target; proportional controllers
    // handle residual error themselves. See notes about improving sync further.
    if (fabsf(wheelTarget[0] - theta[0]) > SETTLE_POS_TOL_RAD &&
        fabsf(wheelTarget[1] - theta[1]) > SETTLE_POS_TOL_RAD) {
      const float remainingMismatch = signR * (wheelTarget[0] - theta[0])
                                    - signL * (wheelTarget[1] - theta[1]);
      const float trim = constrain(-K_SYNC * remainingMismatch, -MAX_TRIM, MAX_TRIM);
      motorVoltage[0] = constrain(motorVoltage[0] - signR * trim, -limit, limit);
      motorVoltage[1] = constrain(motorVoltage[1] + signL * trim, -limit, limit);
    }
  }
  setMotor(0, motorVoltage[0]);
  setMotor(1, motorVoltage[1]);
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 2; ++i) {
    pinMode(ENCODER_A[i], INPUT_PULLUP);
    pinMode(ENCODER_B[i], INPUT_PULLUP);
    pinMode(MOTOR_DIR[i], OUTPUT);
    pinMode(MOTOR_PWM[i], OUTPUT);
  }
  pinMode(MOTOR_STBY_PIN, OUTPUT);
  stopMotors();
  digitalWrite(MOTOR_STBY_PIN, HIGH);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A[0]), encoderISR_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A[1]), encoderISR_L, CHANGE);
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receive);
  lastControlMs = millis();
  setState(SEARCH_PAUSE, lastControlMs);
  Serial.println(F("Robot ready"));
}

void loop() {
  const unsigned long now = millis();
  if (now - lastControlMs < CONTROL_PERIOD_MS) return;
  const float elapsedMs = (float)(now - lastControlMs);
  lastControlMs = now;
  updateMeasurements(elapsedMs * 0.001f);
  readLatestPacket(now);

  // Do not move without a current source of marker observations.
  // This also makes a disconnected Pi fail safe rather than scanning blindly.
  if (lastPacketTimeMs == 0) return;
  if (now - lastPacketTimeMs > PACKET_TIMEOUT_MS &&
      state != COMPLETE && state != FAULT) {
    setState(FAULT, now);
  }

  updateStateMachine(now);
  updateMotorControl(elapsedMs);
}
