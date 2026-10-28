#include <Wire.h>
#define MY_ADDR 8

// 09/25/2026
// Derrick Scott Joao Vitor Peclat Fayad Bode Lamb
//
// Turn in place by desiredPhi, then drive desiredDistance.
// Right wheel = index 0, left wheel = index 1.
//
// // This code now uses a statemachine to define what the Robot should be doing at any moment.
// The turnign state allows it to turn, the zeroing state resets all the variables, and the 
// drive state moves the robot a certain distance. There is also a voltage trimming module
// that allows for the wheels to be synced up and turn at the same rate
//
//
//

// Timing
unsigned long desired_Ts_ms = 20;   // sample period, ms
unsigned long last_time_ms;
unsigned long start_time_ms;
unsigned long current_time_ms = 0;
float dt = 0;                       // seconds, sample period as a float
// ---------------------------------------------------------------------

//PI Communication this is all communication stuff from the tutorial
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
// ---------------------------------------------------------------------

// Encoder pins
const int encoderApinR = 2;
const int encoderBpinR = 5;
const int encoderApinL = 3;
const int encoderBpinL = 6;
// ---------------------------------------------------------------------

// Motor driver pins
const int STBY_PIN = 4;
const int SIGN_PIN[2] = {7, 8};
const int PWM_PINs[2] = {9, 10};
// ---------------------------------------------------------------------

// constants determined by the physical design
const float radius = 0.0747;       // wheel radius, meters
const float fullRotation = 3200;       // encoder counts per full wheel rotation
const float robotDiameter = 0.358775;  // meters, distance between wheels
// ---------------------------------------------------------------------

// Position / velocity (angular, per-wheel)
volatile long encoderCount[2] = {0, 0};
float desired_wheel_theta[2] = {0, 0}; // rad
float theta[2] = {0, 0};               // rad
float prevTheta[2] = {0, 0};           // rad
float angularVelocity[2] = {0, 0};     // rad/s
float linearVelocity[2] = {0, 0};      // m/s
float deltaTheta[2] = {0, 0};          // position error
float accumTheta[2] = {0, 0};          // accumulated position error
// ---------------------------------------------------------------------

// Odometry
float odometry[3] = {0, 0, 0}; // x, y (m) and phi (rad)
float phiVelocity = 0;         // rad/s
// ---------------------------------------------------------------------

// Motor control
float angularVelocity_SP[2];   // angular velocity setpoint, rad/s
float voltage[2] = {0, 0};
const float Ki_pos = .25;      // integral gain
const float Kp_pos = 15;       // proportional gain
const float Kp_vel = 2;

const float turnVoltLimit  = 3.0;  // V clamp while turning (was 2.0; raised to get past motor deadband)
const float driveVoltLimit = 4.0;  // V clamp while driving

// Wheel sync (cross-coupling) tuning
const float Ksync   = 5.0;   // V per rad of wheel-progress mismatch
const float maxTrim = 0.5;   // V, largest correction applied to each wheel (lowered so sync can't stall a wheel)
// ---------------------------------------------------------------------

// Movement
float desiredPhi = 0 * PI / 180;          // degrees converted to radians
float desiredDistance = 9 / 3.2808399;        // feet converted to meters

enum MoveState { TURNING, ZEROING, DRIVING };
MoveState moveState = TURNING;

// "Turn finished" detection
const float settlePosTol = 0.05;            // rad of wheel position error
const float settleVelTol = 0.2;             // rad/s
const unsigned long settleHold_ms = 200;    // must stay settled this long
const unsigned long turnTimeout_ms = 20000; // give up on turn after this long
bool settling = false;
unsigned long settleStart_ms = 0;
float currentDistance = 0;
// ---------------------------------------------------------------------

void setMotor(int i, float v) {
  // negative voltage = backwards, positive = forwards
  if (v > 0) {
    digitalWrite(SIGN_PIN[i], !i ? LOW : HIGH);
    analogWrite(PWM_PINs[i], constrain((int)(fabs(v) / 8.0 * 255), 0, 255));
  } else if (v < 0) {
    digitalWrite(SIGN_PIN[i], !i ? HIGH : LOW);
    analogWrite(PWM_PINs[i], constrain((int)(fabs(v) / 8.0 * 255), 0, 255));
  } else {
    analogWrite(PWM_PINs[i], 0);
  }
}

void setup() {
  pinMode(encoderApinR, INPUT_PULLUP);
  pinMode(encoderBpinR, INPUT_PULLUP);
  pinMode(encoderApinL, INPUT_PULLUP);
  pinMode(encoderBpinL, INPUT_PULLUP);

  pinMode(STBY_PIN, OUTPUT);
  pinMode(SIGN_PIN[0], OUTPUT);
  pinMode(SIGN_PIN[1], OUTPUT);
  pinMode(PWM_PINs[0], OUTPUT);
  pinMode(PWM_PINs[1], OUTPUT);

  digitalWrite(STBY_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(encoderApinR), encoderISR_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderApinL), encoderISR_L, CHANGE);

  Serial.begin(115200);
  last_time_ms = millis();
  start_time_ms = last_time_ms;
  Serial.println("");
  Serial.println("Robot started");
}

void loop() {
  current_time_ms = millis();
  digitalWrite(STBY_PIN, HIGH);

  // read encoders without being interrupted (long, NOT int)
  noInterrupts();
  long localCountL = encoderCount[1];
  long localCountR = encoderCount[0];
  interrupts();

  theta[0] = ((float)localCountR / fullRotation) * 2 * PI;
  theta[1] = ((float)localCountL / fullRotation) * 2 * PI;

  // runs every desired_Ts_ms
  if (current_time_ms - last_time_ms >= desired_Ts_ms) {
    float elapsed_ms = (float)(current_time_ms - last_time_ms);
    dt = elapsed_ms / 1000.0;

    // velocities
    for (int i = 0; i < 2; i++) {
      angularVelocity[i] = (theta[i] - prevTheta[i]) / dt;
      linearVelocity[i] = angularVelocity[i] * radius;
    }

    // odometry
    phiVelocity = radius * ((angularVelocity[0] - angularVelocity[1]) / robotDiameter);
    odometry[2] += ((linearVelocity[0] - linearVelocity[1]) / robotDiameter) * dt;
    float v = (linearVelocity[0] + linearVelocity[1]) / 2;
    odometry[0] += cos(odometry[2]) * v * dt;
    odometry[1] += sin(odometry[2]) * v * dt;

    long denom = (odometry[0] * odometry[0]) + (odometry[1] * odometry[1]);
    
    currentDistance = sqrt(denom);

    // ---------------- movement state machine ----------------
    if (moveState == TURNING) {
      // each wheel travels an arc of (D/2)*phi  ->  wheel angle = D*phi/(2r)
      desired_wheel_theta[0] = robotDiameter * desiredPhi / (2.0 * radius);
      desired_wheel_theta[1] = -desired_wheel_theta[0];

      bool inTol =
        fabs(desired_wheel_theta[0] - theta[0]) < settlePosTol &&
        fabs(desired_wheel_theta[1] - theta[1]) < settlePosTol &&
        fabs(angularVelocity[0]) < settleVelTol &&
        fabs(angularVelocity[1]) < settleVelTol;

      if (inTol) {
        if (!settling) {
          settling = true;
          settleStart_ms = current_time_ms;
        } else if (current_time_ms - settleStart_ms >= settleHold_ms) {
          moveState = ZEROING;
        }
      } else {
        settling = false;
      }

      if (current_time_ms - start_time_ms > turnTimeout_ms) {
        moveState = ZEROING;   // safety: don't hang forever on a stuck turn
      }
    }

    if (moveState == ZEROING) {
      // zero everything in one shot so there's no velocity spike
      noInterrupts();
      encoderCount[0] = 0;
      encoderCount[1] = 0;
      interrupts();
      for (int i = 0; i < 2; i++) {
        theta[i] = 0;
        prevTheta[i] = 0;
        accumTheta[i] = 0;
        desired_wheel_theta[i] = 0;
        angularVelocity[i] = 0;
      }
      moveState = DRIVING;
    }

    if (moveState == DRIVING) {
      for (int i = 0; i < 2; i++) {
        desired_wheel_theta[i] = desiredDistance / radius;
      }
    }

    float limit = (moveState == DRIVING) ? driveVoltLimit : turnVoltLimit;

    // ---------------- per-wheel position -> velocity -> voltage ----------------
    for (int i = 0; i < 2; i++) {
      deltaTheta[i] = desired_wheel_theta[i] - theta[i];
      accumTheta[i] += deltaTheta[i] * desired_Ts_ms;
      prevTheta[i] = theta[i];

      angularVelocity_SP[i] = Ki_pos * accumTheta[i] + Kp_pos * deltaTheta[i];
      voltage[i] = Kp_vel * (angularVelocity_SP[i] - angularVelocity[i]);

      // clamp + anti-windup
      if (voltage[i] >= limit) {
        voltage[i] = limit;
        accumTheta[i] -= deltaTheta[i] * desired_Ts_ms;
      } else if (voltage[i] <= -limit) {
        voltage[i] = -limit;
        accumTheta[i] -= deltaTheta[i] * desired_Ts_ms;
      }
    }

    // ---------------- wheel sync (cross-coupling) ----------------
    // "progress" of each wheel is measured in its own direction of travel, so
    // this works for driving straight (both +) and for turning (+ and -).
    float s0 = (desired_wheel_theta[0] > 0) - (desired_wheel_theta[0] < 0);
    float s1 = (desired_wheel_theta[1] > 0) - (desired_wheel_theta[1] < 0);
    if (s0 != 0 && s1 != 0) {
      float syncErr = s0 * theta[0] - s1 * theta[1];   // >0 : right wheel ahead
      float trim = constrain(Ksync * syncErr, -maxTrim, maxTrim);
      voltage[0] -= s0 * trim;   // slow the wheel that's ahead
      voltage[1] += s1 * trim;   // speed up the wheel that's behind
    }

    // ---------------- write motors (once) ----------------
    setMotor(0, voltage[0]);
    setMotor(1, voltage[1]);

    last_time_ms = current_time_ms;
    // debug: state, wheel angles, commanded voltages, phi
    Serial.print("Desired Distance ");
    Serial.print(desiredDistance, 5);
    Serial.print(" Current Distance ");
    Serial.print(currentDistance, 5);
    Serial.print(" desiredThethaLeft ");
    Serial.print(desired_wheel_theta[1], 5);
    Serial.print(" desiredThethaRight ");
    Serial.print(desired_wheel_theta[0], 5);
    Serial.print('\n');

  }
}

// ---------------------------------------------------------------------
// Encoder ISRs
// ---------------------------------------------------------------------
void encoderISR_L() {
  int AL = digitalRead(encoderApinL);
  int BL = digitalRead(encoderBpinL);
  if (AL == BL) {
    encoderCount[1] -= 2;
  } else {
    encoderCount[1] += 2;
  }
}

void encoderISR_R() {
  int AR = digitalRead(encoderApinR);
  int BR = digitalRead(encoderBpinR);
  if (AR == BR) {
    encoderCount[0] += 2;
  } else {
    encoderCount[0] -= 2;
  }
}
