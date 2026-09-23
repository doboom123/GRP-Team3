// 09/23/2026
// Derrick Scott
// COMBINED SKETCH — merges the motor-voltage-control program with the
// two-wheel odometry (dead reckoning) program. See chat for a summary of
// what was merged, what bugs were fixed, and what's still missing.
//
// This code lets you set the voltage (or angular velocity setpoint) that
// the right-hand motor operates at, prints:
//   Time (s) | RHW Velocity (rad/s) | Voltage (V) | RHW Position (rad)
// and simultaneously tracks the robot's estimated pose (x, y, phi) using
// both wheel encoders.
//
// To control the motor, set EITHER rotationalVelocity_SP (angular velocity
// setpoint) OR voltage directly, in setup().
//
// *** NOTE: MotorControl() and MotorSpeed() are CALLED below but are not
// *** DEFINED anywhere in the two files you provided. They must live in
// *** another tab of your original sketch. Add that tab back in, or send
// *** it over and I'll fold it in. Until then this will not compile.

// ---------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------
unsigned long desired_Ts_ms = 20;   // sample period, ms
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time = 0;             // seconds since start
float dt = 0;                       // seconds, sample period as a float

// ---------------------------------------------------------------------
// Encoder pins (COMBINED: file 1 and file 2 used different names for the
// same physical pins — verified identical, kept one set)
// ---------------------------------------------------------------------
const int encoderApinR = 2;   // was RencApin / encoderAR
const int encoderApinL = 3;   // was LencApin / encoderAL
const int encoderBpinR = 5;   // was RencBpin / encoderBR
const int encoderBpinL = 6;   // was LencBpin / encoderBL

// Motor driver pins (from file 1; unchanged)
const int STBY_PIN = 4;
const int AIN1_PIN = 7;
const int AIN2_PIN = 8;
const int PWMA_PIN = 9;
const int PWMB_PIN = 10;

// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
const float radius = 0.0762;      // wheel radius, meters
const int fullRotation = 3200;    // encoder counts per full wheel rotation
const float trackWidth = 0.3556;  // meters, distance between wheels ("b" in file 2)

// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

// ---------------------------------------------------------------------
// Position / velocity (angular, per-wheel)
// ---------------------------------------------------------------------
float thetaL = 0, thetaR = 0;             // rad
float prevThetaL = 0, prevThetaR = 0;     // rad
float angularVelocityL = 0, angularVelocityR = 0;  // rad/s
float linearVelocityL = 0, linearVelocityR = 0;    // m/s (= angular * radius)

// ---------------------------------------------------------------------
// Odometry 
// ---------------------------------------------------------------------
float phi = 0;   // robot heading, rad
float x = 0;     // robot x position, m
float y = 0;     // robot y position, m

// ---------------------------------------------------------------------
// Motor control 
// ---------------------------------------------------------------------
float rotationalVelocity_SP = 5;   // right-wheel angular velocity setpoint, rad/s
float voltage = 0;                 // set directly to bypass velocity control
float DC_gain = 0.25;

void setup() {

  pinMode(encoderApinR, INPUT_PULLUP);
  pinMode(encoderBpinR, INPUT_PULLUP);
  pinMode(encoderApinL, INPUT_PULLUP);
  pinMode(encoderBpinL, INPUT_PULLUP);

  pinMode(STBY_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);

  digitalWrite(STBY_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(encoderApinR), encoderISR_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderApinL), encoderISR_L, CHANGE);


  Serial.begin(115200);
  last_time_ms = millis();
  start_time_ms = last_time_ms;
  Serial.println("");
}

void loop() {
 
  unsigned long now_ms = millis();

  if (now_ms - last_time_ms >= desired_Ts_ms) {

    // Only run the motor for 3 seconds, for sanity purposes 
    if (now_ms - start_time_ms < 3000) {

      digitalWrite(STBY_PIN, HIGH);
      digitalWrite(AIN1_PIN, LOW);
      digitalWrite(AIN2_PIN, HIGH);

      MotorControl(rotationalVelocity_SP, angularVelocityR, DC_gain, voltage);

      // Voltage protection, keeping voltage limited to 4V
      if (voltage < 4.0 && voltage > 0) {
        analogWrite(PWMA_PIN, (voltage / 8.0 * 255));
        analogWrite(PWMB_PIN, (voltage / 8.0 * 255));
      } else {
        voltage = 2;
      }

      // ---- Read encoder counts safely  ----
      noInterrupts();
      long localCountL = encoderCountL;
      long localCountR = encoderCountR;
      interrupts();

      
      thetaL = ((float)localCountL / (float)fullRotation) * 2 * PI;
      thetaR = ((float)localCountR / (float)fullRotation) * 2 * PI;

  
      angularVelocityL = 1000.0 * (thetaL - prevThetaL) / (float)(now_ms - last_time_ms);
      angularVelocityR = 1000.0 * (thetaR - prevThetaR) / (float)(now_ms - last_time_ms);
      linearVelocityL = angularVelocityL * radius;
      linearVelocityR = angularVelocityR * radius;

      prevThetaL = thetaL;
      prevThetaR = thetaR;

  
      phi += ((linearVelocityR - linearVelocityL) / trackWidth) * dt;
      dt = (float)(now_ms - last_time_ms) / 1000.0;
      x += cos(phi) * (linearVelocityL + linearVelocityR) / 2 * dt;
      y += sin(phi) * (linearVelocityL + linearVelocityR) / 2 * dt;

      current_time = (float)(now_ms - start_time_ms) / 1000;

      // ---- Print  ----
      Serial.print(current_time);
      Serial.print("s \t");
      Serial.print(angularVelocityR);
      Serial.print("\t");
      Serial.print(voltage);
      Serial.print("\t");
      Serial.print(thetaR);
      Serial.print("\tx=");
      Serial.print(x, 4);
      Serial.print("\ty=");
      Serial.print(y, 4);
      Serial.print("\tphi=");
      Serial.print(phi, 4);
      Serial.println("");

      last_time_ms = now_ms;

    } else {
      // Disable the driver and cut power to the motor
      digitalWrite(STBY_PIN, LOW);
      last_time_ms = now_ms;
    }
  }
}

// ---------------------------------------------------------------------
// Encoder ISRs 
// ---------------------------------------------------------------------
void encoderISR_L() {
  int AL = digitalRead(encoderApinL);
  int BL = digitalRead(encoderBpinL);
  if (AL == BL) {
    encoderCountL += 2;
  } else {
    encoderCountL -= 2;
  }
}

void encoderISR_R() {
  int AR = digitalRead(encoderApinR);
  int BR = digitalRead(encoderBpinR);
  if (AR == BR) {
    encoderCountR += 2;
  } else {
    encoderCountR -= 2;
  }
}
