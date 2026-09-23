// 09/23/2026
// Derrick Scott
// COMBINED SKETCH — Adapts the Odometry code merge to meet the goals of miniproject 1
//
// This code lets you set the positions of the wheels in 4 configurations
// each wheel has a "top" considered to be 0, and a "bottom" considered to 
// be 1.
//
//

// Timing
unsigned long desired_Ts_ms = 10;   // sample period, ms
unsigned long last_time_ms;
unsigned long start_time_ms;
long current_time_ms = 0;             // seconds since start
float dt = 0;                       // seconds, sample period as a float
// ---------------------------------------------------------------------

// Encoder pins
const int encoderApinR = 2;   // was RencApin / encoderAR
const int encoderApinL = 3;   // was LencApin / encoderAL
const int encoderBpinR = 5;   // was RencBpin / encoderBR
const int encoderBpinL = 6;   // was LencBpin / encoderBL
// ---------------------------------------------------------------------

// Motor driver pins
const int STBY_PIN = 4;
const int SIGN_PIN[2] = {7, 8};
const int PWM_PINs[2] = {9, 10};
// ---------------------------------------------------------------------

// constants determined by the physical design
const float radius = 0.0762;      // wheel radius, meters
const int fullRotation = 3200;    // encoder counts per full wheel rotation
const float trackWidth = 0.3556;  // meters, distance between wheels ("b" in file 2)
// ---------------------------------------------------------------------

// Position / velocity (angular, per-wheel)
volatile long encoderCount[2] = {0, 0};
float desired_wheel_theta[2] = {3.1 , 3.1}; //rad
float theta;                     // rad
float prevTheta[2];     // rad
float angularVelocity;  // rad/s
float linearVelocity[2];    // m/s (= angular * radius)
// ---------------------------------------------------------------------

// Odometry 
float phi = 0;   // robot heading, rad
float x = 0;     // robot x position, m
float y = 0;     // robot y position, m
// ---------------------------------------------------------------------

// Motor control 
float angularVelocity_SP;   //angular velocity setpoint, rad/s
float voltage = 0;             // set directly to bypass velocity control
float DC_gain = 0.25;
const float Ki_pos = .8;
const float Kp_pos = 15;
const float Kp_vel = 2;
// ---------------------------------------------------------------------

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
}

void loop() {

  digitalWrite(STBY_PIN, HIGH);

  if (current_time_ms - last_time_ms >= desired_Ts_ms) {
    for(int i = 0; i<2; i++){

    noInterrupts();
    long localCount = encoderCount[i];
    interrupts();

    theta = ((float)localCount / (float)fullRotation) * 2 * PI;
    angularVelocity = 1000.0 * (theta - prevTheta[i]) / (float)(current_time_ms - last_time_ms);
    prevTheta[i] = theta;

    float angularVelocity_SP = Ki_pos * (theta + Kp_pos * (desired_wheel_theta[i] - theta));
    float voltage = Kp_vel * (angularVelocity_SP - angularVelocity);

    if(voltage >= 8.0){
      voltage = 8;
    } else if(voltage <= -8.0){
      voltage = -8;
    }

    if (voltage > 0) {
      digitalWrite(SIGN_PIN[i], !i ? LOW : HIGH);
      analogWrite(PWM_PINs[i], (abs(voltage) / 8.0 * 255));
    } else if (voltage < 0) {
      digitalWrite(SIGN_PIN[i], !i ? HIGH : LOW);
      analogWrite(PWM_PINs[i], (abs(voltage) / 8.0 * 255));
    }
    else{
      analogWrite(PWM_PINs[i], 0);
    }

    }

      last_time_ms = current_time_ms;



  }
// ODOMETRY CODE NOT UPDATED FOR ARRAYS
      // linearVelocityL = angularVelocityL * radius;
      // linearVelocityR = angularVelocityR * radius;

      // prevThetaL = thetaL;
      // prevThetaR = thetaR;

  
      // phi += ((linearVelocityR - linearVelocityL) / trackWidth) * dt;
      // dt = (float)(now_ms - last_time_ms) / 1000.0;
      // x += cos(phi) * (linearVelocityL + linearVelocityR) / 2 * dt;
      // y += sin(phi) * (linearVelocityL + linearVelocityR) / 2 * dt;
     
      // ---- Print  ----
      // Serial.print(current_time_ms);
      // Serial.print("s \t");
      // Serial.print(angularVelocity);
      // Serial.print("\t");
      // Serial.print(voltage);
      // Serial.print("\t");
      // Serial.print(thetaR);
      // Serial.print("\tx=");
      // Serial.print(x, 4);
      // Serial.print("\ty=");
      // Serial.print(y, 4);
      // Serial.print("\tphi=");
      // Serial.print(phi, 4);
      // Serial.println("");



            current_time_ms = millis();
    Serial.print("");
    Serial.print(current_time_ms);
    Serial.print("\t");
    Serial.print(prevTheta[0]);
    Serial.print("\t");
    Serial.print(prevTheta[1]);
    Serial.print("\n");

} 

// ---------------------------------------------------------------------
// Encoder ISRs 
// ---------------------------------------------------------------------
void encoderISR_L() {
  int AL = digitalRead(encoderApinL);
  int BL = digitalRead(encoderBpinL);
  if (AL == BL) {
    encoderCount[1] += 2;
  } else {
    encoderCount[1] -= 2;
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
