#include <Wire.h>
#define MY_ADDR 8

// 09/25/2026
// Derrick Scott Joao Vitor Peclat Fayad Bode Lamb
//
// This code lets you set the positions of the wheels in 4 configurations
// each wheel has a "top" considered to be 0, and a "bottom" considered to 
// be 1.
//
//
//for arrays we made the right wheel 0 and the left wheel one

// Timing
unsigned long desired_Ts_ms = 20;   // sample period, ms
unsigned long last_time_ms;
unsigned long start_time_ms;
long current_time_ms = 0;             // seconds since start
float dt = 0;                       // seconds, sample period as a float
// ---------------------------------------------------------------------

//PI Communication this is all communication stuff from the tutorial
volatile uint8_t offset = 0; 
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
// ---------------------------------------------------------------------

// Encoder pins
const int encoderApinR = 2;   // was RencApin / encoderAR
const int encoderBpinR = 5;   // was RencBpin / encoderBR
const int encoderApinL = 3;   // was LencApin / encoderAL
const int encoderBpinL = 6;   // was LencBpin / encoderBL
// ---------------------------------------------------------------------

// Motor driver pins
const int STBY_PIN = 4; 
const int SIGN_PIN[2] = {7, 8};
const int PWM_PINs[2] = {9, 10};
// ---------------------------------------------------------------------

// constants determined by the physical design
const float radius = 0.07381875;      // wheel radius, meters
const float fullRotation = 3200;    // encoder counts per full wheel rotation
const float robotDiameter = 0.358775;  // meters, distance between wheels
// ---------------------------------------------------------------------

// Position / velocity (angular, per-wheel)
volatile long encoderCount[2] = {0, 0};
float desired_wheel_theta[2] = {0 , 0}; //rad
float theta[2] = {0, 0};                     // rad
float prevTheta[2];     // rad
float angularVelocity[2];  // rad/s
float linearVelocity[2];    // m/s (= angular * radius)
float deltaTheta[2] = {0,0}; // change in theta
float accumTheta[2] = {0,0}; //the acumulated change in theta
// ---------------------------------------------------------------------

// Odometry 
float odometry[3] = {0,0,0}; // x, y, and phi. X and Y are in meters and phi is in radians.
float phiVelocity = 0; //Rad/s
float rVector = 0; // Meters;
float waitTime = 5000; //s
// ---------------------------------------------------------------------

// Motor control 
float angularVelocity_SP[2];   //angular velocity setpoint, rad/s
float voltage[2];             // set directly to bypass velocity control
float DC_gain = 0.25;
const float Ki_pos = .25; //integral gain
const float Kp_pos = 15; //proportional gain
const float Kp_vel = 2;
// ---------------------------------------------------------------------

// Movement
float desiredPhi = 0 * PI / 180; //Degrees converted to radians
float desiredDistance = 3 / 3.28084; // feet converted to Meters
float phiError = 0; // radians
float circumference = 2 * PI * radius;
float linearSpeed = 0;


void setup() {
  //sets the encoder pins as inputs
  pinMode(encoderApinR, INPUT_PULLUP);
  pinMode(encoderBpinR, INPUT_PULLUP);
  pinMode(encoderApinL, INPUT_PULLUP);
  pinMode(encoderBpinL, INPUT_PULLUP);
  //sets the motor pins as outputs
  pinMode(STBY_PIN, OUTPUT);
  pinMode(SIGN_PIN[0], OUTPUT);
  pinMode(SIGN_PIN[1], OUTPUT);
  pinMode(PWM_PINs[0], OUTPUT);
  pinMode(PWM_PINs[1], OUTPUT);

  digitalWrite(STBY_PIN, LOW);
// creates the interupts for the interrupts
  attachInterrupt(digitalPinToInterrupt(encoderApinR), encoderISR_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderApinL), encoderISR_L, CHANGE);

  // sets the baudrate and starts the interval timer we'll be using for odometry
  Serial.begin(115200);
  last_time_ms = millis();
  start_time_ms = last_time_ms;
  Serial.println("");
}

void loop() {
  //starts the current time timer
  current_time_ms = millis();

  // writes to the standby pin
  digitalWrite(STBY_PIN, HIGH);

  // updates the encoders and makes sure it's not interuptted during
  noInterrupts();
  int localCountL = encoderCount[1];
  int localCountR = encoderCount[0];
  interrupts();
  // calculates the current theta
  theta[0] = ((float)localCountR / (float)fullRotation) * 2 * PI;
  theta[1] = ((float)localCountL / (float)fullRotation) * 2 * PI;
  // this if occurs every 20 ms aka desired_Ts_ms
  if (current_time_ms - last_time_ms >= desired_Ts_ms) {
    // calculates the current velocity
    angularVelocity[0] = 1000.0 * (theta[0] - prevTheta[0]) / (float)(current_time_ms - last_time_ms);
    angularVelocity[1] = 1000.0 * (theta[1] - prevTheta[1]) / (float)(current_time_ms - last_time_ms);

     // calculates linear velocity
      linearVelocity[1] = angularVelocity[1] * radius;
      linearVelocity[0] = angularVelocity[0] * radius;
      // calculates change in time
      dt = (float)(current_time_ms - last_time_ms) / 1000.0;
      // calculates phi
      phiVelocity = radius * ((angularVelocity[0] - angularVelocity[1])/ robotDiameter);
      odometry[2] += ((linearVelocity[0] - linearVelocity[1]) / robotDiameter) * dt;
      //calculates x
      odometry[0] += cos(odometry[2]) * (linearVelocity[1] + linearVelocity[0]) / 2 * dt;
      //calculates y
      odometry[1] += sin(odometry[2]) * (linearVelocity[1] + linearVelocity[0]) / 2 * dt;
      

    for(int i = 0; i < 2; i++){
    //this makes sure that if the voltage is negative it goes backwards and if it's positive it goes forwards.
    if (voltage[i] > 0) {
      digitalWrite(SIGN_PIN[i], !i ? LOW : HIGH);
      analogWrite(PWM_PINs[i], (abs(voltage[i]) / 8.0 * 255));
    } else if (voltage[i] < 0) {
      digitalWrite(SIGN_PIN[i], !i ? HIGH : LOW);
      analogWrite(PWM_PINs[i], (abs(voltage[i]) / 8.0 * 255));
    }
    else{
      analogWrite(PWM_PINs[i], 0);
    }
    }
    
  if(current_time_ms <= waitTime){  
    desired_wheel_theta[0] = robotDiameter * PI * desiredPhi/circumference;
    desired_wheel_theta[1] = -desired_wheel_theta[0];
  }
  else if (current_time_ms <= waitTime + 50){
    for(int i = 0; i < 2; i++){
    encoderCount[i] = 0;
    desired_wheel_theta[i] = 0;
    }
  }
  else {
    for(int i = 0; i < 2; i++){
    desired_wheel_theta[i] = (desiredDistance * 2 * PI)/circumference;
    }}
  
  linearSpeed = (linearVelocity[0] - linearVelocity[1])/2;

  // this for loop runs twice to make sure it does the same calculations for both wheels.
    for(int i = 0; i < 2; i++){
    
    //this calculates the error  in theta
    deltaTheta[i] = desired_wheel_theta[i] - theta[i];
    // calculates the accumulated error in theta
    accumTheta[i] += deltaTheta[i] * desired_Ts_ms;
    //sets previous theta to current theta after we dont need it
    prevTheta[i] = theta[i];
    // this uses our gain to stabilize the angular velocity to something that we want
    angularVelocity_SP[i] = Ki_pos * accumTheta[i] + (Kp_pos * (deltaTheta[i]));
    // calculates the required voltage to spin the wheel at the right speed
    voltage[i] = Kp_vel * (angularVelocity_SP[i] - angularVelocity[i]);

    if(current_time_ms > waitTime + 50){    
  //clamps voltage
    if(voltage[i] >= 4.0){
      voltage[i] = 4;
      accumTheta[i] -= deltaTheta[i] * desired_Ts_ms;
    } else if(voltage[i] <= -4.0){
      voltage[i] = -4;
      accumTheta[i] -= deltaTheta[i] * desired_Ts_ms;
    }
    if(abs(theta[i] - desired_wheel_theta[i]) > 0.1){ voltage[i] += (i ? -1 : 1) * (angularVelocity[1] - angularVelocity[0]);}
        }
    else{
        if(voltage[i] >= 2.0){
      voltage[i] = 2;
      accumTheta[i] -= deltaTheta[i] * desired_Ts_ms;
    } else if(voltage[i] <= -2.0){
      voltage[i] = -2;
      accumTheta[i] -= deltaTheta[i] * desired_Ts_ms;
    }

    }

    //this makes sure that if the voltage is negative it goes backwards and if it's positive it goes forwards.
    if (voltage[i] > 0) {
      digitalWrite(SIGN_PIN[i], !i ? LOW : HIGH);
      analogWrite(PWM_PINs[i], (abs(voltage[i]) / 8.0 * 255));
    } else if (voltage[i] < 0) {
      digitalWrite(SIGN_PIN[i], !i ? HIGH : LOW);
      analogWrite(PWM_PINs[i], (abs(voltage[i]) / 8.0 * 255));
    }
    else{
      analogWrite(PWM_PINs[i], 0);
    }
    }
      // Updates last time
      last_time_ms = current_time_ms;
      Serial.print(odometry[2]);
      Serial.print("\n");
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


// Trims voltage[0] and voltage[1] so both wheels spin at the same speed.
// Call after the base voltages are set and before writing to the motors.
// void matchWheelSpeeds(float dt) {
//   // Only correct when both wheels are commanded in the same direction
//   // (i.e., driving straight). Otherwise reset and do nothing.
//   if (voltage[0] == 0 || voltage[1] == 0 || (voltage[0] > 0) != (voltage[1] > 0)) {
//     syncIntegral = 0;
//     filteredDiff = 0;
//     return;
//   }

//   // Positive diff = right wheel faster. Both wheels' speeds are signed the
//   // same way when driving straight, so this works forward and backward.
//   float diff = angularVelocity[0] - angularVelocity[1];
//   filteredDiff += syncAlpha * (diff - filteredDiff);

//   syncIntegral += filteredDiff * dt;
//   syncIntegral = constrain(syncIntegral, -2.0, 2.0);   // anti-windup

//   float trim = Kp_sync * filteredDiff + Ki_sync * syncIntegral;
//   trim = constrain(trim, -maxTrim, maxTrim);

//   // Slow down the faster wheel and speed up the slower one
//   float dir = (voltage[0] > 0) ? 1.0 : -1.0;
//   voltage[0] -= dir * trim / 2.0;
//   voltage[1] += dir * trim / 2.0;
// }


