#include <Wire.h>
#define MY_ADDR 8
#define PI 3.1415926535897932384626433832795

// 09/25/2026
// Derrick Scott Joao Vitor Peclat Fayad
//
// This code lets you set the positions of the wheels in 4 configurations
// each wheel has a "top" considered to be 0, and a "bottom" considered to 
// be 1.
//
//

// Timing
unsigned long desired_Ts_ms = 20;   // sample period, ms
unsigned long last_time_ms;
unsigned long start_time_ms;
long current_time_ms = 0;             // seconds since start
float dt = 0;                       // seconds, sample period as a float
// ---------------------------------------------------------------------

//PI Communication
volatile uint8_t offset = 0;
volatile uint8_t instruction[2] = {0};
volatile uint8_t msgLength = 0;
int currentDirection[2] = {0};
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
const float radius = 0.0762;      // wheel radius, meters
const float fullRotation = 3200;    // encoder counts per full wheel rotation
const float robotDiameter = 0.3556;  // meters, distance between wheels
// ---------------------------------------------------------------------

// Position / velocity (angular, per-wheel)
volatile long encoderCount[2] = {0, 0};
float desired_wheel_theta[2] = {0 , 0}; //rad
float theta[2];                     // rad
float prevTheta[2];     // rad
float angularVelocity[2];  // rad/s
float linearVelocity[2];    // m/s (= angular * radius)
// ---------------------------------------------------------------------

// Odometry 
float phi = 0;   // robot heading, rad
float x = 0;     // robot x position, m
float y = 0;     // robot y position, m
float odometry[3] = {0,0,0}; // x, y, and phi. X and Y are in meters and phi is in radians.
// ---------------------------------------------------------------------

// Motor control 
float angularVelocity_SP[2];   //angular velocity setpoint, rad/s
float voltage[2];             // set directly to bypass velocity control
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

  // Initialize I2C
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onReceive(receive);


  Serial.begin(115200);
  last_time_ms = millis();
  start_time_ms = last_time_ms;
  Serial.println("");
}

void loop() {
  current_time_ms = millis();

  // If there is data on the buffer, read it
  if (msgLength > 0) {
    printReceived();
    msgLength = 0;
  }

  if(instruction[0] != 7 && instruction[1] != 7){
    if(instruction[0] != currentDirection[0]){
      desired_wheel_theta[0] = PI * instruction[0];
    }

    if(instruction[1] != currentDirection[1]){
      desired_wheel_theta[1] = PI * instruction[1];
    }
  }


  digitalWrite(STBY_PIN, HIGH);

  
  noInterrupts();
  int localCountL = encoderCount[1];
  int localCountR = encoderCount[0];
  interrupts();

  theta[0] = (localCountR / fullRotation) * 2 * PI;
  theta[1] = (localCountL / fullRotation) * 2 * PI;

  if (current_time_ms - last_time_ms >= desired_Ts_ms) {

    angularVelocity[0] = 1000.0 * (theta[0] - prevTheta[0]) / (float)(current_time_ms - last_time_ms);
    angularVelocity[1] = 1000.0 * (theta[1] - prevTheta[1]) / (float)(current_time_ms - last_time_ms);
    prevTheta[0] = theta[0];
    prevTheta[1] = theta[1];

    angularVelocity_SP[0] = Ki_pos * (theta[0] + Kp_pos * (desired_wheel_theta[0] - theta[0]));
    angularVelocity_SP[1] = Ki_pos * (theta[1] + Kp_pos * (desired_wheel_theta[1] - theta[1]));
    voltage[0] = Kp_vel * (angularVelocity_SP[0] - angularVelocity[0]);
    voltage[1] = Kp_vel * (angularVelocity_SP[1] - angularVelocity[1]);

    for(int i = 0; i < 2; i++){
    if(voltage[i] >= 8.0){
      voltage[i] = 8;
    } else if(voltage[i] <= -8.0){
      voltage[i] = -8;
    }

    if (voltage > 0) {
      digitalWrite(SIGN_PIN[i], !i ? LOW : HIGH);
      analogWrite(PWM_PINs[i], (abs(voltage[i]) / 8.0 * 255));
    } else if (voltage < 0) {
      digitalWrite(SIGN_PIN[i], !i ? HIGH : LOW);
      analogWrite(PWM_PINs[i], (abs(voltage[i]) / 8.0 * 255));
    }
    else{
      analogWrite(PWM_PINs[i], 0);
    }

    }

  

      dt = last_time_ms - current_time_ms;



  
// ODOMETRY CODE NOT UPDATED FOR ARRAYS
      linearVelocity[1] = angularVelocity[1] * radius;
      linearVelocity[0] = angularVelocity[0] * radius;
      dt = (float)(current_time_ms - last_time_ms) / 1000.0;
      odometry[2] += ((linearVelocity[0] - linearVelocity[1]) / robotDiameter) * dt;
      odometry[0] += cos(odometry[2]) * (linearVelocity[1] + linearVelocity[0]) / 2 * dt;
      odometry[1] += sin(odometry[2]) * (linearVelocity[1] + linearVelocity[0]) / 2 * dt;

      Serial.print(current_time_ms);
      Serial.print(",");
      Serial.print(odometry[0]);
      Serial.print(",");
      Serial.print(odometry[1]);
      Serial.print(",");
      Serial.print(odometry[2]);
      Serial.print("\n");
     
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

      last_time_ms = current_time_ms;

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


void printReceived() {
  // Print on serial console
  Serial.print("Offset received: ");
  Serial.println(offset);
  Serial.print("Message Length: ");
  Serial.println(msgLength);
  Serial.print("Instruction received: ");

  for (int i=0;i<msgLength;i++) {
    Serial.print(String(desired_wheel_theta[i])+"\t");
  }
  Serial.println("");
}

// function called when an I2C interrupt event happens
void receive() {
  // Set the offset, this will always be the first byte.
  offset = Wire.read();
  // If there is information after the offset, it is telling us more about the command.
  while (Wire.available()) {
    instruction[msgLength] = Wire.read();
    msgLength++;
  }
}
