#include <Wire.h>
#define MY_ADDR 8
#define PI 3.14

// 09/25/2026
// Derrick Scott Joao Vitor Peclat Fayad Bode Lamb
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
const float radius = 0.0762;      // wheel radius, meters
const float fullRotation = 3200;    // encoder counts per full wheel rotation
const float robotDiameter = 0.3556;  // meters, distance between wheels
// ---------------------------------------------------------------------

// Position / velocity (angular, per-wheel)
volatile long encoderCount[2] = {0, 0};
float desired_wheel_theta[2]  = {0, 0};        // rad
float theta[2]                = {0, 0};        // rad
float prevTheta[2]            = {0, 0};        // rad
float angularVelocity[2]      = {0, 0};        // rad/s
float linearVelocity[2]       = {0, 0};        // m/s (= angular * radius)
float thetaError[2]           = {0, 0};        // change in theta
float accumTheta[2]           = {0, 0};        // the acumulated change in theta
// ---------------------------------------------------------------------

// Odometry 
float phi                     = 0;             // robot heading, rad
float x                       = 0;             // robot x position, m
float y                       = 0;             // robot y position, m
float odometry[3]             = {0,0,0};       // x, y, and phi. X and Y are in meters and phi is in radians.
float phiVelocity             = 0;             //Rad/s
float rVector                 = 0;             // Meters;

// ---------------------------------------------------------------------

// control 
float angularVelocity_SP[2];                   //angular velocity setpoint, rad/s
float voltage[2];                              // set directly to bypass velocity control
float DC_gain                 = 0.25;
const float Ki_pos            = .8;            //integral gain
const float Kp_pos            = 15;            //proportional gain positoin
const float Kp_vel            = 2;
float desiredPhi              = 0;             //Radians
float desiredDistance         = 0;             // Meters
float phiError                = 0;             // radians
float robotLinearVelocity     = 0;
// ---------------------------------------------------------------------




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

  // Initialize I2C
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onReceive(receive);

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


  // The if condition below runs once every 20ms
  if (current_time_ms - last_time_ms >= desired_Ts_ms) {
    last_time_ms = current_time_ms;
    //define dt, our sample time
    dt = (float)(current_time_ms - last_time_ms) / 1000.0;

      // updates the encoders and makes sure it's not interuptted during
    noInterrupts();
    int localCountR = encoderCount[0];
    int localCountL = encoderCount[1];
    interrupts();


    theta[0] = ((float)localCountR / (float)fullRotation) * 2 * PI;
    theta[1] = ((float)localCountL / (float)fullRotation) * 2 * PI;
    //calculate the fwd velocity of the robot
    robotLinearVelocity = (linearVelocity[0] + linearVelocity[1]) / 2.0;
    
    // calculates phi and phiVelocity
    phiVelocity = ((linearVelocity[0] - linearVelocity[1])/ robotDiameter) * dt;

    phi = ((theta[0] - theta[1]) / 2);
      //calculates x
    odometry[0] += cos(phi) * (robotLinearVelocity* dt);
      //calculates y
    odometry[1] += sin(phi) * (robotLinearVelocity * dt);

      Serial.print("Phi:");
      Serial.print("\t");
      Serial.print(phi);
      Serial.print("\t");
      Serial.print("desired theta");
      Serial.print("\t");
      Serial.print(desired_wheel_theta[0]);
      Serial.print("\t");
      Serial.print(desired_wheel_theta[1]);
      Serial.print("\t");
      Serial.print("Thetas");
      Serial.print("\t");
      Serial.print(theta[0]);
      Serial.print("\t");
      Serial.print(theta[1]);
      Serial.print("\n");

    
    for(int i = 0; i < 2; i++){

      // calculates the current velocity
      angularVelocity[i] = 1000.0 * (theta[i] - prevTheta[i]) / (float)(current_time_ms - last_time_ms);
      // calculates linear velocity
      linearVelocity[i] = angularVelocity[i] * radius;

      //this calculates the error  in theta
      thetaError[i] = desired_wheel_theta[i] - theta[i];
      // calculates the accumulated error in theta
      accumTheta[i] += thetaError[i] * desired_Ts_ms;
      // this uses our gain to stabilize the angular velocity to something that we want
      angularVelocity_SP[i] = Ki_pos * accumTheta[i] + (Kp_pos * (thetaError[i]));
      // calculates the required voltage to spin the wheel at the right speed
      voltage[i] = Kp_vel * (angularVelocity_SP[i] - angularVelocity[i]);

      // Calls voltage direction, aligns positve voltage with fwd movement, negative voltage with reverse,
      // and writes the correct value to the PWM PINS
      voltageDirection(voltage[i], SIGN_PIN[i], PWM_PINs[i], i);

      //Clamps the voltage
      clamp(voltage[i], accumTheta[i], thetaError[i], dt);

/*
      if (voltage[i] > 0) {
        digitalWrite(SIGN_PIN[i], !i ? LOW : HIGH);
        analogWrite(PWM_PINs[i], (abs(voltage[i]) / 8.0 * 255));
      } else if (voltage[i] < 0) {
        digitalWrite(SIGN_PIN[i], !i ? HIGH : LOW);
        analogWrite(PWM_PINs[i], (abs(voltage[i]) / 8.0 * 255));
      } else{
        analogWrite(PWM_PINs[i], 0);
      }
*/
      desired_wheel_theta[i] = !i ? desiredPhi*(robotDiameter / (2 * radius)) : -desiredPhi*(robotDiameter / (2 * radius));

      //sets previous theta to current theta after we dont need it
      prevTheta[i] = theta[i];


      } //end of for loop

    phiError = desiredPhi - phi;

    desiredPhi = -1.5;

    // Updates last time



  } //end of if timing loop


    

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


// printReceived helps us see what data we are getting from the leader
void printReceived() {
// Print on serial console
Serial.print("Offset received: ");
Serial.println(offset);
Serial.print("Message Length: ");
Serial.println(msgLength);
Serial.print("Instruction received: ");
for (int i=0;i<msgLength;i++) {
Serial.print(String(instruction[i])+"\t");
}
Serial.println("");
}

// function called when an I2C interrupt event happens
void receive() {
  msgLength = 0;
// Set the offset, this will always be the first byte.
// If there is information after the offset, it is telling us more about the command.
while (Wire.available()) {
instruction[msgLength] = Wire.read();
msgLength++;
}
}
