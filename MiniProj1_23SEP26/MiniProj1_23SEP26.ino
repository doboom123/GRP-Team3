#include <Wire.h>
#define MY_ADDR 8
#define PI 3.14

// 09/25/2026
// Derrick Scott Joao Vitor Peclat Fayad
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
const float radius = 0.0762;      // wheel radius, meters
const float fullRotation = 3200;    // encoder counts per full wheel rotation
const float robotDiameter = 0.3556;  // meters, distance between wheels
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
float phi = 0;   // robot heading, rad
float x = 0;     // robot x position, m
float y = 0;     // robot y position, m
float odometry[3] = {0,0,0}; // x, y, and phi. X and Y are in meters and phi is in radians.
// ---------------------------------------------------------------------

// Motor control 
float angularVelocity_SP[2];   //angular velocity setpoint, rad/s
float voltage[2];             // set directly to bypass velocity control
float DC_gain = 0.25;
const float Ki_pos = .8; //integral gain
const float Kp_pos = 15; //proportional gain
const float Kp_vel = 2;
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

// checks to see if a message came in and prints it out if so
 if (msgLength > 0) {
  if (offset==1) {
    digitalWrite(LED_BUILTIN,instruction[0]);
  }
  printReceived();
  msgLength = 0;
  }


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
    // it receives the instructions and updates the wheel current position by 180 degrees depending on the instruction
    
    //Loops through the directions and switches the desired theta depending on the wanted position.
    switch(instruction[0]){
      case 0:
        desired_wheel_theta[0] = 0;
        desired_wheel_theta[1] = 0;
        break;
      case 1:
        desired_wheel_theta[0] = PI;
        desired_wheel_theta[1] = 0;
        break;
      case 2:
        desired_wheel_theta[0] = PI;
        desired_wheel_theta[1] = PI;
        break;
      case 3:
        desired_wheel_theta[0] = 0;
        desired_wheel_theta[1] = PI;
        break;
    }

  
  // this for loop runs twice to make sure it does the same calculations for both wheels.
    for(int i = 0; i < 2; i++){
    
    //this calculates the error  in theta
    deltaTheta[i] = desired_wheel_theta[i] - theta[i];
    // calculates the accumulated error in theta
    accumTheta[i] += deltaTheta[i] * desired_Ts_ms;

    // calculates the current velocity
    angularVelocity[i] = 1000.0 * (theta[i] - prevTheta[i]) / (float)(current_time_ms - last_time_ms);
    //sets previous theta to current theta after we dont need it
    prevTheta[i] = theta[i];
    // this uses our gain to stabilize the angular velocity to something that we want
    angularVelocity_SP[i] = Ki_pos * accumTheta[i] + (Kp_pos * (deltaTheta[i]));
    // calculates the required voltage to spin the wheel at the right speed
    voltage[i] = Kp_vel * (angularVelocity_SP[i] - angularVelocity[i]);

    
    // this calculates if we need positive or negative voltage to get to the required postion
    if(voltage[i] >= 8.0){
      voltage[i] = 8;
      accumTheta[i] -= deltaTheta[i] * desired_Ts_ms;
    } else if(voltage[i] <= -8.0){
      voltage[i] = -8;
      accumTheta[i] -= deltaTheta[i] * desired_Ts_ms;
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




  
// ODOMETRY CODE NOT UPDATED FOR ARRAYS
      // calculates linear velocity
      linearVelocity[1] = angularVelocity[1] * radius;
      linearVelocity[0] = angularVelocity[0] * radius;
      // calculates change in time
      dt = (float)(current_time_ms - last_time_ms) / 1000.0;
      // calculates phi
      odometry[2] += ((linearVelocity[0] - linearVelocity[1]) / robotDiameter) * dt;
      //calculates x
      odometry[0] += cos(odometry[2]) * (linearVelocity[1] + linearVelocity[0]) / 2 * dt;
      //calculates y
      odometry[1] += sin(odometry[2]) * (linearVelocity[1] + linearVelocity[0]) / 2 * dt;

    
     
      // Updates last time
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
