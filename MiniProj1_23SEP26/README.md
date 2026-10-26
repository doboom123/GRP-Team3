MINI PROJECT 1 README

This project uses a Raspberry Pi and an Arduino-controlled, two-wheeled robot to identify the location of an 
ArUco marker within a camera's field of view (FOV) and display its quadrant as a binary value using the 
robot's wheels.

The camera's FOV is divided into four quadrants, numbered 1 through 4. The Raspberry Pi processes the video 
input, identifies the quadrant containing the ArUco marker, and communicates the corresponding instruction to 
the Arduino via I2C.


Hardware
Raspberry Pi: Processes camera input and determines the ArUco marker's quadrant.
Arduino: Receives instructions over I2C at address 8 and controls wheel positioning.
DC Motors: Two independently controlled motors with encoders.







