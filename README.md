************************
** Group Three ReadMe **
************************


This is the repository for Group 3 in seed lab. 
## files

1. **MiniProj1_23SEP26**
 ```bash
The Arduino and Raspberry Pi code for the first Mini Project can be found in MiniProj1_23SEP26. This project contains files to allow
communication between an Arduino and a Raspberry Pi and to control two motors attached to the Arduino. The code initiates 
movement in wheels controlled by the Arduino corresponding to the location of an ArUco marker in the view of a camera 
connected to the PI. To use, run the Arduino and Pi codes on their respective devices and attach the necessary wires for I2C communication. Hook up the camera and aim it at an ArUco marker.
 ```
2. **Simulink**
 ```bash
The Simulink folder contains matlab code and simulink models for simulating the motor control system and plotting our 
simulation against experimental data.
```
3. **Combined_motor_odometry**
```bash
Combined_motor_odometry is the code for two B we have it here so that we could reference it while doing the mini-project. The 
Animation folder is the MATLAB animation to map how the robot would move. 
```
4. **Animation**
```bash
The Animation folder holds all of the files that are used to run the MATLAB animation that simulates how the robot would be moving in real time
```
5. **Computer Vision**
```bash
The CV folder contains the necessary Python / C++ files that are used to implement real-time ArUco marker detection and establish I2C protocol between the Arduino and Raspberry Pi
```
