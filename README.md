************************
** Group Three ReadMe **
************************


This is the repository for Group 3 in seed lab. 
## files

1. **Demo2**
 ```bash
The Demo2.ino is all the aruduino code. In that file, we implemented a state machine and Pi communication together to create a functioning robot.
We take in information from the Pi and use said data to control our state machine and give us necessary distances and angles.
colorDetect.py contains the logic that detects color which was used in demo2cvFull. demo2cvFull.py implements color detection, distance detection,
and aruco identification together and contains the code to communicate to the Arduino
 ```

2. **Demo1**
 ```bash
The Demo1.ino is all the arudiono code. In that file, we had Voltage trimming to make the motors
spin at the same rate. We also used a PID controller for the individual wheels to make the robot move
to desired positions. In the code, there is also a state machine that ensures the robot is only doing one thing at a time.
 ```

3. **MiniProj1_23SEP26**
 ```bash
The Arduino and Raspberry Pi code for the first Mini Project can be found in MiniProj1_23SEP26. This project contains files to allow
communication between an Arduino and a Raspberry Pi and to control two motors attached to the Arduino. The code initiates 
movement in wheels controlled by the Arduino corresponding to the location of an ArUco marker in the view of a camera 
connected to the PI. To use, run the Arduino and Pi codes on their respective devices and attach the necessary wires for I2C communication. Hook up the camera and aim it at an ArUco marker.
```


4. **colorDetect**
 ```bash
The colorDetect functions are designed to detect arrow colours and pass them to a function called arrowDetect. Though the functions were designed to be modularly implemented using traditional Python import methods, they were hard-coded into the driver program to be better integrated with the state machine.
 ```

5. **Simulink**
 ```bash
The Simulink folder contains matlab code and simulink models for simulating the motor control system and plotting our 
simulation against experimental data.
```
SSH seedlab@raspberrypi
Pass: team3
File: demo2colorderrick
