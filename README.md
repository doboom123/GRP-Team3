************************
** Group Three ReadMe **
************************


This is the repository for Group 3 in seed lab. 
## files

1. **Demo1**
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
3. **Simulink**
 ```bash
The Simulink folder contains matlab code and simulink models for simulating the motor control system and plotting our 
simulation against experimental data.
```
