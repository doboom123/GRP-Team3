Demo1.ino: This is the working file for our Arduino code. To use it, hook up both motors to the Arduino, alongside the power shield and battery. From there, enter your desired angle turned(in degrees) and distance to move(in feet).

demo1 cv.py: This is the working file for our angle detection. To use it, attach the Pi to power and plug in a camera. From there, run the code and the LCD will display the angle to a detected ArUco marker 0.

colorDetect.py: This is a file containing the necessary functions needed to detect the left and right arrows as well as interpret their colour. This preliminary version DOES NOT return any control signals to send to the Arduino; rather, it simply returns a "LEFT" or "RIGHT" direction to be displayed on the LCD. The colorDetect functions will be imported into and called within the driver program. 
