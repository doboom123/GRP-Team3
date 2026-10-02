# Mini Project Computer Vision
#
# Ian Ferguson and Ziero Padilla
# 
# This code detects an ArUco marker using OpenCV and sends
# a 1-bit instruction 

import cv2
from cv2 import aruco

from smbus2 import SMBus

import numpy as np
from time import sleep

import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

import threading
import queue

#start the arduino bus
ARD_ADDR = 8
ard_i2c = SMBus(1)

q = queue.Queue()

# initializing some variables
oldLocation = 0
newLocation = 0

#initializing board
lcd_columns = 16
lcd_rows = 2
lcd_i2c = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(lcd_i2c, lcd_columns, lcd_rows)
lcd.clear()

# function handling all the lcd stuff
def printToLCD():
    while True:
        if not q.empty():
            instruction = q.get()
            print(instruction)
            if (instruction is not None):
                lcd.clear()
                lcd.message = f"{instruction}"
            else:
                lcd.message = "None"

# declaring and starting my thread to handle lcd processing
printThread = threading.Thread(target=printToLCD, args=())
printThread.start()

# center coordinates of the aruco marker
cX = 0
cY = 0

# importing aruco tags
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

camera = cv2.VideoCapture(0) # Initialize the camera

camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320) # shrinking resolution for smoother camera
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

width = camera.get(cv2.CAP_PROP_FRAME_WIDTH) # getting h/w for calc later
height = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)

if camera.isOpened():
    sleep(.5) # wait for image to stabilize
    print("Camera is ready")
    
while(True):
    ret,frame = camera.read() # Take an image
    if not ret:
        print("Failed to capture image")
        break

    key = cv2.waitKey(1) & 0xFF
    
    # Press 'q' to quit the program
    if key == ord('q'):
        break
    
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    # Make the image greyscale for ArUco detection
    cv2.imshow("overlay",grey)

    # hunt for aruco tags
    corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)
    if ids is not None:
        for corner, marker_id in zip(corners, ids):
            # Reshape corners to a 4x2 array (Top-Left, Top-Right, Bottom-Right, Bottom-Left)
            pts = corner.reshape((4, 2))
            (tl, tr, br, bl) = pts

            # calculating center of marker
            cX = int((tl[0] + br[0]) / 2)
            cY = int((tl[1] + br[1]) / 2)

        # check center for quadrant
        if(cX >= width/2):
            if(cY <= height/2):
                newLocation = 0b00 # NE
            else:
                newLocation = 0b11 # SE
        else:
            if(cY <= height/2):
                newLocation = 0b01 # NW
            else:
                newLocation = 0b10 # SW
    else:
        newLocation = None

    # checking for change
    if (newLocation != oldLocation):
        q.put(newLocation)
        command = newLocation
        if (command is not None):
            # Send to the arduino
            ard_i2c.write_byte(ARD_ADDR, command)
        oldLocation = newLocation
        
    
camera.release()
cv2.destroyAllWindows()
