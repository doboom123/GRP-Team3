import cv2
from cv2 import aruco

from smbus2 import SMBus

import numpy as np
from time import sleep

import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

import threading
import queue

q = queue.Queue()

FOV_DEGREES = 60

img_h = 320
img_w = 640
img_cX = img_w / 2

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

camera.set(cv2.CAP_PROP_FRAME_WIDTH, img_w) # set resulution
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, img_h)

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
        if ids[0] == 0:
            for corner, marker_id in zip(corners, ids):
                # Reshape corners to a 4x2 array (Top-Left, Top-Right, Bottom-Right, Bottom-Left)
                pts = corner.reshape((4, 2))

                # calculating center of marker
                tag_cX = np.mean(pts[:,0])

                pixel_offset = tag_cX - img_cX
                phi = (pixel_offset / (img_cX)) * (FOV_DEGREES / 2)
                print(f'angle to marker: {phi:.2f} deg')
        
    
camera.release()
cv2.destroyAllWindows()
