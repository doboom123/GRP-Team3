# Demo 1 Computer Vision
# Ian Ferguson and Ziero Padilla
# Group 3
#
# This code utilizes a calibrated cmarea in order to detect the angle of an ArUco marker.

import cv2
from cv2 import aruco

from smbus2 import SMBus

import numpy as np
from time import sleep

import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

import threading
import queue

import math

q = queue.Queue()

# initialize variables
phi = 0

FOV_DEGREES = 53

img_h = 480
img_w = 640

camera = cv2.VideoCapture(0) # Initialize the camera

camera.set(cv2.CAP_PROP_FRAME_WIDTH, img_w) # set resolution
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, img_h)

# camera calibration matrices
cam_mtx = np.array([[658.30651767, 0, 344.8388955],
                  [0, 656.24217357, 239.78748797],
                  [0, 0, 1]])

dist_coeff = np.array([0.154308352, -1.28270254, 0.00200018049, -0.000650158947, 2.52859272])

# defining focal length of the camera
fX = cam_mtx[0,0]
img_cX = img_w/2

# redefining our centers using an optimized matrix
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx, dist_coeff, (img_w, img_h), 1, (img_w, img_h))
x, y, w_roi, h_roi = roi
new_fX = newcameramtx[0,0]
new_cX = newcameramtx[0,2]-x

oldInstruction = "1"
instruction = "15"

#initializing board
lcd_columns = 16
lcd_rows = 2
lcd_i2c = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(lcd_i2c, lcd_columns, lcd_rows)
lcd.clear()

lcd.color = [100, 0, 100]

# function handling all the lcd stuff
def printToLCD():
    global oldInstruction
    global instruction
    while True:
        if not q.empty():
            instruction = q.get()
            if (len(instruction) != len(oldInstruction)):
                lcd.clear()
            lcd.message = f"{instruction}"
            oldInstruction = instruction

# declaring and starting my thread to handle lcd processing
printThread = threading.Thread(target=printToLCD, args=())
printThread.start()

# center coordinates of the aruco marker
cX = 0
cY = 0

# importing aruco tags
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)



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

    # setting to greyscale and undistorting
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    dst = cv2.undistort(grey, cam_mtx, dist_coeff)

    # cropping black edges out
    dst = dst[y:y+h_roi, x:x+w_roi]
    
    cv2.imshow("overlay",dst)

    # hunt for aruco tags
    corners,ids,rejected = aruco.detectMarkers(dst,aruco_dict)
    if ids is not None:
        if ids[0] == 0:
            for corner, marker_id in zip(corners, ids):
                # Reshape corners to a 4x2 array (Top-Left, Top-Right, Bottom-Right, Bottom-Left)
                pts = corner.reshape((4, 2))

                # calculating center of marker
                tag_cX = np.mean(pts[:,0])

                pixel_offset = tag_cX - new_cX
                phi = -(pixel_offset / (new_cX)) * (FOV_DEGREES / 2)
                print(f'Angle to marker: {phi:.2f} deg')
                
        if (phi != 0):
            if (q.empty()):
                q.put(f"Angle:\n{phi:.2f}")
    else:
        if (q.empty()):
            q.put("None detected")
        
    
camera.release()
cv2.destroyAllWindows()
