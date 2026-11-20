import cv2
from cv2 import aruco

from time import sleep

import numpy as np

from smbus2 import SMBus
import struct

color = 0
direction = 0

# Red wraps around 0 and 179 in OpenCV HSV, so it requires two ranges
redLower1 = np.array([0, 80, 50])
redUpper1 = np.array([10, 255, 255])
redLower2 = np.array([170, 80, 50])
redUpper2 = np.array([179, 255, 255])

# Green HSV range (OpenCV Hue: 0-179)
greenLower = np.array([28, 45, 35])
greenUpper = np.array([92, 255, 255])

#function for red/green detect
def colorDetect(scene, lower, upper):
    hsv = cv2.cvtColor(scene, cv2.COLOR_BGR2HSV) #remember: any hsv values found online should be divided by 2 for your program
    #print(hsv[143,360])

    mask = cv2.inRange(hsv, lower, upper)

    # making use of the OPEN and CLOSE methods for image cleanup from A2, now used for real time arrow detection 
    kernel = np.ones((5,5), np.uint8)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

    # defining contours to draw for the arrows
    contours, _ = cv2.findContours(mask_clean, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # fix this line!!!
        if area > 300:
            cv2.drawContours(scene, [cnt], 0, (255,0,0), 3)
            return True
    return False

def arrowDirection(scene):
    isGreen = colorDetect(scene, greenLower, greenUpper)
    isRed = colorDetect(scene, redLower1, redUpper1)
    isRed2 = colorDetect(scene, redLower2, redUpper2)

    # if-else conditional to check for red/green arrows and turn based on what colour was detected
    if (isGreen):
        print("Green Arrow: LEFT")
        return 1
    elif(isRed or isRed2):
        print("Red Arrow: RIGHT")
        return 2
    else:
        print("No color detected in range")
        return 0
    

ARD_ADDR = 8
ard_i2c = SMBus(1)
sleep(0.1)

toSendArray = [0]*9

# Constants found during your manual calibration step
KNOWN_DISTANCE = 30.5  # e.g., 50 cm away during setup
KNOWN_WIDTH = 4.92125     # e.g., object is 15 cm wide in reality
CALIBRATED_FOCAL_LENGTH = 644.55 # Derived from (Pixel_Width * KNOWN_DISTANCE) / KNOWN_WIDTH

object_width_in_pixels = 100

img_w = 640
img_h = 480

FOV_DEGREES = 53

# Start video capture
camera = cv2.VideoCapture(0)

camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640) # set resolution
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

cam_mtx = np.array([[658.30651767, 0, 344.8388955],
                  [0, 656.24217357, 239.78748797],
                  [0, 0, 1]])

dist_coeff = np.array([0.154308352, -1.28270254, 0.00200018049, -0.000650158947, 2.52859272])

fX = cam_mtx[0,0]
img_cX = img_w/2

newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_mtx, dist_coeff, (img_w, img_h), 1, (img_w, img_h))

cX = 0
cY = 0

detected = 0
phi = 0

# importing aruco tags
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

if camera.isOpened():
    sleep(.5) # wait for image to stabilize
    print("Camera is ready")

while True:
    ret, frame = camera.read()
    
    # [Insert your object detection logic here to find object_width_in_pixels]
    # center coordinates of the aruco marker
    if not ret:
        print("Failed to capture image")
        break

    key = cv2.waitKey(1) & 0xFF
    
    # Press 'q' to quit the program
    if key == ord('q'):
        break

    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    dst = cv2.undistort(grey, cam_mtx, dist_coeff, None, newcameramtx)



   # cv2.imshow("overlay", frame)

    # hunt for aruco tags

    corners,ids,rejected = aruco.detectMarkers(dst,aruco_dict)
    if ids is not None:
        detected = 1
        if ids[0] == 0:
            for corner, marker_id in zip(corners, ids):
                
                
                # Reshape corners to a 4x2 array (Top-Left, Top-Right, Bottom-Right, Bottom-Left)
                pts = corner.reshape((4, 2))
                (tl, tr, br, bl) = pts

                y_coords = pts[:, 1]

                padding = 20  # Optional vertical margin in pixels

                y_min = max(0, int(np.floor(np.min(y_coords))) - padding)
                y_max = min(frame.shape[0], int(np.ceil(np.max(y_coords))) + padding)

                # 3. Slice full width (:) between y_min and y_max
                horizontal_slice = frame[y_min:y_max, :]
                cv2.imshow("overlay", horizontal_slice)

                direction = arrowDirection(horizontal_slice)

                tag_cX = np.mean(pts[:,0])

                pixel_offset = tag_cX - img_cX
                phi = -(pixel_offset / (img_cX)) * (FOV_DEGREES / 2)
                print(f'Angle to marker: {phi:.2f} deg')
                
                # Finding pixel width of detected aruco marker
                object_width_in_pixels = np.sqrt((tr[0] - tl[0])**2 + (tr[1] - tl[1])**2)
    else:
        detected = 0
        direction = 0
    
    # Calculate distance
    distance = (KNOWN_WIDTH * CALIBRATED_FOCAL_LENGTH) / object_width_in_pixels

    distance = round(distance, 1)
    phi = round(phi, 2)
    
    # making the sending array
    # toSendArray[8] = int(detected)
    toSendArray[0] = int(distance // 100)
    toSendArray[1] = int((distance % 100) // 10)
    toSendArray[2] = int((distance % 10) // 1)
    toSendArray[3] = int((distance % 1) // 0.1)
    if (phi < 0):
        toSendArray[4] = 0
        toSendArray[5] = int(-phi // 10)
        toSendArray[6] = int((-phi % 10) // 1)
        toSendArray[7] = int((-phi % 1) // 0.1)
    else:
        toSendArray[4] = 1
        toSendArray[5] = int(phi // 10)
        toSendArray[6] = int((phi % 10) // 1)
        toSendArray[7] = int((phi % 1) // 0.1)
    toSendArray[8] = direction


    print(toSendArray)
    
    try:
        ard_i2c.write_i2c_block_data(ARD_ADDR, 7, toSendArray)
    except:
        pass


    

camera.release()
cv2.destroyAllWindows()
