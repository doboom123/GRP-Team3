#EENG350 Demo 2 CV: Color Detection 
# Ziero Padilla

import numpy as np
import cv2

#arrays for red and green
redLower = np.array([0, 100, 100]) #placeholder values for the bright reds until compared against better lighting 
redUpper = np.array([10, 255, 255])

#Green values from assignment 2 
greenLower = np.array([50, 160, 100])
greenUpper = np.array([79, 200, 200])

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
        if area > 300:
            cv2.drawContours(scene, [cnt], 0, (255,0,0), 3)
            return True
    return False

def arrowDirection(scene):
    isGreen = colorDetect(scene, greenLower, greenUpper)
    isRed = colorDetect(scene, redLower, redUpper)

    # if-else conditional to check for red/green arrows and turn based on what colour was detected
    if (isGreen):
        print("Green Arrow: LEFT")
        return "LEFT"
    elif(isRed):
        print("Red Arrow: RIGHT")
        return "RIGHT"
    else:
        print("No color detected in range")
        return None
    
