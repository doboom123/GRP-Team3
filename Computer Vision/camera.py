# EENG350 Mini Project - Computer Vision
# Ziero Padilla 

import cv2
from cv2 import aruco
import numpy as np
from time import sleep


#step 1: establishing camera parameters and deadband configs

TARGET_ID = 0
ARUCO_DICT =  cv2.aruco.DICT_6X6_50
CAMERA_INDEX = 0
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
DEADBAND_PX = 15


#LUT for the quadrants
QUADRANT_BITS = {
    "NE": (0,0),
    "NW": (0,1),
    "SW": (1,1),
    "SE": (1,0)
}

#step 2: building the aruco detector
def make_detector():
    d = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    p = cv2.aruco.DetectorParameters()
    det = cv2.aruco.ArucoDetector(d, p)
    def detect(gray):
        corners, ids, _ = det.detectMarkers(gray)
        return corners, ids
    return detect


#step 3: locating and isolating a single aruco marker
def find_target(corners, ids, target_id):
    if ids is None:
        print("Cannot find target marker.")
        return None
    ids = ids.flatten()
    for (outline, marker_id) in zip(corners, ids):
        if marker_id == target_id:
            markerCorners = outline.reshape((4, 2))
            return(np.mean(markerCorners, axis = 0))
    return None


#step 4: mapping marker positions to quadrants
def get_quadrant(center, frame_width, frame_height):
    #frame_width, frame_height = 640, 480
    cx, cy = center
    midlineVert = frame_width / 2
    midlineHorz = frame_height / 2
    if abs(cx - midlineVert) < DEADBAND_PX or abs(cy - midlineHorz) < DEADBAND_PX:
        return None
    ns = "N" if cy < midlineHorz else "S"
    ew = "E" if cx > midlineVert else "W"
    return ns + ew

#step 5: information packing using bitshifting 
def pack_goal(quadrant):
    left, right = QUADRANT_BITS[quadrant]
    return (left << 1 ) + right



    




    
