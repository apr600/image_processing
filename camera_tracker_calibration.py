### Python Imports ###
import cv2
import numpy as np
import math

### Ros Imports ###
import rospy
import rospkg
from cv_bridge import CvBridge

def nothing(x):
    ''' Random function that literally does nothing. Needed for opencv. '''
    pass

def set_threshold():
    # get the trackbar threshold values
    r = cv2.getTrackbarPos('Rmax','BGR')
    g = cv2.getTrackbarPos('Gmax','BGR')
    b = cv2.getTrackbarPos('Bmax','BGR')
    upper = (int(b), int(g), int(r))
    r = cv2.getTrackbarPos('Rmin','BGR')
    g = cv2.getTrackbarPos('Gmin','BGR')
    b = cv2.getTrackbarPos('Bmin','BGR')
    lower = (int(b), int(g), int(r))
    return upper, lower

# Read in webcam video image
cv2.namedWindow('Image') # make a window named image

frame = cv2.imread('test_img.png')

# Set filter thresholds
cv2.namedWindow('BGR')
# create trackbars
cv2.createTrackbar('Rmin','BGR',0,255,nothing)
cv2.createTrackbar('Rmax','BGR',0,255,nothing)
cv2.createTrackbar('Gmin','BGR',0,255,nothing)
cv2.createTrackbar('Gmax','BGR',0,255,nothing)
cv2.createTrackbar('Bmin','BGR',0,255,nothing)
cv2.createTrackbar('Bmax','BGR',0,255,nothing)

while True:

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # extract the hsv space
    # Filter for marker on biotac mount
    
    upper, lower = set_threshold()
    mask = cv2.inRange(hsv, lower, upper) # threshold the frame
    mask = cv2.erode(mask, None, iterations=2) # erode it
    mask = cv2.dilate(mask, None, iterations=2) # dilate it to find the contours
    mask = cv2.bitwise_not(mask)

    res = cv2.bitwise_and(frame,frame, mask= mask)
    cv2.imshow('Image', frame)
    cv2.imshow('BGR', res)

    k = cv2.waitKey(1) & 0xFF
    if k == ord('m'):
        # find the contours
        _, cnts, hierarchy  = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        print("num contours: ", len(cnts))

        for i in range(len(cnts)): # loop through the objects
            M = cv2.moments(cnts[i])
            ((x,y), radius) = cv2.minEnclosingCircle(cnts[i])
            center = (int(M["m10"] / M["m00"]), int( M["m01"] / M["m00"] ))
            cv2.circle(frame, (int(x), int(y)), int(radius), (128, 0, 128), 2)
            cv2.circle(frame, center, 2, (128,0,128), -1)

    if k == ord('q'):
        cv2.destroyAllWindows()
        break
