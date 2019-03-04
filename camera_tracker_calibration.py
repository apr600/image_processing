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

class Camera_Tracker_Calibration:
    def __init__(self, image_filename):

        # Read in image
        self.img_fname = image_filename
        
        cv2.namedWindow('Image') # make a window named image

        # Set filter thresholds
        cv2.namedWindow('BGR')
        # create trackbars
        cv2.createTrackbar('Rmin','BGR',0,255,nothing)
        cv2.createTrackbar('Rmax','BGR',0,255,nothing)
        cv2.createTrackbar('Gmin','BGR',0,255,nothing)
        cv2.createTrackbar('Gmax','BGR',0,255,nothing)
        cv2.createTrackbar('Bmin','BGR',0,255,nothing)
        cv2.createTrackbar('Bmax','BGR',0,255,nothing)

    def set_threshold(self):
        # get the trackbar threshold values
        r = cv2.getTrackbarPos('Rmax','BGR')
        g = cv2.getTrackbarPos('Gmax','BGR')
        b = cv2.getTrackbarPos('Bmax','BGR')
        self.upper = (int(b), int(g), int(r))
        r = cv2.getTrackbarPos('Rmin','BGR')
        g = cv2.getTrackbarPos('Gmin','BGR')
        b = cv2.getTrackbarPos('Bmin','BGR')
        self.lower = (int(b), int(g), int(r))

    def filter_image(self):
        self.set_threshold()
        mask = cv2.inRange(self.hsv, self.lower, self.upper) # threshold the frame
        mask = cv2.erode(mask, None, iterations=2) # erode it
        mask = cv2.dilate(mask, None, iterations=2) # dilate it to find the contours
        self.mask = cv2.bitwise_not(mask)
        self.res = cv2.bitwise_and(self.frame,self.frame, mask= self.mask)

    def find_object(self):
        _, self.cnts, hierarchy  = cv2.findContours(self.mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        print("num contours: ", len(self.cnts))
        for i in range(len(self.cnts)): # loop through the objects
            M = cv2.moments(self.cnts[i])
            ((x,y), radius) = cv2.minEnclosingCircle(self.cnts[i])
            self.center = (int(M["m10"] / M["m00"]), int( M["m01"] / M["m00"] ))
            cv2.circle(self.frame, (int(x), int(y)), int(radius), (128, 0, 128), 2)
            cv2.circle(self.frame, self.center, 2, (128,0,128), -1)

    def track_object(self):
        self.frame = cv2.imread(self.img_fname)
        while True:
            self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV) # extract the hsv space
            # Filter for marker on biotac mount

            self.filter_image()
            cv2.imshow('Image', self.frame)
            cv2.imshow('BGR', self.res)

            k = cv2.waitKey(1) & 0xFF
            if k == ord('m'):
                # find the contours
                self.find_object()

            if k == ord('q'):
                cv2.destroyAllWindows()
                break

if __name__ == '__main__':
    ctc = Camera_Tracker_Calibration("test_img.png")
    ctc.track_object()
