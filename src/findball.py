#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

imgBridge = CvBridge()

def image_callback(sensorImage):
    ## convert sensor image to opencv image type
    global imgBridge
    try:
        cv_image = imgBridge.imgmsg_to_cv2(sensorImage, "bgr8")
    except CvBridgeError as be:
        print(be)
    
    ## filter color in range
    hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    binaryImg = cv2.inRange(hsv_img, (0, 70, 50), (20, 255, 255))
    cv2.imshow("binary image", binaryImg)

    ## tracking ball with circle
    _, contours, hierarchy = cv2.findContours(binaryImg.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if (area > 3000):
            M = cv2.moments(c)
            cx = -1
            cy = -1
            if (M['m00']!=0):
                cx= int(M['m10']/M['m00'])
                cy= int(M['m01']/M['m00'])
            cv2.circle(cv_image, (cx, cy), (int)(radius), (0, 0, 255), 1)
    cv2.imshow("ball tracking", cv_image)
    cv2.waitKey(30)


def main(args):
    rospy.init_node("tennis_ball_tracking", anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shuting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
