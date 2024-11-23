#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

bridge = CvBridge()

color_ranges = {'green': [np.array((50,63,0),np.uint8),np.array((88,255,255),np.uint8)],
                'red': [np.array([0, 84, 84]), np.array([17, 255, 255])],
                'yellow': [np.array([16, 100, 99]), np.array([44, 255, 255])],
                'blue': [np.array([98, 111, 91]), np.array([154, 255, 255])]}


def image_callback(data):
    #rospy.loginfo("I heard %s", data)
    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    #gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    #__, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
    #contours, __ = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #for c in contours:
    #    area = cv2.contourArea(c)
    #    if area > 5000:
    #        x,y,w,h = cv2.boundingRect(c)
    #        cv2.rectangle(cv_image, (x,y), (x+w,y+h), (0,255,0), 4)
    #cv2.imshow('123', cv_image)

    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    for color, (lower, upper) in color_ranges.items():
        mask = cv2.inRange(hsv,np.array(lower),np.array(upper))
        contours, __ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for cont in contours:
            area = cv2.contourArea(cont)
            if area > 4000:
                #cv2.drawContours(frame,[cont],0,(0,255,0),4)
                x,y,w,h = cv2.boundingRect(cont)
                #midy, midx = y+h/2, x+w/2
                #center = frame[midy, midx]
                #b, g, r = center
                cv2.rectangle(frame, (x,y),(x+w,y+h),(0,255,0),4)
                centerx = x + w//2 - 50
                centery = y + h//2
                cv2.putText(frame, color, (centerx,centery), cv2.FONT_HERSHEY_SIMPLEX, 1, (128,255,128), 1, cv2.LINE_AA)
    cv2.imshow("Video",frame)
    cv2.waitKey(1)

rospy.init_node('cam_listener')
rospy.Subscriber('image', Image, image_callback, queue_size=1)
rospy.spin()


