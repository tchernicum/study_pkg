#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image

bridge = CvBridge()

def image_callback(data):
    cv_image = 1
    #rospy.loginfo("I heard %s", data)
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    __, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
    contours, __ = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        area = cv2.contourArea(c)
        if area > 5000:
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(cv_image, (x,y), (x+w,y+h), (0,255,0), 4)

    cv2.imshow('123', cv_image)
    cv2.waitKey(1)

rospy.init_node('cam_listener')
rospy.Subscriber('image', Image, image_callback, queue_size=1)
rospy.spin()
