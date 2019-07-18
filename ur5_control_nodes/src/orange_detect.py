#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from ur5_control_nodes.msg import floatList
from cv_bridge import CvBridge, CvBridgeError
import imutils

bridge    = CvBridge()
rgb_img   = np.zeros((480,640,3),np.uint8)

def nothing(x):
    pass

#RGB Image Callback
def rgb_callback(msg):
    global rgb_img   
    rgb_img = bridge.imgmsg_to_cv2(msg, "bgr8")
     

def orange_detect(frame):
#     blurred  = cv2.GaussianBlur(frame, (11, 11), 0)
#     hsv      = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

#     cv2.namedWindow("orange_mask")
#     cv2.createTrackbar("h_low","orange_mask",0,179, nothing)
#     cv2.createTrackbar("s_low","orange_mask",0,255, nothing)
#     cv2.createTrackbar("v_low","orange_mask",0,255, nothing)
#     cv2.createTrackbar("h_high","orange_mask",0,179, nothing)
#     cv2.createTrackbar("s_high","orange_mask",0,255, nothing)
#     cv2.createTrackbar("v_high","orange_mask",0,255, nothing)
#     h_low = cv2.getTrackbarPos("h_low","orange_mask")
#     s_low = cv2.getTrackbarPos("s_low","orange_mask")
#     v_low = cv2.getTrackbarPos("v_low","orange_mask")
#     h_high = cv2.getTrackbarPos("h_high","orange_mask")
#     s_high = cv2.getTrackbarPos("s_high","orange_mask")
#     v_high = cv2.getTrackbarPos("v_high","orange_mask")


#     low_o  = np.array([h_low,s_low,v_low])
#     high_o = np.array([h_high,s_high,v_high])

#     orange    = cv2.inRange(hsv, low_o, high_o)
#     orange    = cv2.erode(orange, None, iterations=2)
#     orange    = cv2.dilate(orange, None, iterations=2)  

    cv2.imshow("Color Tracking",frame)
#     cv2.imshow("orange_mask",orange)
#     cv2.waitKey(1)

def main():
    global rgb_img
    rospy.init_node('d_points', anonymous=True)
    rospy.Subscriber('/camera/rgb/image_raw', Image, rgb_callback )
    
    orange_detect(rgb_img)

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass