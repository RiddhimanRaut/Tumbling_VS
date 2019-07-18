#!/usr/bin/env python
import cv2
import rospy
import freenect
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from collections import deque
from imutils.video import VideoStream
import imutils
import argparse
import time
from ur5_control_nodes.msg import floatList
from std_msgs.msg import String
bridge=CvBridge()
rgb_img=np.zeros((480,640,3))

def rgb_callback(msg):
    global rgb_img
    rgb_img=bridge.imgmsg_to_cv2(msg,"bgr8")
def xyz_pts(img)
   

def main():
    
    rospy.init_node('listener', anonymous=True)
    # rospy.Subscriber('/camera/depth/image_raw', Image, depth_callback)
    rospy.Subscriber('/camera/rgb/image_color',Image,rgb_callback)
    rospy.spin()
if __name__=="__main__":
    main()

       