#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from ur5_control_nodes.msg import floatList  #importing custom msg type created in /src/msg used for custom topic "four_point_features"
from cv_bridge import CvBridge, CvBridgeError
import imutils 
import stat_funcs
from itertools import *
from math import *
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

c=0
rows = 5
cols = 5
d = 2
mat_index = np.array(list(product(range(rows), range(cols)))).reshape(rows,cols,1,2)
bridge    = CvBridge()
rgb_img   = np.zeros((480,640,3),np.uint8)
depth_img = np.zeros((480,640))

def rgb_callback(rgb_msg):  
    global rgb_img   
    rgb_img = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")

def d_callback(msg):
    global depth_img
    depth_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

def SMM(frame):
        global c,rows,cols,d
        global mat_index
        
        
        frame  = cv2.resize(frame,(rows,cols))
        greyscale = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        
        cv2.imshow("Color Tracking",greyscale)
        cv2.waitKey(1)
        SMM = np.zeros((rows,cols))
        count = 0
        for i in range(len(greyscale)):
            for j in range(len(greyscale[i])):
                count+=1
                intensity = greyscale[i][j]
                if intensity <2:
                    intensity = 2
                # mu = np.array([i,j])
                # x_minus_mu = mat_index - mu
                # # print x_minus_mu.shape
                cov = np.array([ [intensity,1],[1,intensity] ])
                cov_det = np.linalg.det(cov)
                # nu = 2.0 * intensity / (intensity - 1)
                # inverse = x_minus_mu.reshape(rows,cols,2,1)
                # prod = np.matmul(x_minus_mu,cov)
                # delta = np.matmul(prod,inverse)
                # delta = delta.reshape(rows,cols)
                # t_denom = (( pi * nu)**d/2 ) * gamma(nu/2)*(1+delta/nu)**((nu+d)/2.0)
                # t_num = gamma((nu+d)/2.0)/sqrt(cov_det)
                # t_dist = t_num/t_denom
                # print t_dist.shape
                # t_dist_sum = t_dist.sum()
                # print t_denom
                # SMM[i][j] = t_dist_sum
                # print count
        c+=1
        # cv2.imshow("SMM",SMM)
        print c
        # return SMM
        


def main():
    global rgb_img
    global depth_img
    rospy.init_node('d_points', anonymous=True)
    rospy.Subscriber('/camera/rgb/image_raw', Image, rgb_callback )
    rospy.Subscriber('/camera/depth/image_raw', Image, d_callback)
    while not rospy.is_shutdown():
        print SMM(rgb_img)

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass