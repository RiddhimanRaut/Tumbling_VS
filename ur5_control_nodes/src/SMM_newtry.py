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
from scipy.special import *

c=0
rows = 20
cols = 20
d = 2

mat_index = np.array(list(product(range(rows), range(cols)))).reshape(rows,cols,1,2)
new_mat = mat_index.reshape(rows,cols,1,1,1,2)
l = np.tile(mat_index,(rows,cols,1,1,1,1))
x_minus_mu = np.subtract(l,new_mat)
transpose = x_minus_mu.reshape(rows,cols,rows,cols,2,1)

ident = np.identity(2)
l_indent = np.tile(ident,(rows,cols,1,1))
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
        global c,rows,cols,d,new_mat,transpose
        global mat_index,l_indent,l,x_minus_mu
        frame  = cv2.resize(frame,(rows,cols))
        greyscale = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) #intensity matrix
        greyscale[greyscale<2] = 2
        cov = greyscale.reshape(rows,cols,1,1)
        sigma = np.multiply(cov,l_indent)
        sigma[sigma==0] = 1
        determinant = np.linalg.det(sigma)  #determinant
        nu_num = 2*greyscale
        nu_denom = greyscale-1
        nu = nu_num/nu_denom
        nu = nu.reshape(rows,cols,1,1)
        t_num_mat = (nu+d)/2
        t_num_first = gamma(t_num_mat)
        t_num_second = 1/np.sqrt(determinant)
        t_num = t_num_first*t_num_second
        t_num = t_num.reshape(rows,cols,rows,cols,1,1)
        t_denom_first = (pi*nu)
        t_denom_second = gamma(nu/2)
        delta = np.matmul(x_minus_mu,sigma)
        delta = np.matmul(delta,transpose)
        # print delta.shape
        # delta = delta.reshape(rows,cols)
        t_denom_third = 1+np.divide(delta,nu)
        t_denom_third = np.power(t_denom_third,t_num_mat)
        t_denom = t_denom_first*t_denom_second*t_denom_third
        t_dist = np.divide(t_num,t_denom)
        t_dist = t_dist.reshape(rows,cols,rows,cols)
        # print t_dist
        smm = np.sum(t_dist,axis=(2,3))
        print smm
        cv2.imshow("SMM",smm)



        


        
        # print x_minus_mu
        
        
        
        cv2.imshow("Color Tracking",greyscale)
        cv2.waitKey(1)
        
        
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