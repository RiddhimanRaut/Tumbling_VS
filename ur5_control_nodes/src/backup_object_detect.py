#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from ur5_control_nodes.msg import floatList
from cv_bridge import CvBridge, CvBridgeError
import imutils

#camshift trial:
r,h,c,w=250,90,400,125
track_window = (c,r,w,h)

ridge    = CvBridge()
rgb_img   = npb.zeros((480,640,3),np.uint8)
depth_img = np.zeros((480,640))
def nothing(x):
    pass

# cv2.namedWindow("orange_mask")
# cv2.createTrackbar("h_low","orange_mask",0,179, nothing)
# cv2.createTrackbar("s_low","orange_mask",0,255, nothing)
# cv2.createTrackbar("v_low","orange_mask",0,255, nothing)

# cv2.createTrackbar("h_high","orange_mask",0,179, nothing)
# cv2.createTrackbar("s_high","orange_mask",0,255, nothing)
# cv2.createTrackbar("v_high","orange_mask",0,255, nothing)
# h_high = cv2.getTrackbarPos("h_high","orange_mask")
# s_high = cv2.getTrackbarPos("s_high","orange_mask")
# v_high = cv2.getTrackbarPos("v_high","orange_mask")


#RGB Image Callback
def rgb_callback(rgb_msg):
    global rgb_img   
    rgb_img = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")    
     


#Depth Image Callback 
def d_callback(msg):
    
    global depth_img
    depth_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    
#Getting XY Points
def xy_points(frame):
    
    global h_low,h_high,s_low,s_high,v_low,v_high,c,r,w,h,track_window 
    ret,frame = frame

    xypoints = np.array([0,0,0,0,0,0,0,0], dtype = np.int64)

    blurred  = cv2.GaussianBlur(frame, (11, 11), 0)
    roi = blurred[r:r+h, c:c+w]
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
     
    hsv      = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
   
    #defining the Range of Blue color
    blue_lower = np.array([87,100,150],np.uint8)
    blue_upper = np.array([110,255,255],np.uint8)

    mask = cv2.inRange(hsv_roi,blue_lower,blue_upper)

    roi_hist = cv2.calcHist([hsv_roi],[0],mask,[180],[0,180])
    cv2.normalize(roi_hist,roi_hist,0,255,cv2.NORM_MINMAX)

    term_crit = ( cv2.TermCriteria_EPS | cv2.TermCriteria_COUNT, 10,1 ) 

    while (1):
        ret,frame = frame
        if ret == True:
            
    
    blue = cv2.inRange(hsv,blue_lower,blue_upper)
    blue = cv2.erode(blue, None, iterations=2)
    blue = cv2.dilate(blue, None, iterations=2)
    
    cnts_b   = cv2.findContours(blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_b   = imutils.grab_contours(cnts_b)
    center_b = None
    if len(cnts_b) > 0:
        c_b = max(cnts_b, key = cv2.contourArea)
        M_b = cv2.moments(c_b)
        center_b = (int(M_b["m10"] / M_b["m00"]), int(M_b["m01"] / M_b["m00"]))
        cv2.circle(frame, center_b, 5, (0,0,0), -1)
        xypoints[2] = center_b[0]
        xypoints[3] = center_b[1]
    #Red
    low_r  = np.array([140,150,0],np.uint8)
    high_r = np.array([180,255,255],np.uint8)

    red      = cv2.inRange(hsv,low_r,high_r)
    red      = cv2.erode(red, None, iterations=2)
    red      = cv2.dilate(red, None, iterations=2)
    cnts_r   = cv2.findContours(red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_r   = imutils.grab_contours(cnts_r)
    center_r = None
    if len(cnts_r) > 0:
        c_r = max(cnts_r, key=cv2.contourArea)
        M_r = cv2.moments(c_r)
        center_r = (int(M_r["m10"] / M_r["m00"]), int(M_r["m01"] / M_r["m00"]))
        cv2.circle(frame, center_r, 5, (0,0,0), -1)
        xypoints[4] = center_r[0]
        xypoints[5] = center_r[1]
    #Green
    low_g  = np.array([40,50,50])
    high_g = np.array([80,255,255])
    #GREEN_DETECTION
    #compute mask, erode and dilate it to remove noise
    green    = cv2.inRange(hsv, low_g, high_g)
    green    = cv2.erode(green, None, iterations=2)
    green    = cv2.dilate(green, None, iterations=2)
    cnts_g   = cv2.findContours(green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_g   = imutils.grab_contours(cnts_g)
    center_g = None
    if len(cnts_g) > 0:
        c_g = max(cnts_g, key=cv2.contourArea)
        # epsilon = 0.1*cv2.arcLength(c,True)
        # approx = cv2.approxPolyDP(c,epsilon,True)
        M_g = cv2.moments(c_g)
        center_g = (int(M_g["m10"] / M_g["m00"]), int(M_g["m01"] / M_g["m00"]))
        cv2.circle(frame, center_g, 5, (0,0,0), -1)
        xypoints[0] = center_g[0]
        xypoints[1] = center_g[1]
        cv2.circle(frame, (320,240), 5, (0,0,255), -1)
    #ORANGE
    # h_low = cv2.getTrackbarPos("h_low","orange_mask")
    # s_low = cv2.getTrackbarPos("s_low","orange_mask")
    # v_low = cv2.getTrackbarPos("v_low","orange_mask")

    # h_high = cv2.getTrackbarPos("h_high","orange_mask")
    # s_high = cv2.getTrackbarPos("s_high","orange_mask")
    # v_high = cv2.getTrackbarPos("v_high","orange_mask")
    
    

    
    low_o  = np.array([3,11,119])
    high_o = np.array([11,255,255])
    #GREEN_DETECTION
    #compute mask, erode and dilate it to remove noise
    orange    = cv2.inRange(hsv, low_o, high_o)
    orange    = cv2.erode(orange, None, iterations=2)
    orange    = cv2.dilate(orange, None, iterations=2)
    cnts_o   = cv2.findContours(orange.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts_o   = imutils.grab_contours(cnts_o)
    center_o = None
    if len(cnts_o) > 0:
        c_o = max(cnts_o, key=cv2.contourArea)
        # epsilon = 0.1*cv2.arcLength(c,True)
        # approx = cv2.approxPolyDP(c,epsilon,True)
        M_o = cv2.moments(c_o)
        center_o = (int(M_o["m10"] / M_o["m00"]), int(M_o["m01"] / M_o["m00"]))
        cv2.circle(frame, center_o, 5, (0,0,0), -1)
        xypoints[6] = center_o[0]
        xypoints[7] = center_o[1]
        
    cv2.imshow("Color Tracking",frame)
    # cv2.imshow("orange_mask",orange)
    cv2.waitKey(1)
    
    return xypoints

    
def f_points(dimg, xy): 
    fl = 531.1   #525
    zr = dimg[xy[5]][xy[4]]
    zb = dimg[xy[3]][xy[2]]    
    zg = dimg[xy[1]][xy[0]]
    zo = dimg[xy[7]][xy[6]]

    fpoint = [xy[0], xy[1], zr, xy[2], xy[3], zb, xy[4], xy[5],zg, xy[6], xy[7],zo]  
    for i in range(0,len(fpoint),3):    
        fpoint[i] = -(fpoint[i]-320)/(fl/640)
    for j in range(1,len(fpoint),3):
        fpoint[j] = -(fpoint[j]-240)/(fl/480)    
    return fpoint
 
def main():
    global rgb_img
    global depth_img
    
    rospy.init_node('d_points', anonymous=True)
    rospy.Subscriber('/camera/rgb/image_raw', Image, rgb_callback )
    rospy.Subscriber('/camera/depth/image_raw', Image, d_callback)
    pub = rospy.Publisher('3_point_features', floatList, queue_size = 1)
    features = floatList()
       
    while not rospy.is_shutdown():
         
        xy = xy_points(rgb_img)
        c = 0
        while c < 20:
            xy = xy + xy_points(rgb_img)
            c += 1
        xy = xy / c
               
        features.data = f_points(depth_img, xy)
        print "Green, Blue, Red, Orange :"
        print(features.data)
        pub.publish(features)
        
if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

