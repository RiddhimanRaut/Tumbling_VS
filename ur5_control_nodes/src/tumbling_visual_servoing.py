#!/usr/bin/env python
import rospy
from ur5_control_nodes.msg import floatList
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Header
import jacobian_func
import cv2
from math import *
from std_msgs.msg import String

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import imutils 


ellipse_data=None
depth_img = np.zeros((480,640))
bridge    = CvBridge()
joint_positions = None

def position_callback(position):
    global joint_positions
    joint_positions = position.position

def ellipse_params_callback(data):
    global ellipse_data
    ellipse_data = data.data

def depth_callback(msg):
    global depth_img
    depth_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    

def joint_velocities(parameters,dimg,pose):
    if pose!= None:
        if parameters!=None:
            ellipse_matrix=np.matrix(parameters)
            desired_points=[154.2459047260403, -50.61193748823197, 524, -37.356430050837886, -187.98719638486162, 530, 157.86104311805687, -187.98719638486162, 530, -59.0472604029373, -58.745998870269254, 532]
            error=[]
            rows=len(ellipse_matrix)
            j=0
            L_total = []
            
            z = [500,500,500]
            
            for i in range(rows):
                
                fl=531.15
                x0 = ellipse_matrix[i,0] #centre1
                y0 = ellipse_matrix[i,1] #centre2
                x0_pixel = int(x0*(-fl/640)+320)
                y0_pixel = int(y0*(-fl/480)+240)
                # print x0_pixel,y0_pixel
                z0 = dimg[y0_pixel][x0_pixel]
                if z0!=0:
                    z[0] = z0
                    
                # print z0
                
                a_major = ellipse_matrix[i,2]
                b_minor = ellipse_matrix[i,3]
                alpha = ellipse_matrix[i,4]
                x = desired_points[j] #desired pts 1
                y = desired_points[j+1] #desired pts 2
                
                sx = cos(alpha)*((x0-x)/a_major)+sin(alpha)*((y0-y)/b_minor)
                sy = -sin(alpha)*((x0-x)/a_major)+cos(alpha)*((y0-y)/b_minor)
                error.append(sx)
                error.append(sy)
                
                x2 = x0 + a_major*cos(alpha)
                y2 = y0 + a_major*sin(alpha)
                x2_pixel = int(x2*(-fl/640)+320)
                y2_pixel = int(y2*(-fl/480)+240)
                # print x2_pixel,y2_pixel
                z2 = dimg[y2_pixel][x2_pixel]
                if z2!=0:
                    z[1] = z2
                    
                
                x3 = x0 + b_minor*cos(alpha+pi/2)
                y3 = y0 + b_minor*sin(alpha+pi/2)
                x3_pixel = int(x3*(-fl/640)+320)
                y3_pixel = int(y3*(-fl/480)+240)
                # print x3_pixel,y3_pixel
                z3 = dimg[y3_pixel][x3_pixel]
                if z3!=0:
                    z[2] = z3
                    
                
                # print z
                # print int(x3*(fl/640)+320),int(y3*(fl/480)+240)
                # print"Depth"
                # print z0,z2,z3
                if z[1]!=0 and z[2]!=0 and z[0]!=0:
                    z0 = z[0]
                    z2 = z[1]
                    z3 = z[2]
                    L_x0 = np.array([-1/z0, 0, x0/z0, x0*y0, -(1+x0**2), y0])
                    L_x2 = np.array([-1/z2, 0, x2/z3, x2*y2, -(1+x2**2), y2])
                    L_x3 = np.array([-1/z3, 0, x3/z3, x3*y3, -(1+x3**2), y3])
                    L_y0 = np.array([0, -1/z0, y0/z0, 1+y0**2, -x0*y0, -x0])
                    L_y2 = np.array([0, -1/z2, y2/z2, 1+y2**2, -x2*y2, -x2])
                    L_y3 = np.array([0, -1/z3, y3/z3, 1+y3**2, -x3*y3, -x3])
                    T1 = -(y2-y0)/((x2-x0)**2+(y2-y0)**2)
                    T2 = (x2-x0)/((x2-x0)**2+(y2-y0)**2)
                    L_alpha = T1*(L_x2-L_x0)+T2*(L_y2-L_y0)
                    L_a = (x2-x0)*(L_x2-L_x0)/a_major+ (y2-y0)*(L_y2-L_y0)/a_major
                    L_b = (x3-x0)*(L_x3-L_x0)/b_minor+ (y3-y0)*(L_y3-L_y0)/b_minor
                    L_sx = sy*L_alpha- cos(alpha)*(x0-x)*L_a/(2*a_major**2)- sin(alpha)*(y0-y)*L_b/(2*b_minor**2)+cos(alpha)*L_x0/a_major+sin(alpha)*L_y0/b_minor
                    L_sy = -sx*L_alpha+ sin(alpha)*(x0-x)*L_a/(2*a_major**2)- cos(alpha)*(y0-y)*L_b/(2*b_minor**2)+cos(alpha)*L_y0/b_minor-sin(alpha)*L_x0/a_major
                    # L_sx = cos(alpha)*L_x0/a_major+sin(alpha)*L_y0/b_minor
                    # L_sy = cos(alpha)*L_y0/b_minor-sin(alpha)*L_x0/a_major
                    L_total.append(L_sx)
                    L_total.append(L_sy)
                    # print "LOOP END"
                    j = j+3
                    
                    
                else:
                    print "missed"
                    if z0 == 0:
                        print "z0"
                    elif z2 == 0:
                        print "z2"
                    elif z3 == 0:
                        print "z3"
                    
                # print "iteration:",i
                    

            var_mu = 0.5
            var_lambda = 0.00001   
            error = np.matrix(error)
            error = error.transpose()
            int_matrix = np.matrix(L_total)
            L_trans = np.transpose(int_matrix)
            H = L_trans*int_matrix
            Hdiag = np.diag(H)
            Hdiag = np.diag(Hdiag)
            H_sum = H+var_mu*Hdiag
            H_sum_inverse = np.linalg.pinv(H_sum)
            # print L_total
            # L_total_inverse = np.linalg.pinv(int_matrix)
            # print L_total_inverse.shape
            # print error
            cam_vel_np = -var_lambda*H_sum_inverse*L_trans*error
            
            #----------Mapping------------#
                
            temp1 = cam_vel_np.item((0,0))
            temp2 = cam_vel_np.item((1,0))
            temp3 = cam_vel_np.item((2,0))
            temp4 = cam_vel_np.item((3,0))
            temp5 = cam_vel_np.item((4,0))
            temp6 = cam_vel_np.item((5,0))
            cam_vel_np[0,0] = temp3
            cam_vel_np[1,0] = temp1
            cam_vel_np[2,0] = temp2
            cam_vel_np[3,0] = temp6
            cam_vel_np[4,0] = temp4
            cam_vel_np[5,0] = temp5
            #-----------------------#
            # return cam_vel_np        
            Q1 = pose[0]
            Q2 = pose[1]
            Q3 = pose[2]
            Q4 = pose[3]
            Q5 = pose[4]
            Q6 = pose[5]
            jacobian = jacobian_func.calc_jack(Q1,Q2,Q3,Q4,Q5,Q6)
            jacobian = np.matrix(jacobian)
            
            jacobian = np.linalg.pinv(jacobian)
            joint_vel = jacobian*cam_vel_np
            joint_vel = np.ndarray.transpose(joint_vel)
            joint_vel = joint_vel.tolist()
            joint_vel = joint_vel[0]
            joint_vel = tuple(joint_vel)
            # print "Joint Velocity: "
            return joint_vel
    

def tumbling_vs_ur5():
    global joint_positions
    global ellipse_data
    global depth_img
    rospy.init_node('tumbling_vs_ur5')
    rospy.Subscriber("/ellipse_param", String, ellipse_params_callback,queue_size=1)
    rospy.Subscriber('/camera/depth/image_raw', Image, depth_callback)
    rospy.Subscriber("/joint_states", JointState, position_callback,queue_size=1)
    cam_pub=rospy.Publisher('ur5_joint_velocities',floatList,queue_size=10)
    while not rospy.is_shutdown():
        new_List   = floatList()
        new_List.data = joint_velocities(ellipse_data,depth_img,joint_positions)
        print new_List.data
        cam_pub.publish(new_List)

if __name__ == '__main__':
    try:
        tumbling_vs_ur5()
    except rospy.ROSInterruptException:
        pass
