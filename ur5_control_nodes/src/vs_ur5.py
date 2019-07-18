#!/usr/bin/env python

#INFO
# This code takes the feature points from the object detection code( object_detect.py / KCF_tracking.py)
# and computes the joint velocities according to the error between the current and desired features.
# Changed the controller from the classical one to the Levenberg-Marquadt controller. Faster convergence, more stable. 


# The end effector and the image plane do not have the same axes. The X,Y,Z axes of the image plane
# are the Y,Z,X axes for the end effector. This mapping has been done below by storing the camera velocities
# in temp variables and then appropriately rearranging them in the camera_velocity_np array.

# topics used:

# Subscribers: /four_point_features (takes feature points) // X,Y coordinates are in pixels, depth is in millimetres.
#              /joint_states     (takes current robot position, velocity, effort)

# Publishers: ur5_joint_velocities (publishes joint velocities) //velocities are in rad/s

# Jacobian is imported from the jacobian_func.py. For a different robot, just change the jacobian in that file.

import rospy
from ur5_control_nodes.msg import floatList
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Header
import jacobian_func
import cv2


joint_positions = None
coordinates = None

j_velocity = None

# takes joint poistions from the topic /joint_states 
def position_callback(position):
    global joint_positions
    joint_positions = position.position

# takes the point feature coordinates from the topic /four_point_features
def coordinate_callback(data):
    global coordinates
    coordinates = data.data

# takes the present joint positions from the topic /joint_states defined in vs_ur5()
# takes the xyz coordinates of the centroids from the topic /four_point_features defined in vs_ur5() 
# returns a list with errors and joint velocities as its two elements
def joint_velocities(pose,points):
    var_lambda = 0.0001
    var_mu=0.5
    if pose != None:
        if points != None:
            int_matrix = None
            fl = 531.15   #camera focal length in pixels
            # Put your desired points as an array here, from the output of the object_detect.py node.
            desired_points =[-84.35322914705328, -97.60873658444737, 1008, 53.022029749576355, 1.8075691960082847, 1006, -109.65919789116927, 5.4227075880248545, 988, 63.86744492562606, -93.08981359442666, 1008]

            desired_points = np.asarray(desired_points) #list to numpy array
            points = np.asarray(points) #real-time points
            error = []
            for index in range(len(points)):
                if (index+1)%3 == 0:
                    index += 1
                else:
                    error.append(points[index] - desired_points[index])
            L=[]
            L.append(error)
            L.append(points)
            # print error
            avg_x=0
            avg_y=0
            error = np.asarray(error) #1v8 array
            for err_x in error[0:len(error):2]:
                avg_x+=err_x**2
            for err_y in error[1:len(error):2]:
                avg_y+=err_y**2
            mean_error=(avg_x+avg_y)**0.5
            # print "Mean error:", mean_error
            if mean_error<=5:
                error=np.array([0,0,0,0,0,0,0,0])
            error = np.matrix(error)
            error = np.ndarray.transpose(error)
            u1 = points[0]
            v1 = points[1]
            z1 = points[2]
            u2 = points[3]
            v2 = points[4]
            z2 = points[5]
            u3 = points[6]
            v3 = points[7]
            z3 = points[8]
            u4 = points[9]
            v4 = points[10]
            z4 = points[11]
            #current interaction matrix
            int_matrix = np.array([   [ -fl/z1,  0,    u1/z1,     u1*v1/fl,     -(fl*fl+u1*u1)/fl,  v1],
                                      [    0,  -fl/z1, v1/z1, (fl*fl+v1*v1)/fl,      -u1*v1/fl,    -u1],
                                      [ -fl/z2,  0,    u2/z2,     u2*v2/fl,      -(fl*fl+u2*u2)/fl, v2],
                                      [    0,  -fl/z2, v2/z2, (fl*fl+v2*v2)/fl,      -u2*v2/fl,    -u2],
                                      [ -fl/z3,  0,    u3/z3,     u3*v3/fl,      -(fl*fl+u3*u3)/fl, v3],
                                      [    0,  -fl/z3, v3/z3, (fl*fl+v3*v3)/fl,      -u3*v3/fl,    -u3],
                                      [ -fl/z4,  0,    u4/z4,     u4*v4/fl,      -(fl*fl+u4*u4)/fl, v4],
                                      [    0,  -fl/z4, v4/z4, (fl*fl+v4*v4)/fl,      -u4*v4/fl,    -u4]
                                    ])
            int_matrix = np.matrix(int_matrix) #8v6 matrix
            # inverse    = np.linalg.pinv(int_matrix) #6v8 matrix
            L_trans = np.transpose(int_matrix)
            x = (np.linalg.det(int_matrix*L_trans))**0.5
            # print "MOM_L : ",x 
            H = L_trans*int_matrix
            Hdiag = np.diag(H)
            Hdiag = np.diag(Hdiag)
            H_sum = H+var_mu*Hdiag
            H_sum_inverse = np.linalg.pinv(H_sum)
            if mean_error > 30:
                cam_vel_np = -var_lambda*H_sum_inverse*L_trans*error  #Levenberg-Marquadt Controller
                L.append(cam_vel_np)
            else:
                var_lambda = 0.0001
                cam_vel_np = -var_lambda*H_sum_inverse*L_trans*error  #Levenberg-Marquadt Controller
                L.append(cam_vel_np)
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
            Q1 = pose[0]
            Q2 = pose[1]
            Q3 = pose[2]
            Q4 = pose[3]
            Q5 = pose[4]
            Q6 = pose[5]
            jacobian = jacobian_func.calc_jack(Q1,Q2,Q3,Q4,Q5,Q6)
            jacobian = np.matrix(jacobian)
            J_trans = np.transpose(jacobian)
            y = (np.linalg.det(jacobian*J_trans))**0.5
            # print "MOM_J : ",y
            jacobian = np.linalg.pinv(jacobian)
            joint_vel = jacobian*cam_vel_np
            joint_vel = joint_vel
            joint_vel = np.ndarray.transpose(joint_vel)
            joint_vel = joint_vel.tolist()
            joint_vel = joint_vel[0]
            # joint_vel[5]=joint_vel[5]*100
            joint_vel = tuple(joint_vel)
            L.append(joint_vel)            
            print "Joint Velocity: "
            print joint_vel
            return L  #returns a list with errors and joint velocities as its two elements



# main function
# publishes joint velocities to the topic ur5_joint_velocities 
# publishes point errors to the topic points_error with timestamp 
def vs_ur5():
    global joint_positions
    global coordinates
    global j_velocity
    data_file = open("visual_servo_data.txt","w+")
    data_file.write("New Data: \n")
    data_file.write("\n")
    rospy.init_node('vs_ur5')
    rospy.Subscriber('/four_point_features', floatList,coordinate_callback,queue_size=1)
    rospy.Subscriber("/joint_states", JointState, position_callback,queue_size=1)
    #Creating the publisher
    cam_pub=rospy.Publisher('ur5_joint_velocities',floatList,queue_size=10)
    error_pub = rospy.Publisher('points_error',floatList,queue_size=10)
    while not rospy.is_shutdown():
        total_data = joint_velocities(joint_positions,coordinates)
        
        if total_data!=None:
            data_file.write("Error: \n"+str(total_data[0])+"\n")
            data_file.write("Points: \n"+str(total_data[1])+"\n")
            data_file.write("Camera Velocity: \n"+str(total_data[1])+"\n")
            data_file.write("Joint Velocity: \n"+str(total_data[1])+"\n")
            data_file.write("\n")
            data_file.write("\n")
            # print total_data
            # final_error_list=[]
            error_list = total_data[0]
            j_velocity = total_data[3]
            # for index in range (0,len(error_list),2):
            #     x = (error_list[index]**2+error_list[index+1])**0.5  #computes the average error for a point
            #     final_error_list.append(x)
            new_List   = floatList()
            new_List.data = j_velocity  
            new_List.header.stamp =  rospy.Time.now()
            error_l = floatList()
            error_l.header.stamp = rospy.Time.now()  #gives the timestamp of the error message as header
            error_l.data = error_list
            cam_pub.publish(new_List)   # publishes joint velocity 
            error_pub.publish(error_l)  # publishes point error 
    data_file.close()
    

if __name__ == '__main__':
    try:
        vs_ur5()
    except rospy.ROSInterruptException:
        pass

    
