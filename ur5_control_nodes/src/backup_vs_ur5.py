#!/usr/bin/env python
import rospy
from ur5_control_nodes.msg import floatList
from sensor_msgs.msg import JointState
import numpy as np
from math import *
from std_msgs.msg import Header
from control_msgs.msg import *
from trajectory_msgs.msg import *
import jacobian_func

joint_positions = None
coordinates = None
c=0
j_velocity = None

def position_callback(position):
    global joint_positions
    joint_positions = position.position

def coordinate_callback(data):
    global coordinates
    coordinates = data.data

def joint_velocities(pose,points):
    if pose != None:
        if points != None:
            theta_x = -pi/2
            theta_y = pi/2
            check   = np.array([True,True])
            int_matrix = None
            fl = 531.15
            # desired_points = [73, 60, 922, -47, -47, 942, 93, -52, 910]
            desired_points =[-115.68442854453022, -33.44003012615327, 915, 33.74129165882132, 70.49519864432311, 942, -145.81058181133497, 90.37845980041423, 910]
            desired_points = np.asarray(desired_points)
            points = np.asarray(points)
            error = []
        
            for index in range(len(points)):
                if (index+1)%3 == 0:
                    index += 1
                else:
                    error.append(points[index] - desired_points[index])
            print "Error:",error
            avg_x=0
            avg_y=0
            error = np.asarray(error) #1v8 array
            for err_x in error[0:len(error):2]:
                avg_x+=err_x**2
            for err_y in error[1:len(error):2]:
                avg_y+=err_y**2
            mean_error=(avg_x+avg_y)**0.5
            print "Mean error:", mean_error
            if mean_error<=50:
                error=np.array([0,0,0,0,0,0])
            

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
            int_matrix = np.array([   [ -fl/z1,  0,    u1/z1,     u1*v1/fl,     -(fl*fl+u1*u1)/fl,  v1],
                                      [    0,  -fl/z1, v1/z1, (fl*fl+v1*v1)/fl,      -u1*v1/fl,    -u1],
                                      [ -fl/z2,  0,    u2/z2,     u2*v2/fl,      -(fl*fl+u2*u2)/fl, v2],
                                      [    0,  -fl/z2, v2/z2, (fl*fl+v2*v2)/fl,      -u2*v2/fl,    -u2],
                                      [ -fl/z3,  0,    u3/z3,     u3*v3/fl,      -(fl*fl+u3*u3)/fl, v3],
                                      [    0,  -fl/z3, v3/z3, (fl*fl+v3*v3)/fl,      -u3*v3/fl,    -u3]
                                  ])
            # int_matrix = np.array([   [ -fl/z1,  0,    u1/z1,     u1*v1/fl,     -(fl*fl+u1*u1)/fl,  v1],
            #                           [    0,  -fl/z1, v1/z1, (fl*fl+v1*v1)/fl,      -u1*v1/fl,    -u1],
            #                           [ -fl/z1,  0,    u2/z1,     u2*v2/fl,      -(fl*fl+u2*u2)/fl, v2],
            #                           [    0,  -fl/z1, v2/z1, (fl*fl+v2*v2)/fl,      -u2*v2/fl,    -u2],
            #                           [ -fl/z1,  0,    u3/z1,     u3*v3/fl,      -(fl*fl+u3*u3)/fl, v3],
            #                           [    0,  -fl/z1, v3/z1, (fl*fl+v3*v3)/fl,      -u3*v3/fl,    -u3]
            #                         ])
            # int_matrix=np.array([
            #                     [ -fl/z1,0,u1/z1,u1*v1/fl,0,0],
            #                     [ 0,-fl/z1,v1/z1,(fl*fl+v1*v1)/fl,0,0],
            #                     [ -fl/z2,0,u2/z2,u2*v2/fl,0,0],
            #                     [ 0,-fl/z2,v2/z2,(fl*fl+v2*v2)/fl,0,0 ],
            #                     [ -fl/z3,0,u3/z3,u3*v3/fl,0,0],
            #                     [ 0,-fl/z3,v3/z3,(fl*fl+v3*v3)/fl,0,0 ]
            #                     ])
            # int_matrix=np.array([
            #                     [ -fl/z1,0,0,0,0,0],
            #                     [ 0,0,0,0,0,0],
            #                     [ -fl/z2,0,0,0,0,0],
            #                     [ 0,0,0,0,0,0],
            #                     [ -fl/z3,0,0,0,0,0],
            #                     [ 0,0,0,0,0,0]
            #                     ])
            
            int_matrix = np.matrix(int_matrix) #8v6 matrix
            inverse    = np.linalg.pinv(int_matrix) #6v8 matrix
            
            cam_vel_np = -(0.00003)*inverse*error #6v1 matrix
                 
            cam_vel_lin     = cam_vel_np[[0,1,2],:]
            cam_vel_angular = cam_vel_np[[3,4,5],:]

            Rx = np.array([[1,0,0],[0,cos(theta_x),-sin(theta_x)],[0,sin(theta_x),cos(theta_x)]])
            Ry = np.array([[cos(theta_y) , 0, sin(theta_y)],[0, 1, 0],[-sin(theta_y), 0, cos(theta_y)]])
            # Lt=np.array([[1,0,0,-210],[0,1,0,0],[0,0,1,-85],[0,0,0,1]])
            # # print Rx
            # # print Ry
            # cam_vel_lin=Ry*cam_vel_lin
            # cam_vel_lin=Rx*cam_vel_lin
            
            # cam_vel_lin=np.vstack((cam_vel_lin,[1]))
            
            # cam_vel_lin=Lt*cam_vel_lin
            # cam_vel_lin=np.delete(cam_vel_lin,(3),axisjoint_vel=0)
            
            # cam_vel_angular=Rx*(Ry*cam_vel_angular)
            # cam_vel_angular=np.vstack((cam_vel_angular,[1]))
            # cam_vel_angular=Lt*cam_vel_angular
            # cam_vel_angular=np.delete(cam_vel_angular,(3),axis=0)

            cam_vel_np = np.concatenate((cam_vel_lin,cam_vel_angular))
            # cam_vel_np[2]=-cam_vel_np[2]
            # cam_vel_np[5]=-cam_vel_np[5]
            # cam_vel_np=np.array([[0],[0],[0],[0],[0],[0]])
            # cam_vel_np=np.ndarray.transpose(cam_vel_np)
            # cam_vel_list=np.ndarray.tolist(cam_vel_np)
            # cam_vel_final=cam_vel_list[0]
            temp1 = cam_vel_np.item((0,0))
            temp2 = cam_vel_np.item((1,0))
            temp3 = cam_vel_np.item((2,0))
            temp4 = cam_vel_np.item((3,0))
            temp5 = cam_vel_np.item((4,0))
            temp6 = cam_vel_np.item((5,0))
            # cam_vel_np[0,0] = temp3
            # cam_vel_np[1,0] = -temp1
            # cam_vel_np[2,0] = temp2
            # cam_vel_np[3,0] = -(10**2)*temp6
            # cam_vel_np[4,0] = (10**2)*temp4
            # cam_vel_np[5,0] = -2*(10**2)*temp5

            cam_vel_np[0,0] = temp3
            cam_vel_np[1,0] = temp1
            cam_vel_np[2,0] = temp2
            cam_vel_np[3,0] = temp6
            cam_vel_np[4,0] = temp4
            cam_vel_np[5,0] = temp5

            # cam_vel_np[3,0] = -(10**2)*temp4
            # cam_vel_np[4,0] = 0
            # cam_vel_np[5,0] = -0
            # cam_vel_np[0,0] = temp3
            # cam_vel_np[1,0] = -temp1
            # cam_vel_np[2,0] = temp2
            # cam_vel_np[3,0] = -temp6
            # cam_vel_np[4,0] = temp4
            # cam_vel_np[5,0] = -temp5
            # cam_vel_np[3,0] = -1000*temp6
            # cam_vel_np[4,0] = 1000*temp4
            # cam_vel_np[5,0] = -(1000)*temp5
        
            # cam_vel_np=np.array([[0],[0],[0],[0.05],[0],[0]])
            print cam_vel_np
            
            # j=np.zeros(6,6)
            
            Q1=pose[0]
            Q2=pose[1]
            Q3=pose[2]
            Q4=pose[3]
            Q5=pose[4]
            Q6=pose[5]

            jacobian=jacobian_func.calc_jack(Q1,Q2,Q3,Q4,Q5,Q6)

            # jacobian=1000*jacobian
            # print jacobian
            jacobian=np.matrix(jacobian)
            jacobian=np.linalg.pinv(jacobian)
            joint_vel=jacobian*cam_vel_np
            joint_vel=joint_vel
            joint_vel=np.ndarray.transpose(joint_vel)

            
            joint_vel=joint_vel.tolist()
            joint_vel=joint_vel[0]
           
            joint_vel=tuple(joint_vel)
            # joint_vel=(0.05,0,0,0,0,0)
            print "Joint Velocity: "
            print joint_vel
            return joint_vel
            #Creating the publisher
            
def vs_ur5():
    global joint_positions
    global coordinates
    global j_velocity
    rospy.init_node('vs_ur5')
    rospy.Subscriber('/3_point_features', floatList,coordinate_callback,queue_size=1)
    rospy.Subscriber("/joint_states", JointState, position_callback,queue_size=1)
    cam_pub=rospy.Publisher('ur5_joint_velocities',floatList,queue_size=10)
    while not rospy.is_shutdown():
        
        j_velocity = joint_velocities(joint_positions,coordinates)
        new_List   = floatList()
        new_List.data = j_velocity    
        cam_pub.publish(new_List) 

if __name__ == '__main__':
    try:
        vs_ur5()
    except rospy.ROSInterruptException:
        pass

    
