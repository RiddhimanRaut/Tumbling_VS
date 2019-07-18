#!/usr/bin/env python
import rospy
from ur5_control_nodes.msg import floatList
from sensor_msgs.msg import JointState
import numpy as np
from std_msgs.msg import Header
import jacobian_func
import cv2
import matplotlib.pyplot as plt
import Translation_matrix
from mpl_toolkits import mplot3d

points = None
joint_velocity = None
joint_pose = None
error = None
time_error = None
time_velocity = None
error_plotter_1 = []
error_plotter_2 = []
error_plotter_3 = []
error_plotter_4 = []
error_plotter_mean = []
vel_plotter_1 = []
vel_plotter_2 = []
vel_plotter_3 = []
vel_plotter_4 = []
vel_plotter_5 = []
vel_plotter_6 = []
green_x = []
green_y = []
blue_x = []
blue_y = []
red_x = []
red_y = []
orange_x = []
orange_y = []
end_eff_x = []
end_eff_y = []
end_eff_z = []

time_axis_error=[]

def error_callback(data):
    global error
    global time_error
    error = data.data
    time_error = data.header.stamp.secs

def point_callback(pts):
    global points
    points = pts.data
    
    

def velocity_callback(vel):
    global joint_velocity, time_velocity, joint_pose
    joint_velocity = vel.velocity
    joint_pose = vel.position
    time_velocity = vel.header.stamp.secs
    # print time_velocity

def plotter(vels,error_list):
    global time_error
    final_error_list=[]
    L=[]
    if error_list!=None:
        for index in range (0,len(error_list),2):
            x = (error_list[index]**2+error_list[index+1])**0.5  #computes the average error for a point
            final_error_list.append(x)
    # print final_error_list
    L.append(final_error_list)
    L.append(time_error)  
    return L


def vs_plotter():
    global error, points
    global joint_velocity,joint_pose
    global error_plotter_1,error_plotter_2,error_plotter_3,error_plotter_4
    global time_axis_error
    global end_eff_x,end_eff_y,end_eff_z
    
    rospy.init_node('vs_plotter')
    rospy.Subscriber('/points_error', floatList,error_callback,queue_size=1)
    rospy.Subscriber("/joint_states", JointState, velocity_callback,queue_size=1)
    rospy.Subscriber('/four_point_features', floatList,point_callback,queue_size=1)
    
    while not rospy.is_shutdown() :
        
        z = plotter(joint_velocity,error)
        errors = z[0]
        
        t = z[1]
        
    
        if t not in time_axis_error and t!=None:
            
            if errors!=None and joint_velocity!=None and points!=None and joint_pose!=None:
                Q1 = joint_pose[0]
                Q2 = joint_pose[1]
                Q3 = joint_pose[2]
                Q4 = joint_pose[3]
                Q5 = joint_pose[4]
                Q6 = joint_pose[5] 
                transl = Translation_matrix.calc_eff_pos(Q1,Q2,Q3,Q4,Q5,Q6)   #1v3 array
                
                time_axis_error.append(t)
                error_1 = errors[0]
                error_2 = errors[1]
                error_3 = errors[2]
                error_4 = errors[3]
                error_mean = (error_1+error_2+error_3+error_4)*0.25
                green_x.append(-points[0])
                green_y.append(points[1])
                blue_x.append(-points[3])
                blue_y.append(points[4])
                red_x.append(-points[6])
                red_y.append(points[7])
                orange_x.append(-points[9])
                orange_y.append(points[10])
                
                vel_plotter_1.append(joint_velocity[0])
                vel_plotter_2.append(joint_velocity[1])
                vel_plotter_3.append(joint_velocity[2])
                vel_plotter_4.append(joint_velocity[3])
                vel_plotter_5.append(joint_velocity[4])
                vel_plotter_6.append(joint_velocity[5])
                error_plotter_1.append(error_1)
                error_plotter_2.append(error_2)
                error_plotter_3.append(error_3)
                error_plotter_4.append(error_4)
                error_plotter_mean.append(error_mean)

                end_eff_x.append(transl[0])
                end_eff_y.append(transl[1])
                end_eff_z.append(transl[2])
                    
                    
    # start_time = time_axis_error[0]  
    # for i in time_axis_error:
    #     i = i - start_time     
    # print time_axis_error        
    # print len(vel_plotter_1)
    # print len(time_axis_error)

    
    plt.figure(1)
    # plt.plot(time_axis_error,error_plotter_1,label= "error 1")
    # plt.plot(time_axis_error,error_plotter_2,label= "error 2")
    # plt.plot(time_axis_error,error_plotter_3,label= "error 3")
    # plt.plot(time_axis_error,error_plotter_4,label= "error 4")
    plt.plot(time_axis_error,error_plotter_mean,label = "Mean Error")
    plt.legend()

    plt.xlabel('Time(s) ------> ')
    plt.ylabel('Mean Error ------>')

    plt.figure(2)
    plt.plot(time_axis_error,vel_plotter_1,label= "base")
    plt.plot(time_axis_error,vel_plotter_2,label= "shoulder")
    plt.plot(time_axis_error,vel_plotter_3,label= "elbow")
    plt.plot(time_axis_error,vel_plotter_4,label= "wrist 1")
    plt.plot(time_axis_error,vel_plotter_5,label= "wrist 2")
    plt.plot(time_axis_error,vel_plotter_6,label= "wrist 3")
    plt.xlabel('Time(s) ------>')
    plt.ylabel('Joint Velocity ------>')
    plt.legend()
    
    plt.figure(3)
    plt.plot(green_x,green_y,label= "green",color='green')
    plt.plot(blue_x,blue_y,label= "blue",color='blue')
    plt.plot(red_x,red_y,label= "red",color='red')
    plt.plot(orange_x,orange_y,label= "orange",color='orange')
    plt.xlabel('X-axis ------>')
    plt.ylabel('Y-Axis ------>')
    plt.legend()

    plt.figure(4)
    ax = plt.axes(projection="3d")
    ax.plot3D(end_eff_x,end_eff_y,end_eff_z,'blue')
    ax.set_xlabel('X-axis ------>')
    ax.set_ylabel('Y-axis ------>')
    ax.set_zlabel('Z-axis ------>')


    plt.show()
    

if __name__ == '__main__':
    try:
        vs_plotter()
    except rospy.ROSInterruptException:
        pass
