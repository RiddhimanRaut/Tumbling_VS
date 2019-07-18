#!/usr/bin/env python
import rospy
from ur5_control_nodes.msg import floatList

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('ur5_joint_velocities', floatList, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    vels = floatList()
    vels.data=[0.05,0,0,0,0,0]
    
    while not rospy.is_shutdown():
            # print rospy.Time.now()
            pub.publish(vels)
            
        

if __name__ == '__main__':
    talker()
