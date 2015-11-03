#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from dynamixel_msgs.msg import JointState
from biped.msg import Motor_Angle
from math import sin
import time

Goal_pos = 0;
Start_pos = -.50;
Tf = 50 ; #no of sec * 50
data = 0;
Curr_pos =[1,1,1,1];

def right_armf():
    global Curr_pos;
    rospy.init_node('Master_node', anonymous=True);
    RA1 = rospy.Publisher('/Right_arm', Motor_Angle, queue_size=10);
    rate = rospy.Rate(50) # 50hz
    time.sleep(2);
    
    while not rospy.is_shutdown():
    
	rospy.loginfo(rospy.get_caller_id() + " Publishing %s to topic 1" ,Curr_pos );
	RA1.publish(Curr_pos);
	time.sleep(0.2);
	
	rospy.spin();


if __name__ == '__main__':
    try:

        right_armf();       

    except rospy.ROSInterruptException:
        pass
