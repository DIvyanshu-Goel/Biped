#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from math import sin
import time

Goal_pos = 0;
Start_pos = -.50;
Tf = 50 ; #no of sec * 50
data = 0;
Curr_pos = 0;
last_motor_state = [0,0,0,0];
goal_motor_state = [0,0,0,0];

def right_arm():
    global Curr_pos;
    rospy.init_node('Right_arm_node', anonymous=True);
    rospy.Subscriber("/Right_arm", Float64, callback);
    
    RA1 = rospy.Publisher('/RA1_controller/command', Float64, queue_size=10);
    RA2 = rospy.Publisher('/RA2_controller/command', Float64, queue_size=10);
    RA3 = rospy.Publisher('/RA3_controller/command', Float64, queue_size=10);
    RA4 = rospy.Publisher('/RA4_controller/command', Float64, queue_size=10);

    rate = rospy.Rate(50) # 50hz
    time.sleep(0.2);      # make the system sleep a while 
    
    while not rospy.is_shutdown():
    
	rospy.loginfo(rospy.get_caller_id() + " Publishing %s to right arm motor 1" ,Curr_pos );
	RA1.publish(Curr_pos );
	time.sleep(0.2);
 
#	rospy.loginfo(rospy.get_caller_id() + " Publishing %s to right arm motor 2" ,Curr_pos );
#	RA2.publish(1.57);
#	time.sleep(0.2);
#
#	rospy.loginfo(rospy.get_caller_id() + " Publishing %s to right arm motor 3" ,Curr_pos );
#	RA3.publish(Curr_pos );
#	time.sleep(0.2);
#
#	rospy.loginfo(rospy.get_caller_id() + " Publishing %s to right arm motor 4" ,Curr_pos );
#	RA4.publish(Curr_pos );
#	time.sleep(0.2);
#	
	rate.sleep();
	
def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)



if __name__ == '__main__':
    try:

        right_arm();       

    except rospy.ROSInterruptException:
        pass
