#!/usr/bin/env python

import rospy
import rosnode
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
from math import sin
import time


RA1 = rospy.Publisher('/RA1_controller/command', Float64, queue_size=10)
RA2 = rospy.Publisher('/RA2_controller/command', Float64, queue_size=10)
RA3 = rospy.Publisher('/RA3_controller/command', Float64, queue_size=10)
RA4 = rospy.Publisher('/RA4_controller/command', Float64, queue_size=10)
LA1 = rospy.Publisher('/LA1_controller/command', Float64, queue_size=10)
LA2 = rospy.Publisher('/LA2_controller/command', Float64, queue_size=10)
LA3 = rospy.Publisher('/LA3_controller/command', Float64, queue_size=10)
LA4 = rospy.Publisher('/LA4_controller/command', Float64, queue_size=10)
Goal_pos = 0;
Start_pos = -.50;
Tf = 50 ; #no of sec * 50

# 4 angle values to be sent for each joint 2 shoulder 1 elbow 1 wrist
def right_arm(Angles): 
    Curr_pos = 0 ;

    rate = rospy.Rate(50) # 50hz
    time.sleep(1);
    flag = 1;
    while (flag == 1 and rospy.is_shutdown() == 0):
	rospy.loginfo(rospy.get_caller_id() + " Publishing %s to right arm 1" ,Curr_pos );
	RA1.publish(Curr_pos );
	time.sleep(1);
 
	rospy.loginfo(rospy.get_caller_id() + " Publishing %s to right arm 2" ,Curr_pos );
	RA2.publish(1.57);
	time.sleep(1);

	rospy.loginfo(rospy.get_caller_id() + " Publishing %s to right arm 3" ,Curr_pos );
	RA3.publish(Curr_pos );
	time.sleep(1);

	rospy.loginfo(rospy.get_caller_id() + " Publishing %s to right arm 4" ,Curr_pos );
	RA4.publish(Curr_pos );
	flag = 0;
	#rate.sleep();

def left_arm():
    Curr_pos = 0 ;
    

    rate = rospy.Rate(50) # 50hz
    timer = 0;
    forw = 1;
    time.sleep(1);
    flag = 1;
    while (flag == 1 and rospy.is_shutdown() == 0):      
	rospy.loginfo(rospy.get_caller_id() + " Publishing %s to left arm 1" ,Curr_pos );
	RA1.publish(1.0 );
	time.sleep(1);

	rospy.loginfo(rospy.get_caller_id() + " Publishing %s to left arm 2" ,Curr_pos );
	LA2.publish(-1.57 );
	time.sleep(1);

	rospy.loginfo(rospy.get_caller_id() + " Publishing %s to left arm 3" ,Curr_pos );
	LA3.publish(Curr_pos );
	time.sleep(1);

	rospy.loginfo(rospy.get_caller_id() + " Publishing %s to left arm 4" ,Curr_pos );
	LA4.publish(Curr_pos ); 
	time.sleep(1);
	flag = 0;
	#rate.sleep();


if __name__ == '__main__':
    try:

    	rospy.init_node('main_client', anonymous=True)
        right_arm(['1','1',0,0]);
        left_arm();

    except rospy.ROSInterruptException:
        pass
