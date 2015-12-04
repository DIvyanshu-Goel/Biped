#!/usr/bin/env python

import sys
import rospy
import time
import numpy as np
from std_msgs.msg import *
from math import *
from dynamixel_msgs.msg import JointState
from biped.msg import *
from biped.srv import *
#for details on motor ids see Data_Server.py
start_pos = [0, 0 ,0 ,0];
motorid_LA = [4,5,6,7];
update_rate = 50;

###########################################################################################################################

def left_arm(goal_pos,time_limit):
    global start_pos;
    motorLA1_response = motor_data_client(motorid_LA[0]);
    motorLA2_response = motor_data_client(motorid_LA[1]);
    motorLA3_response = motor_data_client(motorid_LA[2]);
    motorLA4_response = motor_data_client(motorid_LA[3]);
    start_pos = [motorLA1_response.current_pos,motorLA2_response.current_pos,motorLA3_response.current_pos,motorLA4_response.current_pos];
    curr_pos = start_pos;

    #handlers for motor publishers
    LA1 = rospy.Publisher('/LA1_controller/command', Float64, queue_size=10);
    LA2 = rospy.Publisher('/LA2_controller/command', Float64, queue_size=10);
    LA3 = rospy.Publisher('/LA3_controller/command', Float64, queue_size=10);
    LA4 = rospy.Publisher('/LA4_controller/command', Float64, queue_size=10);
    
    #initialize node for the specific subpart
    #rospy.init_node('Left_arm_node', anonymous=True);  

    rate = rospy.Rate(update_rate) # 50hz update rate
    time.sleep(0.05);      # make the system sleep a while 
    time_count = 0 ;
    time_limit = time_limit * update_rate; 

    while (rospy.is_shutdown() == 0 and time_count <= time_limit ):
        global curr_pos; 
        curr_pos = calculate_trajectory(time_count,start_pos,goal_pos,time_limit);

        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to left arm motor 1" %curr_pos[0] );
        LA1.publish(curr_pos[0] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to left arm motor 2" %curr_pos[1] );
        LA2.publish(curr_pos[1] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to left arm motor 3" %curr_pos[2] );
        LA3.publish(curr_pos[2] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to left arm motor 4" %curr_pos[3] );
        LA4.publish(curr_pos[3] );
        time_count = time_count + 1;
        time.sleep(0.02);
   
###########################################################################################################################

def calculate_trajectory(time_count,start_pos,goal_pos,time_limit):
    curr_position = start_pos;    
    curr_position[0] = start_pos[0] + ((goal_pos[0]-start_pos[0])/time_limit)*(time_count - (time_limit/2/3.14)*sin(2*3.14*time_count/time_limit));
    curr_position[1] = start_pos[1] + ((goal_pos[1]-start_pos[1])/time_limit)*(time_count - (time_limit/2/3.14)*sin(2*3.14*time_count/time_limit));
    curr_position[2] = start_pos[2] + ((goal_pos[2]-start_pos[2])/time_limit)*(time_count - (time_limit/2/3.14)*sin(2*3.14*time_count/time_limit));
    curr_position[3] = start_pos[3] + ((goal_pos[3]-start_pos[3])/time_limit)*(time_count - (time_limit/2/3.14)*sin(2*3.14*time_count/time_limit));
    return(curr_position);   

###########################################################################################################################

def motor_data_client(x):
    rospy.wait_for_service('Fetch_Motor_data')
    client = rospy.ServiceProxy('Fetch_Motor_data', Fetch_Motor_Data)
    resp1 = client(x);
    return (resp1);

###########################################################################################################################

if __name__ == '__main__':
    try:
        left_arm([0,1,1,1],1);
        time.sleep(2);
        left_arm([0,0,0,0],2);
        time.sleep(2);
    except rospy.ROSInterruptException:
        pass
