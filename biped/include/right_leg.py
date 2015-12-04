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
motorid_RL = [13,14,15,16,17,18];
update_rate = 50;

############################################################################################################################

def right_leg(goal_pos,time_limit):
    global start_pos;
    motorRL1_response = motor_data_client(motorid_RL[0]);
    motorRL2_response = motor_data_client(motorid_RL[1]);
    motorRL3_response = motor_data_client(motorid_RL[2]);
    motorRL4_response = motor_data_client(motorid_RL[3]);
    motorRL5_response = motor_data_client(motorid_RL[4]);
    motorRL6_response = motor_data_client(motorid_RL[5]);
    start_pos = [motorRL1_response.current_pos,motorRL2_response.current_pos,motorRL3_response.current_pos,motorRL4_response.current_pos,motorRL5_response.current_pos,motorRL6_response.current_pos];
    curr_pos = start_pos;

    #handlers for motor publishers
    RL1 = rospy.Publisher('/RL1_controller/command', Float64, queue_size=10);
    RL2 = rospy.Publisher('/RL2_controller/command', Float64, queue_size=10);
    RL3 = rospy.Publisher('/RL3_controller/command', Float64, queue_size=10);
    RL4 = rospy.Publisher('/RL4_controller/command', Float64, queue_size=10);
    RL5 = rospy.Publisher('/RL5_controller/command', Float64, queue_size=10);
    RL6 = rospy.Publisher('/RL6_controller/command', Float64, queue_size=10);
    
    #initialize node for the specific subpart
    #rospy.init_node('Right_arm_node', anonymous=True);  

    rate = rospy.Rate(update_rate) # 50hz update rate
    time.sleep(0.05);      # make the system sleep a while 
    time_count = 0 ;
    time_limit = time_limit * update_rate; 

    while (rospy.is_shutdown() == 0 and time_count <= time_limit ):
        global curr_pos; 
        curr_pos = calculate_trajectory(time_count,start_pos,goal_pos,time_limit);

        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to right leg motor 1" %curr_pos[0] );
        RL1.publish(curr_pos[0] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to right leg motor 2" %curr_pos[1] );
        RL2.publish(curr_pos[1] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to right leg motor 3" %curr_pos[2] );
        RL3.publish(curr_pos[2] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to right leg motor 4" %curr_pos[3] );
        RL4.publish(curr_pos[3] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to right leg motor 5" %curr_pos[4] );
        RL5.publish(curr_pos[4] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to right leg motor 6" %curr_pos[5] );
        RL6.publish(curr_pos[5] );
        time_count = time_count + 1;
        time.sleep(0.02);
   
###########################################################################################################################

def calculate_trajectory(time_count,start_pos,goal_pos,time_limit):
    curr_position = start_pos;    
    curr_position[0] = start_pos[0] + ((goal_pos[0]-start_pos[0])/time_limit)*(time_count - (time_limit/2/3.14)*sin(2*3.14*time_count/time_limit));
    curr_position[1] = start_pos[1] + ((goal_pos[1]-start_pos[1])/time_limit)*(time_count - (time_limit/2/3.14)*sin(2*3.14*time_count/time_limit));
    curr_position[2] = start_pos[2] + ((goal_pos[2]-start_pos[2])/time_limit)*(time_count - (time_limit/2/3.14)*sin(2*3.14*time_count/time_limit));
    curr_position[3] = start_pos[3] + ((goal_pos[3]-start_pos[3])/time_limit)*(time_count - (time_limit/2/3.14)*sin(2*3.14*time_count/time_limit));
    curr_position[4] = start_pos[4] + ((goal_pos[4]-start_pos[4])/time_limit)*(time_count - (time_limit/2/3.14)*sin(2*3.14*time_count/time_limit));
    curr_position[5] = start_pos[5] + ((goal_pos[5]-start_pos[5])/time_limit)*(time_count - (time_limit/2/3.14)*sin(2*3.14*time_count/time_limit));
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
        right_leg([0,1,1,1,0,0],1);
        time.sleep(2);
        right_leg([0,0,0,0,0,0],2);
        time.sleep(2);
    except rospy.ROSInterruptException:
        pass
