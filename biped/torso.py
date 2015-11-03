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
start_pos = [0, 0 ,0 ,0 ,0];
motorid_T = [8,9,10,11,12];
update_rate = 50;

###########################################################################################################################

def torso(goal_pos,time_limit):
    global start_pos;
    motorT1_response = motor_data_client(motorid_T[0]);
    motorT2_response = motor_data_client(motorid_T[1]);
    motorT3_response = motor_data_client(motorid_T[2]);
    motorT4_response = motor_data_client(motorid_T[3]);
    motorT5_response = motor_data_client(motorid_T[4]);
    start_pos = [motorT1_response.current_pos,motorT2_response.current_pos,motorT3_response.current_pos,motorT4_response.current_pos,motorT5_response.current_pos];
    curr_pos = start_pos;

    #handlers for motor publishers
    T1 = rospy.Publisher('/T1_controller/command', Float64, queue_size=10);
    T2 = rospy.Publisher('/T2_controller/command', Float64, queue_size=10);
    T3 = rospy.Publisher('/T3_controller/command', Float64, queue_size=10);
    T4 = rospy.Publisher('/T4_controller/command', Float64, queue_size=10);
    T5 = rospy.Publisher('/T5_controller/command', Float64, queue_size=10);
    
    #initialize node for the specific subpart
    rospy.init_node('Torso_node', anonymous=True);  

    rate = rospy.Rate(update_rate) # 50hz update rate
    time.sleep(0.05);      # make the system sleep a while 
    time_count = 0 ;
    time_limit = time_limit * update_rate; 

    while (rospy.is_shutdown() == 0 and time_count <= time_limit ):
        global curr_pos; 
        curr_pos = calculate_trajectory(time_count,start_pos,goal_pos,time_limit);

        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to Torso motor 1" %curr_pos[0] );
        T1.publish(curr_pos[0] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to Torso motor 2" %curr_pos[1] );
        T2.publish(curr_pos[1] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to Torso motor 3" %curr_pos[2] );
        T3.publish(curr_pos[2] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to Torso motor 4" %curr_pos[3] );
        T4.publish(curr_pos[3] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to Torso motor 5" %curr_pos[4] );
        T5.publish(curr_pos[4] );
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
        torso([0,1,1,1,0],1);
        time.sleep(2);
        torso([0,0,0,0,0],2);
        time.sleep(2);
    except rospy.ROSInterruptException:
        pass
