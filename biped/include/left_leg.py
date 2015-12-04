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
start_pos = [0, 0 ,0 , 0, 0, 0];
motorid_LL = [19,20,21,22,23,24];
update_rate = 50;

###########################################################################################################################

def left_leg(goal_pos,time_limit):
    global start_pos;
    motorLL1_response = motor_data_client(motorid_LL[0]);
    motorLL2_response = motor_data_client(motorid_LL[1]);
    motorLL3_response = motor_data_client(motorid_LL[2]);
    motorLL4_response = motor_data_client(motorid_LL[3]);
    motorLL5_response = motor_data_client(motorid_LL[4]);
    motorLL6_response = motor_data_client(motorid_LL[5]);
    start_pos = [motorLL1_response.current_pos,motorLL2_response.current_pos,motorLL3_response.current_pos,motorLL4_response.current_pos,motorLL5_response.current_pos,motorLL6_response.current_pos];
    curr_pos = start_pos;

    #handlers for motor publishers
    LL1 = rospy.Publisher('/LL1_controller/command', Float64, queue_size=10);
    LL2 = rospy.Publisher('/LL2_controller/command', Float64, queue_size=10);
    LL3 = rospy.Publisher('/LL3_controller/command', Float64, queue_size=10);
    LL4 = rospy.Publisher('/LL4_controller/command', Float64, queue_size=10);
    LL5 = rospy.Publisher('/LL5_controller/command', Float64, queue_size=10);
    LL6 = rospy.Publisher('/LL6_controller/command', Float64, queue_size=10);
    
    #initialize node for the specific subpart
    #rospy.init_node('Left_leg_node', anonymous=True);  

    rate = rospy.Rate(update_rate) # 50hz update rate
    time.sleep(0.05);      # make the system sleep a while 
    time_count = 0 ;
    time_limit = time_limit * update_rate; 

    while (rospy.is_shutdown() == 0 and time_count <= time_limit ):
        global curr_pos; 
        curr_pos = calculate_trajectory(time_count,start_pos,goal_pos,time_limit);

        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to left leg motor 1" %curr_pos[0] );
        LL1.publish(curr_pos[0] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to left leg motor 2" %curr_pos[1] );
        LL2.publish(curr_pos[1] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to left leg motor 3" %curr_pos[2] );
        LL3.publish(curr_pos[2] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to left leg motor 4" %curr_pos[3] );
        LL4.publish(curr_pos[3] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to left leg motor 5" %curr_pos[4] );
        LL5.publish(curr_pos[4] );
        rospy.loginfo(rospy.get_caller_id() + " Publishing %s to left leg motor 6" %curr_pos[5] );
        LL6.publish(curr_pos[5] );
        time_count = time_count + 1;
        time.sleep(0.03);
   
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
        left_leg([0,1,1,1],1);
        time.sleep(2);
        left_leg([0,0,0,0],2);
        time.sleep(2);
    except rospy.ROSInterruptException:
        pass
