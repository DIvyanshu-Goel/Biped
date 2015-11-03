#!/usr/bin/env python

#################################################################################
#
#matrix index --> motor_id
#0 --> 51 (Right_Arm_1)
#1 --> 52 (Right_Arm_2)
#2 --> 53 (Right_Arm_3)
#3 --> 54 (Right_Arm_4)
#4 --> 41 (Left_Arm_1)
#5 --> 42 (Left_Arm_2)
#6 --> 43 (Left_Arm_3)
#7 --> 44 (Left_Arm_4)
#8 --> 35 (Torso_1)
#9 --> 34 (Torso_2)
#10 --> 33 (Torso_3)
#11 --> 32 (Torso_4)
#12 --> 31 (Torso_5)
#13 --> 21 (Right_Leg_1)
#14 --> 22 (Right_Leg_2)
#15 --> 23 (Right_Leg_3)
#16 --> 24 (Right_Leg_4)
#17 --> X (Right_Leg_5)
#18 --> X (Right_Leg_6)
#19 --> 11 (Left_Leg_1)
#20 --> 12 (Left_Leg_2)
#21 --> 13 (Left_Leg_3)
#22 --> 14 (Left_Leg_4)
#23 --> 15 (Left_Leg_5)
#24 --> 16 (Left_Leg_6)
#25 --> 36 (Head_1)
#26 --> 37 (Head_2)
#27 --> X
#
#################################################################################
import sys
import rospy
import time
import numpy as np
from std_msgs.msg import *
from math import sin
from dynamixel_msgs.msg import JointState
from biped.msg import *
from biped.srv import *

motor_states = [JointState() for i in range(27)];
#################################################################################

def handle(req):
    global motor_states;
    rospy.loginfo('Reuested Feedback for Motor no %s '%req.id);
    pub1.publish(motor_states[req.id]);
    return Fetch_Motor_DataResponse(motor_states[req.id].motor_ids[0],motor_states[req.id].current_pos,motor_states[req.id].velocity,motor_states[req.id].load,motor_states[req.id].is_moving);
	
#################################################################################
#Right_Arm
def callback_RA1(data):
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
	global motor_states;
	motor_states[0] = data ;
def callback_RA2(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[1] = data ;
def callback_RA3(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[2] = data ;
def callback_RA4(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[3] = data ; 

#Left_Arm
def callback_LA1(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[4] = data ;
def callback_LA2(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[5] = data ;
def callback_LA3(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[6] = data ;
def callback_LA4(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[7] = data ;  

#Torso
def callback_T1(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[8] = data ;
def callback_T2(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[9] = data ;
def callback_T3(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[10] = data ;
def callback_T4(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[11] = data ;
def callback_T5(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[12] = data ;  

#Right_leg
def callback_RL1(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[13] = data ;
def callback_RL2(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[14] = data ;
def callback_RL3(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[15] = data ;
def callback_RL4(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[16] = data ;  

#Left_Leg
def callback_LL1(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[19] = data ;
def callback_LL2(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[20] = data ;
def callback_LL3(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[21] = data ;
def callback_LL4(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[22] = data ;
def callback_LL5(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[23] = data ;
def callback_LL6(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[24] = data ;

#Head
def callback_H1(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[25] = data ;
def callback_H2(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data);
    global motor_states;
    motor_states[26] = data ;
#################################################################################

def fetch_motor_data_server():
    rospy.init_node('Fetch_Motor_data_server',anonymous=True);
    server = rospy.Service('Fetch_Motor_data', Fetch_Motor_Data, handle);
    global pub1;
    pub1 = rospy.Publisher('/Motor_Info_Request', JointState , queue_size = 10); 

    # Right_Arm    
    rospy.Subscriber('/RA1_controller/state', JointState, callback_RA1);  
    rospy.Subscriber('/RA2_controller/state', JointState, callback_RA2);  
    rospy.Subscriber('/RA3_controller/state', JointState, callback_RA3);  
    rospy.Subscriber('/RA4_controller/state', JointState, callback_RA4); 

    # Left_Arm  
    rospy.Subscriber('/LA1_controller/state', JointState, callback_LA1);   
    rospy.Subscriber('/LA2_controller/state', JointState, callback_LA2);   
    rospy.Subscriber('/LA3_controller/state', JointState, callback_LA3);   
    rospy.Subscriber('/LA4_controller/state', JointState, callback_LA4);   

    #Torso
    rospy.Subscriber('/T1_controller/state', JointState, callback_T1);
    rospy.Subscriber('/T2_controller/state', JointState, callback_T2); 
    rospy.Subscriber('/T3_controller/state', JointState, callback_T3);
    rospy.Subscriber('/T4_controller/state', JointState, callback_T4);  
    rospy.Subscriber('/T5_controller/state', JointState, callback_T5); 

    #Right_leg
    rospy.Subscriber('/RL1_controller/state', JointState, callback_RL1); 
    rospy.Subscriber('/RL2_controller/state', JointState, callback_RL2); 
    rospy.Subscriber('/RL3_controller/state', JointState, callback_RL3); 
    rospy.Subscriber('/RL4_controller/state', JointState, callback_RL4);

    #Left_leg
    rospy.Subscriber('/LL1_controller/state', JointState, callback_LL1); 
    rospy.Subscriber('/LL2_controller/state', JointState, callback_LL2); 
    rospy.Subscriber('/LL3_controller/state', JointState, callback_LL3); 
    rospy.Subscriber('/LL4_controller/state', JointState, callback_LL4);
    rospy.Subscriber('/LL5_controller/state', JointState, callback_LL5);
    rospy.Subscriber('/LL6_controller/state', JointState, callback_LL6);   

    #Head
    rospy.Subscriber('/H1_controller/state', JointState, callback_H1); 
    rospy.Subscriber('/H2_controller/state', JointState, callback_H2); 

    rospy.loginfo( "Feedback Server Started");
    rospy.spin();
	
#################################################################################

if __name__ == "__main__":
 try:
    fetch_motor_data_server();
 except rospy.ROSInterruptException:
    pass
        
