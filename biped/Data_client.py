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
#For motor mapping to matrix id see Server.py file
def motor_data_client(x):
    rospy.wait_for_service('Fetch_Motor_data')
    client = rospy.ServiceProxy('Fetch_Motor_data', Fetch_Motor_Data)
    resp1 = client(x);
    return resp1



if __name__ == "__main__":
	while(1):
		try:
			mode=int(raw_input('Input:'));	    
		except ValueError:
			print("Not a number");
		if(mode <= 26 and mode != 17 and mode != 18):	
			response1 =motor_data_client(mode);
			print "%s %s %s %s %s"%(response1.motor_ids,response1.current_pos,response1.velocity,response1.load,response1.is_moving);
		else:
			print('Enter value less than 27 and not 17,18');
