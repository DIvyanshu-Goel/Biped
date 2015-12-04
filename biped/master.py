#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from dynamixel_msgs.msg import JointState
from biped.msg import Motor_Angle
from math import sin
import time
import os
import sys
sys.path.append(os.getcwd()+"/include");
import head
import torso
import right_arm
import left_arm
import right_leg
import left_leg


t_pos = -.50;
Tf = 50 ; #no of sec * 50
data = 0;
Curr_pos = [1,1,1,1];
update_rate = 50;

rospy.init_node('Main_node', anonymous=True); 
rate = rospy.Rate(update_rate) # 50hz update rate

right_arm.right_arm([0,1.5,0,0],.2);
left_arm.left_arm([0,-1.5,0,0],.2);
torso.torso([0,0,0,0,0],.2);
right_leg.right_leg([0,0,0,0,0,0],.2);
left_leg.left_leg([0,0,0,0,0,0],.2);
head.head([0,0],.2);

right_arm.right_arm([1,1.5,-1.5,1.16],.1);
time.sleep(0.5);
#torso.torso([-0.3,0,0,-.1,0],1);
time.sleep(0.5);
#torso.torso([0,0,0,0,0],1);
time.sleep(0.5);
right_arm.right_arm([0,1.5,0,0],.2);
time.sleep(0.5);
left_arm.left_arm([-1.4,-1.9,1.4,-1.5],.1);
time.sleep(0.5);
left_arm.left_arm([-.93,-1.9,1.4,-1.5],.2);
time.sleep(0.5);
left_arm.left_arm([-1.6,-1.9,1.4,-1.5],.1);
time.sleep(0.5);
right_arm.right_arm([1,1.5,-1.5,1.16],.1);


def master():
    global Curr_pos;
 
  #  while not rospy.is_shutdown():
        

        #time.sleep(0.02);
    #    rospy.spin();


if __name__ == '__main__':
    try:

        master();       

    except rospy.ROSInterruptException:
        pass
