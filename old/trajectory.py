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

def callback(data):
    current_value=data.current_pos   
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.current_pos)


def trajectory():
    global Goal_pos,Start_pos,Tf;
    rospy.init_node('trajectory', anonymous=True)
    pub = rospy.Publisher('/RA1_controller/command', Float64, queue_size=10)
#    rospy.Subscriber("/joint4_controller/state", JointState, callback)
    rate = rospy.Rate(50) # 50hz
    timer = 0;
    forw = 1;

    while not rospy.is_shutdown():        
       Curr_pos = Start_pos + ((Goal_pos - Start_pos)/Tf )* ( timer - (1/(2*3.14))* sin(2*3.14*timer/Tf));
       rospy.loginfo(Curr_pos );
       pub.publish(Curr_pos );
       timer = timer + 1;
       if (timer > Tf and forw == 1):
		timer = 0;
		Goal_pos = -.50;
		Start_pos = 0;
		forw = 2;
		time.sleep(1);

       if (timer > Tf and forw == 2):
		timer = 1;
		Goal_pos = 0;
		Start_pos = -0.50;
		forw = 1;
		time.sleep(1);

       rate.sleep();



if __name__ == '__main__':
    try:

        trajectory()

    except rospy.ROSInterruptException:
        pass
