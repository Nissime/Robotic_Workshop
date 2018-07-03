#!/usr/bin/env python

import rospy
from makeblock_ros.srv import *
from std_msgs.msg import Float32
from time import sleep


rospy.init_node('timer')

def timer_callback(event):
    print 'Timer called at ' + str(event.current_real)

rospy.Timer(rospy.Duration(2), timer_callback)

rospy.spin()
