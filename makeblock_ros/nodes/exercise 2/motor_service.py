#!/usr/bin/env python

import rospy
from makeblock_ros.srv import *
from time import sleep

print ("waiting for service: makeblock_ros_move_robot")
rospy.wait_for_service('makeblock_ros_move_robot')
print ("found service: makeblock_ros_move_robot")

motors = rospy.ServiceProxy('makeblock_ros_move_robot', MakeBlockMover)

def motor_turn(s1, s2):
    response = motors (s1, s2)
    print "turning ", s1 , "," ,s2, " returned" , response.sum

rospy.init_node('motor_2_controller')

while not rospy.is_shutdown():
   motor_turn(50,50)
   sleep(1)
   motor_turn(0,0)
   sleep(2)
