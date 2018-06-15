#!/usr/bin/env python

import rospy
from makeblock_ros.srv import *
from std_msgs.msg import Float32
from time import sleep


current_count=0
prev_count=0

rospy.wait_for_service('makeblock_ros_move_motors')
print ("found service: makeblock_ros_move_motors")

motors = rospy.ServiceProxy('makeblock_ros_move_motors', MakeBlockMover)


def encoder2_callback(msg):
    current_count=msg.data
    global prev_count

    if current_count > 1000:
        print "current count " , current_count ,
        print (" go backwards")
        motor_turn(-30,-30)

    if current_count < -1000:
        print "current count " , current_count ,
        print ("go forward")
        motor_turn(30,30)



def motor_turn(s1, s2):
    resp1 = motors (s1, s2)
    print "motor service returned" , resp1.sum


resp1 = motors(30, 30)
print "INIT! motor service returned" , resp1.sum

rospy.init_node('motor_2_controller')

rospy.Subscriber("rosbot_encoder_2", Float32, encoder2_callback)


rospy.spin()
