#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
#from geometry_msgs.msg import Point, Pose, Twist, Vector3
from math import sin, cos, pi , atan2 , sqrt
from makeblock_ros.srv import *

def get_WC_orientaion (msg):
    global curr_th
    curr_th=msg

def get_WC_position (msg):
    global posx,posy
    posx = msg.pose.pose.position.x
    posy = msg.pose.pose.position.y

def calc_heading(req):
    return 1

def move():
    return 1

s = rospy.Service('navigation', MakeBlockMover, calc_heading)
rospy.init_node('simple_navigator')

rospy.Subscriber("yaw_angle" , Float32 , get_WC_orientaion)
rospy.Subscriber ('odom', Odometry, get_WC_position)

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    try:
        move()
        rate.sleep()
    except rospy.ROSInterruptException: pass
