#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
#from geometry_msgs.msg import Point, Pose, Twist, Vector3
from math import sin, cos, pi , atan2
from makeblock_ros.srv import *

print "init navigator"

def get_orientaion (msg):
    global curr_th
    curr_th=msg

def get_position (msg):
    global posx,posy
    posx = msg.pose.pose.position.x
    posy = msg.pose.pose.position.y

def moveRobot(req):
    x1=posx
    y1=posy
    x2=req.s1
    y2=req.s2
    th1=curr_th
    target_th=atan2(y2-y1,x2-x1)
    print "required angle" , target_th*180/pi
    return 1

s = rospy.Service('navigate_robot', MakeBlockMover, moveRobot)

rospy.init_node('simple_navigator')

rospy.Subscriber("yaw_angle" , Float32 , get_orientaion)
rospy.Subscriber ('odom', Odometry, get_position)

rospy.spin()
