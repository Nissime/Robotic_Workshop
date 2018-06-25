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
    x1=posx
    y1=posy
    x2=req.s1
    y2=req.s2
    th1=curr_th
    target_th=atan2(y2-y1,x2-x1)
    print "------nav request-------"
    print "required angle" , target_th*180/pi , "degrees"
    target_dist=sqrt(pow(y2-y1,2)+pow(x2-x1,2))
    print "required distance" , target_dist , "mm"
    reqDistance_LC.publish(target_dist)
    reqAngle_LC.publish(target_th)
    return 1

s = rospy.Service('navigation', MakeBlockMover, calc_heading)
reqDistance_LC = rospy.Publisher('reqDistance_LC', Float32, queue_size=1)
reqAngle_LC = rospy.Publisher('reqAngle_LC', Float32, queue_size=1)
rospy.init_node('simple_navigator')

rospy.Subscriber("yaw_angle" , Float32 , get_WC_orientaion)
rospy.Subscriber ('odom', Odometry, get_WC_position)

rospy.spin()
