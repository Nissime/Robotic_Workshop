#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
#from geometry_msgs.msg import Point, Pose, Twist, Vector3
from math import sin, cos, pi , atan2 , sqrt
from makeblock_ros.srv import *
from geometry_msgs.msg import Twist
from time import sleep


print ("waiting for service: move_rotate_robot")
rospy.wait_for_service('move_rotate_robot')
print ("found service: move_rotate_robot")

move_rotate_service = rospy.ServiceProxy('move_rotate_robot', TwoFloats)


start_x=0
start_y=0


def get_WC_orientaion (msg):
    global current_angle
    current_angle=msg.data

def get_WC_position (msg):
    global posx,posy,start_x,start_y,current_dist
    posx = msg.pose.pose.position.x
    posy = msg.pose.pose.position.y
    current_dist=sqrt(pow(posy-start_y,2)+pow(posx-start_x,2))

def calc_pose(req):
    global target_y,target_x,start_x,start_y,start_angle,current_angle,target_heading,target_dist

    start_x=posx
    start_y=posy
    start_angle=current_angle
    
    target_x=req.s1
    target_y=req.s2
    
    #calculate heading and range
    target_heading=0.7
    target_dist=30

    print "------pose request-------"
    print "rotate angle" , target_heading , "radians"
    print "move distance" , target_dist , "mm"
    #call tom
    move_rotate_service(target_dist,target_heading)
    return 1




s = rospy.Service('gotoPos_robot', TwoFloats, calc_pose)
rospy.init_node('xy_navigator')

rospy.Subscriber("yaw_angle" , Float32 , get_WC_orientaion)
rospy.Subscriber ('odom', Odometry, get_WC_position)

rospy.spin()
