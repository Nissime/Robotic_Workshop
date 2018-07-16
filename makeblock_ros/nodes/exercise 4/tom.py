#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
#from geometry_msgs.msg import Point, Pose, Twist, Vector3
from math import sin, cos, pi , atan2 , sqrt
from makeblock_ros.srv import *
from geometry_msgs.msg import Twist
from time import sleep

start_x=0
start_y=0

def sign(x):
    if x > 0:
        return 1.
    elif x < 0:
        return -1.
    elif x == 0:
        return 0.

def get_WC_orientaion (msg):
    global current_angle
    current_angle=msg.data

def get_WC_position (msg):
    global posx,posy,start_x,start_y,current_dist
    posx = msg.pose.pose.position.x
    posy = msg.pose.pose.position.y
    current_dist=sqrt(pow(posy-start_y,2)+pow(posx-start_x,2))

def calc_heading(req):
    global target_dist,target_angle,start_x,start_y,start_angle,current_angle

    start_x=posx
    start_y=posy
    start_angle=current_angle
    
    target_dist=req.s1
    target_angle=req.s2
    
    print "------angle/range request-------"
    print "rotate angle" , target_angle , "radians"
    print "move distance" , target_dist , "mm"
    move()
    return 1

def move():
    linear_speed=15.0
    angular_speed=0.15


    global target_dist,target_angle,current_dist,current_angle
    
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0


    print "step 1-turning" , target_angle , "radians"
    while abs(current_angle - start_angle) < abs(target_angle):
        #Publish the angular velocity
        vel_msg.angular.z = sign(target_angle)*angular_speed
        velocity_publisher.publish(vel_msg)
        print abs(current_angle) - abs(target_angle) , "rads to go, speed" , vel_msg.angular.z
        sleep(0.5)

    #After the loop, stops the robot
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

    print "step 2-moving" , target_dist , "mm"
    while(abs(current_dist) < abs(target_dist)):
        #Publish the linear velocity
        vel_msg.linear.x = sign(target_dist)*linear_speed
        velocity_publisher.publish(vel_msg)
        print abs(current_dist) - abs(target_dist), "to go, speed" , sign(target_dist)*linear_speed
        sleep(0.5)
    #After the loop, stops the robot
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    print "done"


s = rospy.Service('move_rotate_robot', TwoFloats, calc_heading)
rospy.init_node('simple_navigator')

rospy.Subscriber("yaw_angle" , Float32 , get_WC_orientaion)
rospy.Subscriber ('odom', Odometry, get_WC_position)

rospy.spin()
