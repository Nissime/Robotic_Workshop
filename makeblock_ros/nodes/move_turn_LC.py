#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from makeblock_ros.srv import *
from math import pi
from time import sleep

target_dist=0.0
target_angle=0.0

def sign(x):
    if x > 0:
        return 1.
    elif x < 0:
        return -1.
    elif x == 0:
        return 0.

def getDistance (msg):
    global target_dist
    target_dist = msg.data

def getAngle (msg):
    global target_angle
    target_angle = msg.data

def move():
    linear_speed=15.0
    angular_speed=2.0
    linear_tolerance=5.0
    angular_tolerance=0.5
    global target_dist
    global target_angle


    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    current_distance = 0.0
    current_angle = 0.0
    print "1-turning" , target_angle * 180/pi , "degrees"
    while abs(current_angle - target_angle) > angular_tolerance:
        #Publish the angular velocity
        vel_msg.angular.z = sign(target_angle)*angular_speed
        velocity_publisher.publish(vel_msg)
        print current_angle - target_angle , "to go, speed" , sign(target_angle)*angular_speed
        sleep(0.5)
    #After the loop, stops the robot
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

    print "2-moving" , target_dist , "mm"
    while(abs(current_distance - target_dist)>linear_tolerance):
        #Publish the linear velocity
        vel_msg.linear.x = sign(target_dist)*linear_speed
        velocity_publisher.publish(vel_msg)
        print current_distance - target_dist, "to go, speed" , sign(target_dist)*linear_speed
        sleep(0.5)
    #After the loop, stops the robot
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    print "done"

rospy.Subscriber("reqDistance_LC" , Float32 , getDistance)
rospy.Subscriber("reqAngle_LC" , Float32 , getAngle)
rospy.init_node('move_turn_LC')

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    try:
        move()
        rate.sleep()
    except rospy.ROSInterruptException: pass
