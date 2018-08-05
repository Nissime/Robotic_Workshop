#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
#from geometry_msgs.msg import Point, Pose, Twist, Vector3
from math import sin, cos, pi , atan2 , sqrt
from makeblock_ros.srv import *
from geometry_msgs.msg import Twist
from time import sleep


def move():
    linear_speed=0.03 #meter/sec!!
    angular_speed=0.15 #radian/sec!


    global target_dist,target_angle,current_dist,current_angle
    
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0


    
    vel_msg.linear.x = linear_speed
    velocity_publisher.publish(vel_msg)
    print "moving" 
    sleep(2)
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    print "stopping"
    sleep(1)
    vel_msg.angular.z = angular_speed
    velocity_publisher.publish(vel_msg)
    print "turning" 
    sleep(2)
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    print "stopping"
    sleep(1)

rospy.init_node('simple_navigator')


while not rospy.is_shutdown():
    try:
        move()
    except rospy.ROSInterruptException: pass


