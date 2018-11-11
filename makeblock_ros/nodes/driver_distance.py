#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist, Vector3
from math import sin, cos, pi , atan2 , sqrt
from time import sleep


def get_current_position (msg):
    #read the current X , Y  from the ODOM message into global variables 
    #this updates (only) every time the ODOM message is published
    global current_x,current_y
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

def move(msg):
    #check that we have odometry. if not - exit
    try:
        current_x 
    except NameError:
        print "error: can't move - no odometry data"
        return 0

    #set up default speed and threshold for end condition
    linear_speed=0.1
    linear_threshold=0.015

    #read the requested distance from the NAV message. in our message it is defined as the "x" value.
    req_distance=msg.x

    #pick the rotation direction according to the incoming message 
    if req_distance < 0:
        linear_speed= -1*linear_speed
        req_distance=abs(req_distance)

    #take snapshot of starting position for distance measurement
    start_x=current_x
    start_y=current_y
    driven_distance=0

    #initialize the cmd_vel message for moving straight (message type is Twist)
    vel_msg = Twist()
    vel_msg.linear.x = linear_speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    print "got message! moving" , req_distance , "m at speed" , linear_speed , "m/s"
    sleep(0.5)
    while(abs(req_distance-driven_distance) > linear_threshold):
        #Publish the linear velocity - forward only
        velocity_publisher.publish(vel_msg)
        #calculate the driven distance (for end condition)
        driven_distance=sqrt(pow(current_x-start_x,2)+pow(current_y-start_y,2))
        #debug message to screen
        print req_distance-driven_distance, "to go"
        sleep(0.1)
        
    #When end condition is met - we are close enough, stop moving
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    print "stopped"


rospy.init_node('driver')
rospy.Subscriber("nav_delta", Vector3, move)
rospy.Subscriber ('odom', Odometry, get_current_position)

    
velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

rospy.spin()
