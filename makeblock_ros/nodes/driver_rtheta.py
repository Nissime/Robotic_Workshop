#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist, Vector3
from tf.transformations import euler_from_quaternion
from math import sin, cos, pi , atan2 , sqrt
from time import sleep


def sign(x):
    if x > 0:
        return 1.
    elif x < 0:
        return -1.
    elif x == 0:
        return 0.

def get_current_position_orientation (msg):
    #read the current X , Y and yaw from the ODOM message into global variables
    global current_x,current_y, current_yaw
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    current_yaw=yaw

def yaw_delta(delta)
    if delta > pi:
        turn_angle=turn_angle-2*pi
    if delta < -pi:
        return delta

def move(msg):
    linear_speed=0.1
    linear_threshold=0.015
    angular_speed=0.8
    angular_threshold=0.1

    #read the requested angle,distance. 
    req_distance=msg.x
    turn_angle=msg.z

    #pick the rotation direction according to the incoming message 
    if turn_angle > 0:
        angular_speed= -1*angular_speed

    #take snapshot of starting position for distance measurement
    start_x=current_x
    start_y=current_y
    driven_distance=0
    start_angle=current_yaw

    #initialize the cmd_vel message (message type is Twist)
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0


    print "current yaw ", current_yaw, "required yaw" , current_yaw+turn_angle , "radians"
    while abs(turn_angle - current_yaw) > angular_threshold :
        #Publish the angular velocity in the proper direction
        vel_msg.angular.z = sign(turn_angle - current_yaw)*angular_speed
        velocity_publisher.publish(vel_msg)
        print turn_angle-current_yaw , "rads to go, speed" , vel_msg.angular.z
        sleep(0.1)

    #When we are close enough, stop turning
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

    print "step 2-moving" , req_distance , "m"
    while((req_distance-driven_distance) > linear_threshold):
        #Publish the linear velocity - forward only
        vel_msg.linear.x = linear_speed
        velocity_publisher.publish(vel_msg)
        #calculate the driven distance
        driven_distance=sqrt(pow(current_x-start_x,2)+pow(current_y-start_y,2))
        print req_distance-driven_distance, "to go"
        sleep(0.1)
        
    #When we are close enough, stop moving
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    print "done"


rospy.init_node('driver')
rospy.Subscriber("nav_delta", Vector3, move)
rospy.Subscriber ('odom', Odometry, get_current_position_orientation)

    
velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

rospy.spin()
