#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float32

robot_wheel_base_mm=85 #wheel distance is 170mm
robot_wheel_radius_mm=31.5
robot_wheel_CPR=368 #clicks per rotation of wheel
pi=3.14
robot_wheel_circum=2*pi*robot_wheel_radius_mm
robot_base_circum=2*pi*robot_wheel_base_mm

enc1_speed=0
enc2_speed=0

rospy.init_node('odometry_publisher')

def enc1_speed_onRead(msg):
    global enc1_speed
    enc1_speed=msg.data

def enc2_speed_onRead(msg):
    global enc2_speed
    enc2_speed=-1*msg.data  #invert one encoder because of robot struct

def calc_speeds():

    global enc1_speed
    global enc2_speed

    print "DEBUG speeds clicks/s ", enc1_speed , "  " , enc2_speed
    delta_1_mm=enc1_speed * robot_wheel_circum /robot_wheel_CPR
    delta_2_mm=enc2_speed * robot_wheel_circum /robot_wheel_CPR
    v_robot=(enc1_speed+enc2_speed)/2
    vth_robot=(delta_2_mm-delta_1_mm) / robot_base_circum * 360
    print "DEBUG speeds robot ", vx , "  mm/s  vth  " , vth_robot , " deg/s"
    return (v_robot,vth_robot)

rospy.Subscriber("rosbot_encoder_1_speed", Float32, enc1_speed_onRead)
rospy.Subscriber("rosbot_encoder_2_speed", Float32, enc2_speed_onRead)

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0
vth = 0.0

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(5.0)
while not rospy.is_shutdown():

    current_time = rospy.Time.now()
    vx,vth_degrees=calc_speeds()
    vth=vth_degrees*pi/180
    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt
    print "DEBUG speeds WC vx ", delta_x , " vy  " , delta_y ," vth ", vth
    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
