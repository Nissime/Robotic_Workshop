#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32
from megapi import *
from makeblock_ros.srv import *
from time import sleep

robot_wheel_base_mm=100 #TODO not accurate - need to measure
robot_wheel_radius_mm=31.5
robot_wheel_CPR=368 #clicks per rotation of wheel
pi=3.14
robot_wheel_circum=2*pi*robot_wheel_radius_mm
robot_base_circum=2*pi*robot_wheel_base_mm

bot = None

def onRead_ultrasonic(v):
    rospy.loginfo(v)
    pub_usonic.publish(v)

def onRead_enc_1(v):
    rospy.loginfo(v)
    pub_enc1.publish(v)

def onRead_enc_2(v):
    rospy.loginfo(v)
    pub_enc2.publish(v)

def onFinishMove_m1(v):
    msg=0
    print "DEBUG - finish move" ,v
    pub_m1_inmotion.publish(msg)

def onFinishMove_m2(v):
    msg=0
    print "DEBUG - finish move" ,v
    pub_m2_inmotion.publish(msg)

def moveRobot(req):
    global bot
    clicks=req.s1/robot_wheel_circum*robot_wheel_CPR
    clicks=round(clicks)
    print "DEBUG straight move ", clicks, " clicks" #DEBUG
    bot.encoderMotorMove(1,req.s2,clicks,onFinishMove_m1)
    bot.encoderMotorMove(2,req.s2,-1*clicks,onFinishMove_m2)
    msg=1
    pub_m1_inmotion.publish(msg)
    pub_m2_inmotion.publish(msg)
    return 1

def turnRobot(req):
    global bot
    dist= req.s1 * robot_base_circum / 360
    clicks=dist/robot_wheel_circum*robot_wheel_CPR
    clicks=round(clicks)
    print "DEBUG rotation move ", clicks, " clicks" #DEBUG
    bot.encoderMotorMove(1,req.s2,clicks,onFinishMove_m1)
    bot.encoderMotorMove(2,req.s2,clicks,onFinishMove_m2)
    msg=1
    pub_m1_inmotion.publish(msg)
    pub_m2_inmotion.publish(msg)
    return 1

pub_usonic = rospy.Publisher('makeblock_ros_ultrasensor', Float32, queue_size=1)
pub_enc1 = rospy.Publisher('rosbot_encoder_1', Float32, queue_size=1)
pub_enc2 = rospy.Publisher('rosbot_encoder_2', Float32, queue_size=1)
pub_m1_inmotion = rospy.Publisher('rosbot_m1_inmotion', Float32, queue_size=1)
pub_m2_inmotion = rospy.Publisher('rosbot_m2_inmotion', Float32, queue_size=1)

s = rospy.Service('makeblock_ros_move_robot', MakeBlockMover,
                  moveRobot)

r = rospy.Service('makeblock_ros_turn_robot', MakeBlockMover,
                  turnRobot)

def main():
    global bot
    bot = MegaPi()
    bot.start("/dev/ttyUSB0")
    rospy.init_node('makeblock_ros', anonymous=False)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        sleep(0.2)
        bot.ultrasonicSensorRead(6, onRead_ultrasonic)
        bot.encoderMotorPosition( 1, onRead_enc_1 )
        bot.encoderMotorPosition( 2, onRead_enc_2 )


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
