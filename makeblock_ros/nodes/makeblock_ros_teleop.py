#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32
from megapi import *
from makeblock_ros.srv import *
from time import sleep


enc1_last=0
enc2_last=0


bot = None

def onRead_ultrasonic(v):
    rospy.loginfo(v)
    pub_usonic.publish(v)

def onRead_enc_1(v):
    rospy.loginfo(v)
    enc1=v
    global enc1_last
    delta_1 = enc1 - enc1_last
    print "DEBUG speed enc1 ", delta_1
    pub_enc1.publish(v)
    pub_enc1_speed.publish(delta_1)
    enc1_last=enc1

def onRead_enc_2(v):
    rospy.loginfo(v)
    enc2=v
    global enc2_last
    delta_2 = enc2 - enc2_last
    print "DEBUG speed enc2 ", delta_2
    pub_enc2.publish(v)
    pub_enc2_speed.publish(delta_2)
    enc2_last=enc2




def moveRobot(req):
    global bot
    speed1=req.s1
    speed2=req.s2
    print "DEBUG  move ", speed1 , " ", speed2 , " clicks/sec" #DEBUG
    bot.encoderMotorRun(1, req.s1)
    bot.encoderMotorRun(2, req.s2)
    return 1


pub_usonic = rospy.Publisher('makeblock_ros_ultrasensor', Float32, queue_size=1)
pub_enc1 = rospy.Publisher('rosbot_encoder_1', Float32, queue_size=1)
pub_enc1_speed = rospy.Publisher('rosbot_encoder_1_speed', Float32, queue_size=1)
pub_enc2 = rospy.Publisher('rosbot_encoder_2', Float32, queue_size=1)
pub_enc2_speed = rospy.Publisher('rosbot_encoder_2_speed', Float32, queue_size=1)

s = rospy.Service('makeblock_ros_move_robot', MakeBlockMover,
                  moveRobot)


def main():
    global bot
    bot = MegaPi()
    bot.start("/dev/ttyUSB0")
    rospy.init_node('makeblock_ros', anonymous=False)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        sleep(0.1)
        #bot.ultrasonicSensorRead(6, onRead_ultrasonic)
        bot.encoderMotorPosition( 1, onRead_enc_1 )
        bot.encoderMotorPosition( 2, onRead_enc_2 )


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
