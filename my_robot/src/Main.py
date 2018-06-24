#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from sensor_msgs.msg import LaserScan

def Print_data(msg):
    print msg.ranges

def laser_listener():
    rospy.init_node('laser_listener', anonymous=True)
    rospy.Subscriber("/scan",LaserScan, Print_data)
    rospy.spin()

if __name__ == '__main__':
    laser_listener()
