#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def Print_data(msg):
    print msg.ranges

def laser_listener():
    rospy.Subscriber("/scan",LaserScan, Print_data)

def main():
    rospy.init_node('laser_listener', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        laser_listener()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
