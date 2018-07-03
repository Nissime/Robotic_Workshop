#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def Print_data_odom(msg):
    print msg.ranges

def odometer_listener():
    rospy.Subscriber("/husky_velocity_controller/odom",Odometry, Print_data_odom)


def main():
    rospy.init_node('odom_listener', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        odometer_listener()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
