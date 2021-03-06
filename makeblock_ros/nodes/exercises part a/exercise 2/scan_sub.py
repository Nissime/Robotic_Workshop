#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
padding=150

def callback(msg):
    ranges=msg.ranges
    mid_index=len(ranges)/2
    min_index=mid_index-padding
    max_index=mid_index+padding
    actual_ranges=[]
    for i in range (min_index,max_index):
        if not np.isinf(ranges[i]):
            actual_ranges.append(ranges[i])

    
    if actual_ranges:
        avg_range=np.mean(actual_ranges)
        dist_publisher.publish(avg_range)
        print avg_range

dist_publisher = rospy.Publisher('front_dist', Float32, queue_size=1)
rospy.init_node('lidar_subscriber')

rospy.Subscriber("/scan", LaserScan, callback)
print "start lidar subscriber"
rospy.spin()
