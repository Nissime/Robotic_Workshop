#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
from time import sleep
padding=3

def callback(msg):
    global ranges
    ranges=msg.ranges


def dist(angle):
    angle_index=angle/360
    mid_index=int(len(ranges)*angle_index)
    min_index=mid_index-padding
    max_index=mid_index+padding
    actual_ranges=[]
    #print "array index:", mid_index
    for i in range (min_index,max_index):
        if not np.isinf(ranges[i]):
            actual_ranges.append(ranges[i])
    
    if actual_ranges:
        avg_range=np.mean(actual_ranges)
        dist_publisher.publish(avg_range)
        #print avg_range
    else:
        #print "out of range"
        avg_range=999
    return(avg_range)

def scan():
    for angle in range (5,355,20):
        print "angle ", angle , "range" , dist(float(angle))
    sleep(2)

dist_publisher = rospy.Publisher('front_dist', Float32, queue_size=1)
rospy.init_node('lidar_subscriber')

rospy.Subscriber("/rosbot/laser/scan", LaserScan, callback)
print "start lidar subscriber"
sleep(1)

while not rospy.is_shutdown():
    try:
        scan()
    except rospy.ROSInterruptException: pass

