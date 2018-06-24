#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print "yaw " ,yaw
    pub.publish(yaw)

rospy.init_node('yaw_angle_publisher')
pub = rospy.Publisher('yaw_angle', Float32, queue_size=1)
sub = rospy.Subscriber ('/odom', Odometry, get_rotation)

r = rospy.Rate(10)
while not rospy.is_shutdown():
    r.sleep()
