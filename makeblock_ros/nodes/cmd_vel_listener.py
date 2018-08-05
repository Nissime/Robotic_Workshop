#!/usr/bin/env python
# license removed for brevity
#!/usr/bin/env python
import roslib
 #roslib.load_manifest('YOUR_PACKAGE_NAME_HERE')
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from makeblock_ros.srv import *

robot_wheel_base_mm=85 #wheel distance is 170mm
robot_wheel_radius_mm=31.5
robot_wheel_CPR=368 #clicks per rotation of wheel
pi=3.14
robot_wheel_circum=2*pi*robot_wheel_radius_mm
robot_base_circum=2*pi*robot_wheel_base_mm

print ("waiting for service : makeblock_ros_move_robot")
rospy.wait_for_service('makeblock_ros_move_robot')
print ("found services : makeblock_ros_move_robot")

move = rospy.ServiceProxy('makeblock_ros_move_robot', MakeBlockMover)

def mm_to_clicks(dist):
    clicks=dist/robot_wheel_circum*robot_wheel_CPR
    clicks=round(clicks)
    return clicks

def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    move_mm_sec=msg.linear.x*1000 #conversion from m/s to mm/s
    turn_mm_sec=msg.angular.z * robot_base_circum / (2*pi)
    leftwheel_clicks=mm_to_clicks(move_mm_sec+turn_mm_sec)
    rightwheel_clicks=mm_to_clicks(move_mm_sec-turn_mm_sec)*-1
    print "DEBUG left: ", leftwheel_clicks, " right: " , rightwheel_clicks
    resp=move(leftwheel_clicks,rightwheel_clicks)
    return resp.sum


def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
