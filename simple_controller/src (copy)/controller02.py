#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sin, sqrt

x = 0.0
y = 0.0
yaw = 0.0

def newOdom (msg):
    global x
    global y
    global yaw

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node ("speed_controller")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

speed = Twist ()

r = rospy.Rate(4)

goal = Point ()

goal.x = 2.0
goal.y = 0.0
inc_x = goal.x - x
inc_y = goal.y - y

while not rospy.is_shutdown():
    inc_x = goal.x - x
    inc_y = goal.y - y
   
    angle_to_goal = atan2 (inc_y, inc_x)
    
    if abs(angle_to_goal - yaw) > 0.15:
        speed.linear.x = 0.0
        speed.angular.z = 0.9
    else:
        speed.linear.x = 0.0
        speed.angular.z = -0.9

pub.publish(speed)
r.sleep()