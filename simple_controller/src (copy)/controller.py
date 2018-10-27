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
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

speed = Twist ()

r = rospy.Rate(10)

goal = Point ()

while not rospy.is_shutdown():

    rospy.loginfo("going to 1.0, 0.0")    
################ 1st point
    goal.x = 1.0
    goal.y = 0.0
    inc_x = goal.x - x
    inc_y = goal.y - y
    
    while not ((abs(inc_x) < 0.03) & (abs(inc_y) < 0.03)):
        
        inc_x = goal.x - x
        inc_y = goal.y - y
    
        angle_to_goal = atan2 (inc_y, inc_x)
    
        if ((abs(inc_x) < 0.03) & (abs(inc_y) < 0.03)):
            speed.linear.x = 0.0
            speed.angular.z = 0.0
        else:
            if (angle_to_goal - yaw) > 0.05:
                speed.linear.x = 0.0
                speed.angular.z = 0.2
            elif (angle_to_goal - yaw) < -0.05:
                speed.linear.x = 0.0
                speed.angular.z = -0.2
            else:
                speed.linear.x = 0.2
                #*sqrt( (inc_x*inc_x) + (inc_y*inc_y) )
                speed.angular.z = 0.0

        pub.publish(speed)
        r.sleep()


    rospy.loginfo("going to 1.0, 3.0")
################ 2nd point
    goal.x = 1.0
    goal.y = 3.0
    inc_x = goal.x - x
    inc_y = goal.y - y
    
    while not ((abs(inc_x) < 0.03) & (abs(inc_y) < 0.03)):
    
        inc_x = goal.x - x
        inc_y = goal.y - y

        angle_to_goal = atan2 (inc_y, inc_x)
    
        if ((abs(inc_x) < 0.03) & (abs(inc_y) < 0.03)):
            speed.linear.x = 0.0
            speed.angular.z = 0.0
        else:
            if (angle_to_goal - yaw) > 0.05:
                speed.linear.x = 0.0
                speed.angular.z = 0.2
            elif (angle_to_goal - yaw) < -0.05:
                speed.linear.x = 0.0
                speed.angular.z = -0.2
            else:
                speed.linear.x = 0.2
                #*sqrt( (inc_x*inc_x) + (inc_y*inc_y) )
                speed.angular.z = 0.0

        pub.publish(speed)
        r.sleep()

    rospy.loginfo("going to 0.0, 3.5")
################ 2.5nd point
    goal.x = 0.0
    goal.y = 3.5
    inc_x = goal.x - x
    inc_y = goal.y - y
    
    while not ((abs(inc_x) < 0.03) & (abs(inc_y) < 0.03)):
    
        inc_x = goal.x - x
        inc_y = goal.y - y

        angle_to_goal = atan2 (inc_y, inc_x)
    
        if ((abs(inc_x) < 0.03) & (abs(inc_y) < 0.03)):
            speed.linear.x = 0.0
            speed.angular.z = 0.0
        else:
            if (angle_to_goal - yaw) > 0.05:
                speed.linear.x = 0.0
                speed.angular.z = 0.2
            elif (angle_to_goal - yaw) < -0.05:
                speed.linear.x = 0.0
                speed.angular.z = -0.2
            else:
                speed.linear.x = 0.2
                #*sqrt( (inc_x*inc_x) + (inc_y*inc_y) )
                speed.angular.z = 0.0

        pub.publish(speed)
        r.sleep()


    rospy.loginfo("going to -1.5, 3.0")
################ 3rd point
    goal.x = -1.5
    goal.y = 3.0
    inc_x = goal.x - x
    inc_y = goal.y - y
    
    while not ((abs(inc_x) < 0.03) & (abs(inc_y) < 0.03)):
    
        inc_x = goal.x - x
        inc_y = goal.y - y

        angle_to_goal = atan2 (inc_y, inc_x)
    
        if ((abs(inc_x) < 0.03) & (abs(inc_y) < 0.03)):
            speed.linear.x = 0.0
            speed.angular.z = 0.0
        else:
            if (angle_to_goal - yaw) > 0.05:
                speed.linear.x = 0.0
                speed.angular.z = 0.2
            elif (angle_to_goal - yaw) < -0.05:
                speed.linear.x = 0.0
                speed.angular.z = -0.2
            else:
                speed.linear.x = 0.2
                #*sqrt( (inc_x*inc_x) + (inc_y*inc_y) )
                speed.angular.z = 0.0

        pub.publish(speed)
        r.sleep()


    rospy.loginfo("going to -1.5, 0.0")
################ 4th point
    goal.x = -1.5
    goal.y = 0.0
    inc_x = goal.x - x
    inc_y = goal.y - y
    
    while not ((abs(inc_x) < 0.03) & (abs(inc_y) < 0.03)):
    
        inc_x = goal.x - x
        inc_y = goal.y - y

        angle_to_goal = atan2 (inc_y, inc_x)
    
        if ((abs(inc_x) < 0.03) & (abs(inc_y) < 0.03)):
            speed.linear.x = 0.0
            speed.angular.z = 0.0
        else:
            if (angle_to_goal - yaw) > 0.05:
                speed.linear.x = 0.0
                speed.angular.z = 0.2
            elif (angle_to_goal - yaw) < -0.05:
                speed.linear.x = 0.0
                speed.angular.z = -0.2
            else:
                speed.linear.x = 0.2
                #*sqrt( (inc_x*inc_x) + (inc_y*inc_y) )
                speed.angular.z = 0.0

        pub.publish(speed)
        r.sleep()

