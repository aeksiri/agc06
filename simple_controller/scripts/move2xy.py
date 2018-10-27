#!/usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sin, sqrt
from std_msgs.msg import String

def newOdom (msg):
    global x
    global y
    global yaw

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

x = 0.0
y = 0.0
yaw = 0.0

def move2xy(x_goal, y_goal):

    rospy.init_node('move2xy', anonymous=True)

    speed = Twist ()

    goal = Point ()

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        goal.x = x_goal
        goal.y = y_goal

        rospy.loginfo("moving to x:%f, " % goal.x + "y:%f" % goal.y)

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
            		if (angle_to_goal - yaw) > 0.15:
                		speed.linear.x = 0.0
                		speed.angular.z = 0.3
            		elif (angle_to_goal - yaw) < -0.15:
                		speed.linear.x = 0.0
                		speed.angular.z = -0.3
            		else:
                		speed.linear.x = 0.25
                		#*sqrt( (inc_x*inc_x) + (inc_y*inc_y) )
                		speed.angular.z = 0.0

        	pub.publish(speed)
        	rate.sleep()
        
        return
    
    return


def rotate(z_goal):
    rospy.init_node('rotate', anonymous=True)

    speed = Twist ()

    goal = Point ()

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        inc_z = abs(z_goal) - yaw
        rospy.loginfo("yaw : %f" % yaw + "inc_z : %f" % inc_z)
 
        # while not (abs(inc_z) < 0.1): 
        #     if (inc_z > 0):
        #         speed.linear.x = 0.0
        #         speed.angular.z = 0.3

        # return


    return



if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            move2xy(2.3, 0.0)
            move2xy(2.3, 2.85)
            move2xy(0.0, 2.85)
            move2xy(0.0, 0.0)
            #rotate(1.5708)
            
    except rospy.ROSInterruptException:
        pass


