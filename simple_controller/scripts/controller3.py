#!/usr/bin/env python
# license removed for brevity
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

def move2xy(x_goal, y_goal):
    sub = rospy.Subscriber("/odom", Odometry, newOdom)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.init_node('move2xy', anonymous=True)

    speed = Twist ()

    rate = rospy.Rate(10)

    goal = Point ()

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        goal.x = x_goal
        goal.y = y_goal

    	rospy.loginfo("moveto x: %f" % goal.x)
    	rospy.loginfo("moveto y: %f" % goal.y)

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
        	rate.sleep()
        return


if __name__ == '__main__':
    try:
        move2xy(1,0)
        move2xy(1,3)
    except rospy.ROSInterruptException:
        pass


