#!/usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sin, sqrt
from std_msgs.msg import String
from sensor_msgs.msg import Imu

def newImu (msg):

    global yaw

    rot_q = msg.orientation
    (roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

sub = rospy.Subscriber("/imu/data", Imu, newImu)
rospy.init_node('imu_read', anonymous=True)

def read_imu():
    
    while not rospy.is_shutdown():
        rospy.loginfo("imu_yaw : %f" % yaw)
    



if __name__ == '__main__':
    try:
        read_imu()
            
    except rospy.ROSInterruptException:
        pass
