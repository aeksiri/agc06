#!/usr/bin/env python
'''test ROS Node'''
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker(message):
    '''test Publisher'''
    pub = rospy.Publisher('agc04_status', String, queue_size=10)
    rospy.init_node('test', anonymous=True)
    rate = rospy.Rate(10) # 10hz


    while not rospy.is_shutdown():
        hello_str = message
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        #talker("wait_a_minute")

        talker("clear_to_go")
        #talker("wait_a_minute")
       


    except rospy.ROSInterruptException:
        pass
