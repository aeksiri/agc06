#!/usr/bin/env python
'''testmove_AA ROS Node'''
# license removed for brevity
#import nav_go
#import test_ar_follower
#import ar_follower

import rospy
from std_msgs.msg import String

def talker():
    '''testmove_AA Publisher'''
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('testmove_AA', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        #talker()
        xxx = 'nav_go.py'
        run_cmd(xxx)
    except rospy.ROSInterruptException:
        pass
