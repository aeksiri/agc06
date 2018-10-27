#!/usr/bin/env python
'''service_caller ROS Node'''
# license removed for brevity
import rospy
from std_msgs.msg import String
from subprocess import call

def cart_lock_up():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/bodylock_srv", "input: '1'"])
    # print s

def cart_lock_down():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/bodylock_srv", "input: '0'"])
    # print s

if __name__ == '__main__':
    try:
        #iocon_srv_caller()
        
        rospy.init_node('cart_lock_up_srv_caller', anonymous=True)

        #while not rospy.is_shutdown():
            
        cart_lock_up()

    except rospy.ROSInterruptException:
        pass
