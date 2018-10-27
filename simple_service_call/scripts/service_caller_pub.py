#!/usr/bin/env python
'''service_caller ROS Node'''
# license removed for brevity
import rospy
from std_msgs.msg import String

import commands
#import subprocess
from subprocess import call

def led_on():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    # print s

def led_off():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###
    
    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '0'"])
    s = call(["rosservice", "call", "/agc04/iocon_srv", "input: '0'"]) 
    # print s

if __name__ == '__main__':
    try:
        #iocon_srv_caller()
        
        rospy.init_node('iocon_srv_caller', anonymous=True)

        while not rospy.is_shutdown():
            
            led_on()
            rospy.sleep(3.0)
            led_off()
            rospy.sleep(3.0)

    except rospy.ROSInterruptException:
        pass
