#!/usr/bin/env python
'''service_caller ROS Node'''
# license removed for brevity
import rospy
from std_msgs.msg import String
from subprocess import call

def leds_off():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/led_srv", "input: '0'"])
    # print s

if __name__ == '__main__':
    try:
        #iocon_srv_caller()
        
        rospy.init_node('leds_off_srv_caller', anonymous=True)

        #while not rospy.is_shutdown():
            
        leds_off()

    except rospy.ROSInterruptException:
        pass
