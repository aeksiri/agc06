#!/usr/bin/env python
# license removed for brevity
import rospy
from tf.transformations import euler_from_quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers

def newAlvarMarkers (msg):

    global tag_id

    tag_id = id

sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, newAlvarMarkers)
rospy.init_node('ar_read', anonymous=True)

def read_ar():
    
    while not rospy.is_shutdown():
        rospy.loginfo("tag_id : %f" % tag_id)
    
if __name__ == '__main__':
    try:
        read_ar()
            
    except rospy.ROSInterruptException:
        pass
