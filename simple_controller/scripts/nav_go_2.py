#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

from std_srvs.srv import *

class GoToPose():
    def __init__(self, x, y, theta):

        self.goal_sent = False
    
        self.x_goal = x
        self.y_goal = y
        self.theta_goal = theta

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
	
	    # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

	    # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

        self.goto()

    def goto(self):

        rospy.loginfo("Going to x:%f, " % self.x_goal + "y:%f, " % self.y_goal + "theta:%f, " % self.theta_goal)

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = self.x_goal
        goal.target_pose.pose.position.y = self.y_goal
        goal.target_pose.pose.position.z = 0.0

        row = 0.0
        pitch = 0.0
        #theta -> rad
        yaw = math.radians(self.theta_goal)
        #rospy.loginfo("yaw [deg] = [%s]" % self.theta_goal)
        #rospy.loginfo("yaw [rad] = [%s]" % yaw)

        # Quaternion to Euler angles
        rot_q = goal.target_pose.pose.orientation
        (rot_q.x, rot_q.y, rot_q.z, rot_q.w) = quaternion_from_euler(row,pitch,yaw)
        #rospy.loginfo("qx = %s" % rot_q.x + "qy = %s" % rot_q.y + "qz = %s" % rot_q.z + "qw = %s" % rot_q.w)
        #rospy.loginfo("moving to x:%f, " % goal.x + "y:%f" % goal.y)

        goal.target_pose.pose.orientation.x = rot_q.x
        goal.target_pose.pose.orientation.y = rot_q.y
        goal.target_pose.pose.orientation.z = rot_q.z
        goal.target_pose.pose.orientation.w = rot_q.w

	    # Start moving
        self.move_base.send_goal(goal)

	    # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(300)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
            rospy.loginfo("Hooray, reached the desired pose,")
            rospy.sleep(1)
        else:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to reach the desired pose")
            rospy.sleep(1)

        self.goal_sent = False
        return result

    # def clear_costmaps_handler(self):
    #     rospy.loginfo("clear_costmaps_handler")

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        #navigator = GoToPose()

        while not rospy.is_shutdown():

            GoToPose(2.3,0.0,90)
            rospy.sleep(3)

            GoToPose(2.3,2.85,180)
            rospy.sleep(3)            

            GoToPose(0.0,2.85,-90)
            rospy.sleep(3)

            GoToPose(0.0,0.0,0.0)
            rospy.sleep(3)

            #rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

