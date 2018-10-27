#!/usr/bin/env python


import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from math import copysign
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#################### nav_go ###############################
# import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
###########################################################

#################### VOICE ################################
from std_msgs.msg import Empty
###########################################################

#################### AGC04 Roller #########################
from std_msgs.msg import String
###########################################################



class ARFollower():

    def __init__(self, tag_id ,x_goal, y_goal):

        #rospy.init_node("ar_follower", anonymous=False)

        self.data_target_offset_x = 1
        self.data_target_offset_y = 1

        count_timeout = 0

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # How often should we update the robot's motion?
        self.rate = rospy.get_param("~rate", 20)
        r = rospy.Rate(self.rate) 
        
        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.1)  #0.095
        
        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.1)
        
        # The maximum distance a target can be from the robot for us to track
        self.max_x = rospy.get_param("~max_x", 4.0)
        
        # The goal distance (in meters) to keep between the robot and the marker
        #self.goal_x = rospy.get_param("~goal_x", 0.6)
        self.goal_x = rospy.get_param("~goal_x", x_goal)

        # How far away from the goal distance (in meters) before the robot reacts
        self.x_threshold = rospy.get_param("~x_threshold", 0.04)
        
        # How far away from being centered (y displacement) on the AR marker
        # before the robot reacts (units are meters)
        self.y_threshold = rospy.get_param("~y_threshold", y_goal)    #0.05
        
        # How much do we weight the goal distance (x) when making a movement
        self.x_scale = rospy.get_param("~x_scale", 0.5)  #0.5

        # How much do we weight y-displacement when making a movement        
        self.y_scale = rospy.get_param("~y_scale", 1.0) # 1.0
        
        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.15) #0.073
        
        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.057)

        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # Intialize the movement command
        self.move_cmd = Twist()
        
        # Set flag to indicate when the AR marker is visible
        self.target_visible = False
        
        # Wait for the ar_pose_marker topic to become available
        rospy.loginfo("Waiting for ar_pose_marker topic...")
        rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
        
        # Subscribe to the ar_pose_marker topic to get the image width and height
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.set_cmd_vel)
        
        rospy.loginfo("Marker messages detected. Starting follower...")
        
        self.target_id = tag_id




        # Begin the cmd_vel publishing loop
        while not rospy.is_shutdown():
           # while self.marker.pose.pose.position.x > x_goal:

            # Send the Twist command to the robot
            self.cmd_vel_pub.publish(self.move_cmd)
        
            # Sleep for 1/self.rate seconds

            #rospy.loginfo(" data_target_offset_x = %s  ,  %s", self.data_target_offset_x,self.data_target_offset_y)
            if (self.data_target_offset_x == 0) and (self.data_target_offset_y == 0): 
                if count_timeout > 50:    # 1/self.rate x timeout = T
                    #rospy.loginfo(" return -----------------------")
                    return
                else:
                    count_timeout = count_timeout + 1
                    #rospy.loginfo(" count_timeout %s  = ", count_timeout)
            else:
                count_timeout = 0
                #rospy.loginfo(" count_timeout %s  = ", count_timeout)

            r.sleep()

            
    def set_cmd_vel(self, msg):
        # Pick off the first marker (in case there is more than one)
        try:
            marker = msg.markers[0]

            #### Extract the ID ####################################
            n = len(msg.markers)
            #rospy.loginfo("Number of Tags = %s", n)

            if n == 0:
                return
            
            elif n == 1:
                buffer_marker0 = msg.markers[0]
                if buffer_marker0.id == self.target_id:
                    marker = buffer_marker0
                else:
                    pass

            elif n == 2:
                buffer_marker0 = msg.markers[0]
                buffer_marker1 = msg.markers[1]

                if buffer_marker0.id == self.target_id:
                    marker = buffer_marker0
                elif buffer_marker1.id == self.target_id:
                    marker = buffer_marker1
                else:
                    pass

            elif n == 3:
                buffer_marker0 = msg.markers[0]
                buffer_marker1 = msg.markers[1]
                buffer_marker2 = msg.markers[2]

                if buffer_marker0.id == self.target_id:
                    marker = buffer_marker0
                elif buffer_marker1.id == self.target_id:
                    marker = buffer_marker1
                elif buffer_marker2.id == self.target_id:
                    marker = buffer_marker2
                else:
                    pass

            elif n == 4:
                buffer_marker0 = msg.markers[0]
                buffer_marker1 = msg.markers[1]
                buffer_marker2 = msg.markers[2]
                buffer_marker3 = msg.markers[3]

                if buffer_marker0.id == self.target_id:
                    marker = buffer_marker0
                elif buffer_marker1.id == self.target_id:
                    marker = buffer_marker1
                elif buffer_marker2.id == self.target_id:
                    marker = buffer_marker2
                elif buffer_marker3.id == self.target_id:
                    marker = buffer_marker3
                else:
                    pass

            else:
                pass

            #rospy.loginfo("Marker ID = %s", marker.id)
            #rospy.loginfo("Marker x = %s", marker.pose.pose.position.x)
            #### END Extract the ID ####################################

            if not self.target_visible:
                rospy.loginfo("FOLLOWER is Tracking Target!")
            self.target_visible = True
        except:
            # If target is loar, stop the robot by slowing it incrementally
            self.move_cmd.linear.x /= 1.5
            self.move_cmd.angular.z /= 1.5
            
            if self.target_visible:
                rospy.loginfo("FOLLOWER LOST Target!")
            self.target_visible = False
            
            return
                
        # Get the displacement of the marker relative to the base
        target_offset_y = marker.pose.pose.position.y
        # Get the distance of the marker from the base
        target_offset_x = marker.pose.pose.position.x




        # Rotate the robot only if the displacement of the target exceeds the threshold
        self.data_target_offset_x = target_offset_x
        self.data_target_offset_y = target_offset_y

        if abs(target_offset_y) > self.y_threshold:
            # Set the rotation speed proportional to the displacement of the target
            speed = target_offset_y * self.y_scale
            #speed = speed * -1
            self.move_cmd.angular.z = copysign(max(self.min_angular_speed, min(self.max_angular_speed, speed)), speed)
            self.data_target_offset_y = 1

        else:
            self.move_cmd.angular.z = 0.0
            self.data_target_offset_y = 0

 
        # Now get the linear speed
        if abs(target_offset_x - self.goal_x) > self.x_threshold:
            speed = (target_offset_x - self.goal_x) * self.x_scale
            if speed < 0:
                speed *= 1.5
            speed = speed * -1
            self.move_cmd.linear.x = copysign(min(self.max_linear_speed, max(self.min_linear_speed, speed)), speed)
            #self.move_cmd.linear.x = copysign(min(self.max_linear_speed, max(self.min_linear_speed, abs(speed))), speed)
            self.data_target_offset_x = 1

        else:
            self.move_cmd.linear.x = 0.0
            self.data_target_offset_x = 0
        
        # if abs(target_offset_x - self.goal_x) < self.x_threshold:
        #     rospy.loginfo("xxxxxxxxxxxxx")
        #     self.flag = 0
        #     return self.flag
            
    
    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.loginfo("AR_Marker is stoped...")
        rospy.sleep(1)


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
        rospy.loginfo("AGC04-GOTOPOSE is Stoped")
        rospy.sleep(1)


class AGC04Roller ():
    def __init__(self, roller_command):

        cmd_load = "load"
        cmd_unload = "unload"
        cmd_stop = "stop"
        cmd_mload = "m_load"
        cmd_munload = "m_unload"

        self.status_complete_load = "complete_load"
        self.status_complete_unload = "complete_unload"
        self.status_no_object = "roller_no_object"
        self.status_loading = "roller_loading"
        self.status_unloading = "roller_unloading"
        self.status_idle = "roller_idle"

        # Set the shutdown function (stop the roller)
        rospy.on_shutdown(self.shutdown)
        
        # How often should we update the agc04 roller?
        self.rate = rospy.get_param("~rate", 1)
        r = rospy.Rate(self.rate)

        ###################### Subscribe from AGC04 Roller ###################################################
        rospy.Subscriber('agc04_roller_status', String, self.status)
        ###################### Publish to AGC04 Roller #######################################################
        self.agc04_roller_cmd_pub = rospy.Publisher('agc04_roller_cmd', String, queue_size=1)
        ######################################################################################################

        # Begin the cmd_vel publishing loop
        while not rospy.is_shutdown():
      
            #rospy.loginfo("roller status : %s ", self.roller_status)
            #rospy.loginfo("roller command : %s ", roller_command)
            rospy.sleep(1)

            if (roller_command == cmd_load):
                self.agc04_roller_cmd_pub.publish(cmd_load)
                rospy.sleep(1)
                # while not rospy.is_shutdown():
                #     rospy.loginfo("roller status : %s ", self.roller_status)
                #     if self.roller_status==self.status_complete_load:
                #         rospy.loginfo("Done Loading")
                #         r.sleep()
                #         return
                #     elif self.roller_status==self.status_loading:
                #         rospy.loginfo("AGC04 Roller is Loading")
                #         r.sleep()
                #rospy.sleep(15)
                #rospy.loginfo("Done Loading")
                return
            elif (roller_command == cmd_unload):
                rospy.loginfo("AGC04 Roller is Unloading")
                self.agc04_roller_cmd_pub.publish(cmd_unload)
                rospy.sleep(1)
                # while not rospy.is_shutdown():
                #     rospy.loginfo("roller status : %s ", self.roller_status)
                #     if self.roller_status==self.status_complete_unload:
                #         rospy.loginfo("Done Un-Loading")
                #         r.sleep()
                #         return
                #     elif self.roller_status==self.status_unloading:
                #         rospy.loginfo("AGC04 Roller is Un-Loading")
                #         r.sleep()
                #rospy.sleep(15)
                #rospy.loginfo("Done Loading")
                return
            else:
                #rospy.loginfo("AGC04 Roller is Stoping")
                #agc04_roller_stop_pub.publish()
                #rospy.sleep(5)
                #while not self.roller_status == "agc04_roller_complete_load_out":
                #    rospy.sleep(1)
                #rospy.sleep(15)
                #rospy.loginfo("Done Unloading")
                return

            r.sleep()

    def status(self, msg):
        
        self.roller_status = msg.data
        global status_of_agc04_roller
        status_of_agc04_roller = self.roller_status
        
        # temp_data = msg.data
        # #rospy.loginfo("AGC04 Roller Status : %s", temp_data)
        # if temp_data == "agc04_roller_complete_load_in":
        #     self.roller_status = self.status_load_done
        # elif temp_data == "agc04_roller_complete_load_out":
        #     self.roller_status = self.status_unload_done
        # elif temp_data == "agc04_roller_idle":
        #     self.roller_status = self.status_idle
        # else:
        #     self.roller_status = self.status_unknow
        # #rospy.loginfo("AGC04 Status : %s", self.roller_status)


    def shutdown(self):
        rospy.loginfo("AGC04 Roller is stoped")
        rospy.sleep(1)

class Roller ():
    def __init__(self, roller_id, roller_command):

        cmd_load = "load"
        cmd_unload = "unload"
        cmd_stop = "stop"
        cmd_mload = "mload"
        cmd_munload = "munload"

        self.status_complete_load = "complete_load"
        self.status_complete_unload = "complete_unload"
        self.status_no_object = "roller_no_object"
        self.status_loading = "roller_loading"
        self.status_unloading = "roller_unloading"
        self.status_idle = "roller_idle"

        # Set the shutdown function (stop the roller)
        rospy.on_shutdown(self.shutdown)
        
        # How often should we update the rollers?
        self.rate = rospy.get_param("~rate", 1)
        r = rospy.Rate(self.rate)

        # ###################### Subscribe from Roller10 ###################################################
        # rospy.Subscriber('roller10_status', String, self.roller10_status)
        # ###################### Publish to Roller13 #######################################################
        # self.roller10_cmd_pub = rospy.Publisher('roller10_cmd', String, queue_size=1)
        # ##################################################################################################

        # ###################### Subscribe from Roller11 ###################################################
        # rospy.Subscriber('roller11_status', String, self.roller11_status)
        # ###################### Publish to Roller14 #######################################################
        # self.roller11_cmd_pub = rospy.Publisher('roller11_cmd', String, queue_size=1)
        # ##################################################################################################

        # ###################### Subscribe from Roller12 ###################################################
        # rospy.Subscriber('roller12_status', String, self.roller15_status)
        # ###################### Publish to Roller15 #######################################################
        # self.roller12_cmd_pub = rospy.Publisher('roller12_cmd', String, queue_size=1)
        # ##################################################################################################

        ###################### Subscribe from Roller13 ###################################################
        rospy.Subscriber('roller13_status', String, self.roller13_status)
        ###################### Publish to Roller13 #######################################################
        self.roller13_cmd_pub = rospy.Publisher('roller13_cmd', String, queue_size=1)
        ##################################################################################################

        ###################### Subscribe from Roller14 ###################################################
        rospy.Subscriber('roller14_status', String, self.roller14_status)
        ###################### Publish to Roller14 #######################################################
        self.roller14_cmd_pub = rospy.Publisher('roller14_cmd', String, queue_size=1)
        ##################################################################################################

        ###################### Subscribe from Roller15 ###################################################
        rospy.Subscriber('roller15_status', String, self.roller15_status)
        ###################### Publish to Roller15 #######################################################
        self.roller15_cmd_pub = rospy.Publisher('roller15_cmd', String, queue_size=1)
        ##################################################################################################

        # Begin the cmd_vel publishing loop
        while not rospy.is_shutdown():
      
            rospy.sleep(1)

            # if ((roller_id == 10) and (roller_command == cmd_load)):
            #     self.roller10_cmd_pub.publish(cmd_load)
            #     rospy.sleep(1)
            #     return
            # elif ((roller_id == 10) and (roller_command == cmd_unload)):
            #     self.roller10_cmd_pub.publish(cmd_unload)
            #     rospy.sleep(1)
            #     return
            # elif ((roller_id == 11) and (roller_command == cmd_load)):
            #     self.roller11_cmd_pub.publish(cmd_load)
            #     rospy.sleep(1)
            #     return
            # elif ((roller_id == 11) and (roller_command == cmd_unload)):
            #     self.roller11_cmd_pub.publish(cmd_unload)
            #     rospy.sleep(1)
            #     return
            # elif ((roller_id == 12) and (roller_command == cmd_load)):
            #     self.roller12_cmd_pub.publish(cmd_load)
            #     rospy.sleep(1)
            #     return
            # elif ((roller_id == 12) and (roller_command == cmd_unload)):
            #     self.roller15_cmd_pub.publish(cmd_unload)
            #     rospy.sleep(1)
            #     return
            if ((roller_id == 13) and (roller_command == cmd_load)):
                self.roller13_cmd_pub.publish(cmd_load)
                rospy.sleep(1)
                return
            elif ((roller_id == 13) and (roller_command == cmd_unload)):
                self.roller13_cmd_pub.publish(cmd_unload)
                rospy.sleep(1)
                return
            elif ((roller_id == 14) and (roller_command == cmd_load)):
                self.roller14_cmd_pub.publish(cmd_load)
                rospy.sleep(1)
                return
            elif ((roller_id == 14) and (roller_command == cmd_unload)):
                self.roller14_cmd_pub.publish(cmd_unload)
                rospy.sleep(1)
                return
            elif ((roller_id == 15) and (roller_command == cmd_load)):
                self.roller15_cmd_pub.publish(cmd_load)
                rospy.sleep(1)
                return
            elif ((roller_id == 15) and (roller_command == cmd_unload)):
                self.roller15_cmd_pub.publish(cmd_unload)
                rospy.sleep(1)
                return
            else:
                return

            r.sleep()

    # def roller10_status(self, msg):
    #     self.roller10_status = msg.data
    #     global status_of_roller10
    #     status_of_roller10 = self.roller10_status

    # def roller11_status(self, msg):
    #     self.roller11_status = msg.data
    #     global status_of_roller11
    #     status_of_roller11 = self.roller11_status

    # def roller12_status(self, msg):
    #     self.roller12_status = msg.data
    #     global status_of_roller12
    #     status_of_roller12 = self.roller12_status

    def roller13_status(self, msg):
        self.roller13_status = msg.data
        global status_of_roller13
        status_of_roller13 = self.roller13_status

    def roller14_status(self, msg):
        self.roller14_status = msg.data
        global status_of_roller14
        status_of_roller14 = self.roller14_status

    def roller15_status(self, msg):
        self.roller15_status = msg.data
        global status_of_roller15
        status_of_roller15 = self.roller15_status        

    def shutdown(self):
        # self.roller10_cmd_pub.publish(cmd_stop)
        # self.roller11_cmd_pub.publish(cmd_stop)
        # self.roller12_cmd_pub.publish(cmd_stop)
        rospy.loginfo("Rollers 10,11,12 are stoped")
        self.roller13_cmd_pub.publish(cmd_stop)
        self.roller14_cmd_pub.publish(cmd_stop)
        self.roller15_cmd_pub.publish(cmd_stop)
        rospy.loginfo("Rollers 13,14,15 are stoped")
        rospy.sleep(1)

class AGC04_COMMAND() :
    def __init__(self, status):

        global sending_agc04_status
        sending_agc04_status = status

        # Set the shutdown function (stop the agc04 command)
        rospy.on_shutdown(self.shutdown)
        
        # How often should we update the rollers?
        self.rate = rospy.get_param("~rate", 1)
        self.r = rospy.Rate(self.rate)

        ######################### Subscribe from AGC02 ###################################################
        rospy.Subscriber('agc04_cmd', String, self.recieved_cmd)
        ######################### Publish to AGC02 #######################################################
        self.roller10_cmd_pub = rospy.Publisher('agc04_status', String, queue_size=1)
        ##################################################################################################

        # Begin the agc04_status publishing loop
        while not rospy.is_shutdown():
            self.roller10_cmd_pub.publish(sending_agc04_status)
            self.r.sleep()
            return

    def recieved_cmd(self, msg):
        #global received_agc04_cmd
        temp_cmd = msg.data
        self.r.sleep()
        return temp_cmd

    def shutdown(self):
        rospy.loginfo("AGC04_CMD stopped")
        rospy.sleep(1)

def agc04_load_in(roller_id):
    AGC04Roller("load")
    rospy.sleep(1)
    Roller(roller_id, "unload")
    #rospy.sleep(15)
    flag = False
    while not ( rospy.is_shutdown() or flag ):
        rospy.loginfo("AGC04 roller status : %s " % status_of_agc04_roller)
        rospy.loginfo("Roller13 status : %s " % status_of_roller13)
        if (status_of_roller13 == "roller13_complete_unload") and (status_of_agc04_roller == "complete_load"):
            rospy.loginfo("AGC04-Loading is Done.")
            rospy.sleep(1)
            flag = True
        else:
            rospy.loginfo("AGC04 are loading the object.")
            rospy.sleep(1)
            flag = False
    return

def agc04_load_out(roller_id):
    Roller(roller_id, "load")
    rospy.sleep(1)
    AGC04Roller("unload")
    #rospy.sleep(15)
    flag = False
    while not ( rospy.is_shutdown() or flag ):
        rospy.loginfo("AGC04 roller status : %s " % status_of_agc04_roller)
        rospy.loginfo("Roller14 status : %s " % status_of_roller14)
        if (status_of_roller14 == "roller14_complete_load") and (status_of_agc04_roller == "complete_unload"):
            rospy.loginfo("Done Un-Loading.")
            rospy.sleep(1)
            flag = True
        else:
            rospy.loginfo("AGC04 is Un-Loading the object.")
            rospy.sleep(1)
            flag = False
    return

if __name__ == '__main__':
    try:
        rospy.init_node('agc04_task_scheduler', anonymous=False)

        # temp_cmd = AGC04_COMMAND()

        # AGC04_COMMAND("wait_a_minute")

        # received_agc04_cmd = temp_cmd.recieved_cmd()

        ###################### VOICES publisher  ##################################
        voice1_on_pub = rospy.Publisher('VOICE1_ON', Empty, queue_size=1)
        voice2_on_pub = rospy.Publisher('VOICE2_ON', Empty, queue_size=1)
        voice3_on_pub = rospy.Publisher('VOICE3_ON', Empty, queue_size=1)
        voice4_on_pub = rospy.Publisher('VOICE4_ON', Empty, queue_size=1)
        voice_off_pub = rospy.Publisher('VOICE_OFF', Empty, queue_size=1)
        ###########################################################################

        rospy.sleep(1)
        ARFollower(10, 0.55, 0.005)
        rospy.sleep(10)

        while not rospy.is_shutdown():

            agc04_load_in(13)

            #voice1_on_pub.publish()
            ARFollower(13, 0.8, 0.05)
            rospy.sleep(1)
            voice_off_pub.publish()
            rospy.sleep(1)

            #voice2_on_pub.publish()
            #rospy.sleep(1)
            #GoToPose(-0.1,2.85,90)
            GoToPose(-0.1,3.15,-90)
            voice_off_pub.publish()
            rospy.sleep(1)

            #voice1_on_pub.publish()
            ARFollower(14, 0.49, 0.005)
            rospy.sleep(1)
            voice_off_pub.publish()
            rospy.sleep(1)

            agc04_load_out(14)

            #voice1_on_pub.publish()

            ARFollower(14, 0.8, 0.05)
            rospy.sleep(1)
            
            voice_off_pub.publish()
            rospy.sleep(1)

            #voice3_on_pub.publish()
            rospy.sleep(1)

            #GoToPose(0.85,2.85,-90)
            GoToPose(0.85,2.55,90)
            rospy.sleep(1)

            voice_off_pub.publish()
            rospy.sleep(1)

            #voice1_on_pub.publish()

            ARFollower(13, 0.50, 0.005)
            rospy.sleep(1)

            voice_off_pub.publish()
            rospy.sleep(1)

        #rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AGC04 task scheduler node terminated.")

