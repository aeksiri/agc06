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
#from std_msgs.msg import Empty
###########################################################
from std_srvs.srv import *

#################### AGC03 Roller #########################
from std_msgs.msg import String
###########################################################



class ARFollower():

    def __init__(self, tag_id ,x_goal, y_goal, time_goal):

        #rospy.init_node("ar_follower", anonymous=False)

        self.data_target_offset_x = 1
        self.data_target_offset_y = 1

        count_timeout = 0
        count_timeoutbreak = 0

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # How often should we update the robot's motion?
        self.rate = rospy.get_param("~rate", 20)
        r = rospy.Rate(self.rate) 
        
        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.1)  #0.1 #0.091
        
        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.1)  #0.1 #0.065  /0.089
        
        # The maximum distance a target can be from the robot for us to track
        self.max_x = rospy.get_param("~max_x", 2.5)
        
        # The goal distance (in meters) to keep between the robot and the marker
        #self.goal_x = rospy.get_param("~goal_x", 0.6)
        self.goal_x = rospy.get_param("~goal_x", x_goal)

        # How far away from the goal distance (in meters) before the robot reacts
        self.x_threshold = rospy.get_param("~x_threshold", 0.02)
        
        # How far away from being centered (y displacement) on the AR marker
        # before the robot reacts (units are meters)
        self.y_threshold = rospy.get_param("~y_threshold", y_goal)    #0.05
        
        # How much do we weight the goal distance (x) when making a movement
        self.x_scale = rospy.get_param("~x_scale", 0.5)  #0.5

        # How much do we weight y-displacement when making a movement        
        self.y_scale = rospy.get_param("~y_scale", 3.0) # 1.0
        
        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.15) #0.15
        
        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.057) #0.057

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
                if (count_timeout >= time_goal) :    # // 50    1/self.rate x timeout = T
                #if count_timeout >= 100:    # // 50    1/self.rate x timeout = T
                    #rospy.loginfo(" return -----------------------")
                    return
                elif (count_timeoutbreak >= 180):
                    rospy.loginfo(" return -------count_timeoutbreak*********")
                    return
                else:
                    count_timeout = count_timeout + 1
                    count_timeoutbreak = count_timeoutbreak + 1
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
        agc03_buzzer_cmd_pub.publish("ch0")


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
        self.move_base.wait_for_server(rospy.Duration(20))  #5

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

        #clear_costmaps(10)

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
        rospy.loginfo("AGC03-GOTOPOSE is Stoped")
        rospy.sleep(1)
        agc03_buzzer_cmd_pub.publish("ch0")


class AGC03Roller ():
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
        
        # How often should we update the agc03 roller?
        self.rate = rospy.get_param("~rate", 1)
        r = rospy.Rate(self.rate)

        ###################### Subscribe from AGC03 Roller ###################################################
        rospy.Subscriber('agc03_roller_status', String, self.status)
        ###################### Publish to AGC03 Roller #######################################################
        self.agc03_roller_cmd_pub = rospy.Publisher('agc03_roller_cmd', String, queue_size=1)
        ######################################################################################################

        # Begin the cmd_vel publishing loop
        while not rospy.is_shutdown():
      
            #rospy.loginfo("roller status : %s ", self.roller_status)
            #rospy.loginfo("roller command : %s ", roller_command)
            rospy.sleep(1)

            if (roller_command == cmd_load):
                self.agc03_roller_cmd_pub.publish(cmd_load)
                rospy.sleep(1)
                # while not rospy.is_shutdown():
                #     rospy.loginfo("roller status : %s ", self.roller_status)
                #     if self.roller_status==self.status_complete_load:
                #         rospy.loginfo("Done Loading")
                #         r.sleep()
                #         return
                #     elif self.roller_status==self.status_loading:
                #         rospy.loginfo("AGC03 Roller is Loading")
                #         r.sleep()
                #rospy.sleep(15)
                #rospy.loginfo("Done Loading")
                return
            elif (roller_command == cmd_unload):
                rospy.loginfo("AGC03 Roller is Unloading")
                self.agc03_roller_cmd_pub.publish(cmd_unload)
                rospy.sleep(1)
                # while not rospy.is_shutdown():
                #     rospy.loginfo("roller status : %s ", self.roller_status)
                #     if self.roller_status==self.status_complete_unload:
                #         rospy.loginfo("Done Un-Loading")
                #         r.sleep()
                #         return
                #     elif self.roller_status==self.status_unloading:
                #         rospy.loginfo("AGC03 Roller is Un-Loading")
                #         r.sleep()
                #rospy.sleep(15)
                #rospy.loginfo("Done Loading")
                return
            else:
                #rospy.loginfo("AGC03 Roller is Stoping")
                #agc03_roller_stop_pub.publish()
                #rospy.sleep(5)
                #while not self.roller_status == "agc03_roller_complete_load_out":
                #    rospy.sleep(1)
                #rospy.sleep(15)
                #rospy.loginfo("Done Unloading")
                return

            r.sleep()

    def status(self, msg):
        
        self.roller_status = msg.data
        global status_of_agc03_roller
        status_of_agc03_roller = self.roller_status
        
        # temp_data = msg.data
        # #rospy.loginfo("AGC03 Roller Status : %s", temp_data)
        # if temp_data == "agc03_roller_complete_load_in":
        #     self.roller_status = self.status_load_done
        # elif temp_data == "agc03_roller_complete_load_out":
        #     self.roller_status = self.status_unload_done
        # elif temp_data == "agc03_roller_idle":
        #     self.roller_status = self.status_idle
        # else:
        #     self.roller_status = self.status_unknow
        # #rospy.loginfo("AGC03 Status : %s", self.roller_status)


    def shutdown(self):
        rospy.loginfo("AGC03 Roller is stoped")
        rospy.sleep(1)
        agc03_buzzer_cmd_pub.publish("ch0")

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

        ###################### Subscribe from Roller10 ###################################################
        rospy.Subscriber('roller10_status', String, self.roller10_status)
        ###################### Publish to Roller13 #######################################################
        self.roller10_cmd_pub = rospy.Publisher('roller10_cmd', String, queue_size=1)
        ##################################################################################################

        ###################### Subscribe from Roller11 ###################################################
        rospy.Subscriber('roller11_status', String, self.roller11_status)
        ###################### Publish to Roller14 #######################################################
        self.roller11_cmd_pub = rospy.Publisher('roller11_cmd', String, queue_size=1)
        ##################################################################################################

        ###################### Subscribe from Roller12 ###################################################
        rospy.Subscriber('roller12_status', String, self.roller12_status)
        ###################### Publish to Roller15 #######################################################
        self.roller12_cmd_pub = rospy.Publisher('roller12_cmd', String, queue_size=1)
        ##################################################################################################

        # ###################### Subscribe from Roller13 ###################################################
        # rospy.Subscriber('roller13_status', String, self.roller13_status)
        # ###################### Publish to Roller13 #######################################################
        # self.roller13_cmd_pub = rospy.Publisher('roller13_cmd', String, queue_size=1)
        # ##################################################################################################

        # ###################### Subscribe from Roller14 ###################################################
        # rospy.Subscriber('roller14_status', String, self.roller14_status)
        # ###################### Publish to Roller14 #######################################################
        # self.roller14_cmd_pub = rospy.Publisher('roller14_cmd', String, queue_size=1)
        # ##################################################################################################

        # ###################### Subscribe from Roller15 ###################################################
        # rospy.Subscriber('roller15_status', String, self.roller15_status)
        # ###################### Publish to Roller15 #######################################################
        # self.roller15_cmd_pub = rospy.Publisher('roller15_cmd', String, queue_size=1)
        # ##################################################################################################

        # Begin the cmd_vel publishing loop
        while not rospy.is_shutdown():
      
            rospy.sleep(1)

            if ((roller_id == 10) and (roller_command == cmd_load)):
                self.roller10_cmd_pub.publish(cmd_load)
                rospy.sleep(1)
                return
            elif ((roller_id == 10) and (roller_command == cmd_unload)):
                self.roller10_cmd_pub.publish(cmd_unload)
                rospy.sleep(1)
                return
            elif ((roller_id == 11) and (roller_command == cmd_load)):
                self.roller11_cmd_pub.publish(cmd_load)
                rospy.sleep(1)
                return
            elif ((roller_id == 11) and (roller_command == cmd_unload)):
                self.roller11_cmd_pub.publish(cmd_unload)
                rospy.sleep(1)
                return
            elif ((roller_id == 12) and (roller_command == cmd_load)):
                self.roller12_cmd_pub.publish(cmd_load)
                rospy.sleep(1)
                return
            elif ((roller_id == 12) and (roller_command == cmd_unload)):
                self.roller12_cmd_pub.publish(cmd_unload)
                rospy.sleep(1)
                return
            # if ((roller_id == 13) and (roller_command == cmd_load)):
            #     self.roller13_cmd_pub.publish(cmd_load)
            #     rospy.sleep(1)
            #     return
            # elif ((roller_id == 13) and (roller_command == cmd_unload)):
            #     self.roller13_cmd_pub.publish(cmd_unload)
            #     rospy.sleep(1)
            #     return
            # elif ((roller_id == 14) and (roller_command == cmd_load)):
            #     self.roller14_cmd_pub.publish(cmd_load)
            #     rospy.sleep(1)
            #     return
            # elif ((roller_id == 14) and (roller_command == cmd_unload)):
            #     self.roller14_cmd_pub.publish(cmd_unload)
            #     rospy.sleep(1)
            #     return
            # elif ((roller_id == 15) and (roller_command == cmd_load)):
            #     self.roller15_cmd_pub.publish(cmd_load)
            #     rospy.sleep(1)
            #     return
            # elif ((roller_id == 15) and (roller_command == cmd_unload)):
            #     self.roller15_cmd_pub.publish(cmd_unload)
            #     rospy.sleep(1)
            #     return
            # else:
            #     return

            r.sleep()

    def roller10_status(self, msg):
        self.roller10_status = msg.data
        global status_of_roller10
        status_of_roller10 = self.roller10_status

    def roller11_status(self, msg):
        self.roller11_status = msg.data
        global status_of_roller11
        status_of_roller11 = self.roller11_status

    def roller12_status(self, msg):
        self.roller12_status = msg.data
        global status_of_roller12
        status_of_roller12 = self.roller12_status

    # def roller13_status(self, msg):
    #     self.roller13_status = msg.data
    #     global status_of_roller13
    #     status_of_roller13 = self.roller13_status

    # def roller14_status(self, msg):
    #     self.roller14_status = msg.data
    #     global status_of_roller14
    #     status_of_roller14 = self.roller14_status

    # def roller15_status(self, msg):
    #     self.roller15_status = msg.data
    #     global status_of_roller15
    #     status_of_roller15 = self.roller15_status        

    def shutdown(self):
        self.roller10_cmd_pub.publish(cmd_stop)
        self.roller11_cmd_pub.publish(cmd_stop)
        self.roller12_cmd_pub.publish(cmd_stop)
        rospy.loginfo("Rollers 10,11,12 are stoped")
        #self.roller13_cmd_pub.publish(cmd_stop)
        #self.roller14_cmd_pub.publish(cmd_stop)
        #self.roller15_cmd_pub.publish(cmd_stop)
        #rospy.loginfo("Rollers 13,14,15 are stoped")
        rospy.sleep(1)
        agc03_buzzer_cmd_pub.publish("ch0")
        agc03_buzzer_cmd_pub.publish("ch0")



    

class AGC03_cmd_status():   
    def __init__(self, AGC03_command, AGC03_status):

        # cmd_give_me_a_way = "give_me_a_way"
        # cmd_I_am_gone = "_I_am_gone"



        # Set the shutdown function (stop the roller)
        rospy.on_shutdown(self.shutdown)
        
        # How often should we update the rollers?
        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate)

        #received_agc03_cmd
        ###################### Subscribe from Roller15 ###################################################
        rospy.Subscriber('agc03_cmd', String, self.received_agc03_cmd)
        ###################### Publish to Roller15 #######################################################
        #self.agc03_status_pub = rospy.Publisher('agc03_status', String, queue_size=1)
        
        
        #agc03_status_pub.publish(AGC03_status_XX)
        ##################################################################################################

        # Begin the cmd_vel publishing loop
        while not rospy.is_shutdown():
            rospy.sleep(1)
            rospy.loginfo("status_of_received_agc03_cmd : %s " % status_of_received_agc03_cmd)
            if (AGC03_command == status_of_received_agc03_cmd):
                rospy.sleep(1)
                talker("wait_a_minute")
                talker("wait_a_minute")
                talker("wait_a_minute")
                return
            else:
                #rospy.loginfo(".")
                AGC03_status_XX = AGC03_status
                #self.agc03_status_pub.publish(AGC03_status_XX)
                talker(AGC03_status_XX)
                rospy.sleep(1)
                clear_costmaps(0)
                
            r.sleep()


    def received_agc03_cmd(self, msg):
        self.received_agc03_cmd = msg.data
        global status_of_received_agc03_cmd
        status_of_received_agc03_cmd = self.received_agc03_cmd  
        #rospy.loginfo("status_of_received_agc03_cmd : %s " % status_of_received_agc03_cmd) 

    def shutdown(self):
        #rospy.loginfo("Rollers 10,11,12 are stoped")
        #self.roller13_cmd_pub.publish(cmd_stop)
        rospy.loginfo("AGC03_cmd_status stop")
        rospy.sleep(1)



def agc03_load_in_10(roller_id):
    AGC03Roller("load")
    AGC03Roller("load")
    rospy.sleep(1)
    Roller(10, "unload")
    Roller(10, "unload")
    rospy.sleep(1)
    flag = False
    while not ( rospy.is_shutdown() or flag ):
        rospy.loginfo("AGC03 roller status : %s " % status_of_agc03_roller)
        rospy.loginfo("Roller10 status     : %s " % status_of_roller10)
        rospy.loginfo("--------------------------------------------------")
        #if ((status_of_roller10 == "roller10_complete_unload")or(status_of_roller10 == "roller10_complete_load")) and (status_of_agc03_roller == "complete_load"):
        if (status_of_agc03_roller == "complete_load"):
            rospy.loginfo("AGC03-Loading is Done.")
            rospy.sleep(1)
            flag = True
        else:
            rospy.loginfo("AGC03 are loading the object.")
            rospy.sleep(1)
            flag = False
    return
def agc03_load_out_10(roller_id):
    Roller(10, "load")
    Roller(10, "load")
    rospy.sleep(1)
    AGC03Roller("unload")
    AGC03Roller("unload")
    rospy.sleep(1)
    flag = False
    while not ( rospy.is_shutdown() or flag ):
        rospy.loginfo("AGC03 roller status : %s " % status_of_agc03_roller)
        rospy.loginfo("Roller10 status     : %s " % status_of_roller10)
        rospy.loginfo("----------------------------------------------------")
        #if (status_of_roller10 == "roller10_complete_load") and (status_of_agc03_roller == "complete_unload"):
        if (status_of_agc03_roller == "complete_unload"):
            rospy.loginfo("Done Un-Loading.")
            rospy.sleep(1)
            flag = True
        else:
            rospy.loginfo("AGC03 is Un-Loading the object.")
            rospy.sleep(1)
            flag = False
    return

def agc03_load_in_11(roller_id):
    AGC03Roller("load")
    AGC03Roller("load")
    rospy.sleep(1)
    Roller(11, "unload")
    Roller(11, "unload")
    rospy.sleep(1)
    flag = False
    while not ( rospy.is_shutdown() or flag ):
        rospy.loginfo("AGC03 roller status : %s " % status_of_agc03_roller)
        rospy.loginfo("Roller11 status     : %s " % status_of_roller11)
        rospy.loginfo("--------------------------------------------------")
        #if (status_of_roller11 == "roller11_complete_load") and (status_of_agc03_roller == "complete_load"):
        if (status_of_agc03_roller == "complete_load"):
            rospy.loginfo("AGC03-Loading is Done.")
            rospy.sleep(1)
            flag = True
        else:
            rospy.loginfo("AGC03 are loading the object.")
            rospy.sleep(1)
            flag = False
    return

def agc03_load_out_11(roller_id):
    Roller(11, "load")
    Roller(11, "load")
    rospy.sleep(1)
    AGC03Roller("unload")
    AGC03Roller("unload")
    rospy.sleep(1)
    flag = False
    while not ( rospy.is_shutdown() or flag ):
        rospy.loginfo("AGC03 roller status : %s " % status_of_agc03_roller)
        rospy.loginfo("Roller11 status     : %s " % status_of_roller11)
        rospy.loginfo("----------------------------------------------------")
        #if (status_of_roller11 == "roller11_complete_load") and (status_of_agc03_roller == "complete_unload"):
        if (status_of_agc03_roller == "complete_unload"):
            rospy.loginfo("Done Un-Loading.")
            rospy.sleep(1)
            flag = True
        else:
            rospy.loginfo("AGC03 is Un-Loading the object.")
            rospy.sleep(1)
            flag = False
    return

def agc03_load_in_12(roller_id):
    AGC03Roller("load")
    AGC03Roller("load")
    rospy.sleep(1)
    Roller(12, "unload")
    Roller(12, "unload")
    rospy.sleep(1)
    flag = False
    while not ( rospy.is_shutdown() or flag ):
        rospy.loginfo("AGC03 roller status : %s " % status_of_agc03_roller)
        rospy.loginfo("Roller12 status     : %s " % status_of_roller12)
        rospy.loginfo("--------------------------------------------------")
        #if (status_of_roller12 == "roller12_complete_load") and (status_of_agc03_roller == "complete_load"):
        if (status_of_agc03_roller == "complete_load"):
            rospy.loginfo("AGC03-Loading is Done.")
            rospy.sleep(1)
            flag = True
        else:
            rospy.loginfo("AGC03 are loading the object.")
            rospy.sleep(1)
            flag = False
    return

def agc03_load_out_12(roller_id):
    Roller(12, "load")
    Roller(12, "load")
    rospy.sleep(1)
    AGC03Roller("unload")
    AGC03Roller("unload")
    rospy.sleep(1)
    flag = False
    while not ( rospy.is_shutdown() or flag ):
        rospy.loginfo("AGC03 roller status : %s " % status_of_agc03_roller)
        rospy.loginfo("Roller12 status     : %s " % status_of_roller12)
        rospy.loginfo("----------------------------------------------------")
        #if (status_of_roller15 == "roller15_complete_load") and (status_of_agc03_roller == "complete_unload"):
        if (status_of_agc03_roller == "complete_unload"):
            rospy.loginfo("Done Un-Loading.")
            rospy.sleep(1)
            flag = True
        else:
            rospy.loginfo("AGC03 is Un-Loading the object.")
            rospy.sleep(1)
            flag = False
    return

def talker(message):
    '''test Publisher'''
    pub = rospy.Publisher('agc03_status', String, queue_size=10)
    #rospy.init_node('test', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = message
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        #rate.sleep()
        pub.publish(hello_str)
        #rate.sleep()
        pub.publish(hello_str)
        #rate.sleep()
        pub.publish(hello_str)
        #rate.sleep()
        return

def delay_loop(message):
    count = 0
    flag = False
    while not ( rospy.is_shutdown() or flag ):

        if (count< message):
            clear_costmaps(1)
            rospy.sleep(1)
            count = count+1
            agc03_buzzer_cmd_pub.publish("ch0")
            flag = False
        else:
            flag = True

    return








def clear_costmaps(howlong):
    rospy.wait_for_service('move_base/clear_costmaps')
    try:
        rospy.loginfo("Clearing costmaps")
        clear_costmaps_srv = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
        ccm = clear_costmaps_srv()
        #print "%s" %ccm
        rospy.sleep(howlong)
        rospy.loginfo("Clearing costmaps ... Done")
        return ccm
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



class agc03_AUTO_START():
    def __init__(self):

        # Set the shutdown function (stop the roller)
        rospy.on_shutdown(self.shutdown)
        
        # How often should we update the rollers?
        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate)

        ###################### Subscribe from agc03_AUTORUN_SW_status ####################################
        rospy.Subscriber('agc03_AUTORUN_SW_status', String, self.received_agc03_AUTORUN_SW_status)
        ##################################################################################################
        ###################### Subscribe from agc03_AUTORUN_SW_status ####################################
        rospy.Subscriber('agc03_E_STOP_status', String, self.received_agc03_E_STOP_status)
        ##################################################################################################
        ###################### Subscribe from agc03_AUTORUN_SW_status ####################################
        rospy.Subscriber('agc03_MASTERON_SW_status', String, self.received_agc03_MASTERON_SW_status)
        ##################################################################################################
        rospy.loginfo("  agc03_AUTO_START  ")

        rospy.sleep(1)
        # Begin the cmd_vel publishing loop
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            # rospy.loginfo("status_of_agc03_AUTORUN_SW  : %s " % status_of_agc03_AUTORUN_SW)
            # rospy.loginfo("status_of_agc03_E_STOP      : %s " % status_of_agc03_E_STOP)
            # rospy.loginfo("status_of_agc03_MASTERON_SW : %s " % status_of_agc03_MASTERON_SW)

            if ((status_of_agc03_E_STOP == "E_STOP_OFF") and (status_of_agc03_MASTERON_SW == "MASTERON_SW_ON")):
                rospy.sleep(0.1)
                rospy.loginfo("--------------------------------------------")
                return
            # else:
            #     rospy.sleep(0.1)
            #     rospy.loginfo("  agc03_AUTO_STAR  ")



    def received_agc03_AUTORUN_SW_status(self, msg):
        self.received_agc03_AUTORUN_SW = msg.data
        global status_of_agc03_AUTORUN_SW
        status_of_agc03_AUTORUN_SW = self.received_agc03_AUTORUN_SW
        # rospy.loginfo("status_of_agc03_AUTORUN_SW : %s " % status_of_agc03_AUTORUN_SW) 

    def received_agc03_E_STOP_status(self, msg):
        self.received_agc03_E_STOP = msg.data
        global status_of_agc03_E_STOP
        status_of_agc03_E_STOP = self.received_agc03_E_STOP
        # rospy.loginfo("status_of_agc03_E_STOP     : %s " % status_of_agc03_E_STOP) 

    def received_agc03_MASTERON_SW_status(self, msg):
        self.received_agc03_MASTERON_SW = msg.data
        global status_of_agc03_MASTERON_SW
        status_of_agc03_MASTERON_SW = self.received_agc03_MASTERON_SW
        # rospy.loginfo("status_of_agc03_MASTERON_SW : %s " % status_of_agc03_MASTERON_SW) 


    def shutdown(self):
        rospy.loginfo("AGC03_status stop")
        rospy.sleep(1)







if __name__ == '__main__':

    

    try:
        rospy.init_node('agc03_task_scheduler', anonymous=False)
        #AGC03_COMMAND("wait_a_minute")

        # received_agc03_cmd = temp_cmd.recieved_cmd()

        ###################### VOICES publisher  ##################################
        # voice1_on_pub = rospy.Publisher('VOICE1_ON', Empty, queue_size=1)
        # voice2_on_pub = rospy.Publisher('VOICE2_ON', Empty, queue_size=1)
        # voice3_on_pub = rospy.Publisher('VOICE3_ON', Empty, queue_size=1)
        # voice4_on_pub = rospy.Publisher('VOICE4_ON', Empty, queue_size=1)
        # voice_off_pub = rospy.Publisher('VOICE_OFF', Empty, queue_size=1)
        agc03_buzzer_cmd_pub = rospy.Publisher("agc03_buzzer_cmd", String, queue_size=1)
        ###########################################################################
        agc03_LED_cmd_pub = rospy.Publisher("agc03_LED_cmd", String, queue_size=1)
        ###########################################################################

        rospy.sleep(1)
        agc03_buzzer_cmd_pub.publish("ch4")
        rospy.sleep(1)
        agc03_buzzer_cmd_pub.publish("ch4")
        rospy.sleep(1)
        agc03_buzzer_cmd_pub.publish("ch4")
        rospy.sleep(2)
        agc03_buzzer_cmd_pub.publish("ch0")
        rospy.sleep(1)
        agc03_buzzer_cmd_pub.publish("ch0")

        #agc03_AUTO_START()
        rospy.sleep(1)
        agc03_buzzer_cmd_pub.publish("ch0")
        rospy.sleep(1)
        agc03_buzzer_cmd_pub.publish("ch0")


        while not rospy.is_shutdown():

            rospy.sleep(1)

            #talker("clear_to_go")
            #talker("clear_to_go")
            #agc03_load_in_10(10)
            #--------------------------------------------------------------------#
            #--------------------------------------------------------------------#
            #--------------------------------------------------------------------#
            clear_costmaps(1)
            rospy.sleep(1)
            delay_loop(1)
            #AGC03_cmd_status("I_am_gone", "clear_to_go")
            #talker("wait_a_minute")
            #talker("wait_a_minute")
            rospy.loginfo("status :-------AR 10---goto Roller AR 11----------- ")
            agc03_buzzer_cmd_pub.publish("ch1")
            agc03_buzzer_cmd_pub.publish("ch1")
            agc03_LED_cmd_pub.publish("RUN1")
            agc03_LED_cmd_pub.publish("RUN1")
            rospy.sleep(1)
            ARFollower(10, 1.10, 0.1, 10)

            GoToPose(-1.80,1.11,0.0)    #GoToPose Roller AR 11
            agc03_LED_cmd_pub.publish("RUN2")
            agc03_LED_cmd_pub.publish("RUN2")
            ARFollower(11, 0.51, 0.006, 16)
            rospy.loginfo("status :-------------END Roller 11------------------ ")
            agc03_buzzer_cmd_pub.publish("ch0")
            agc03_buzzer_cmd_pub.publish("ch0")
            agc03_LED_cmd_pub.publish("OFF_")
            agc03_LED_cmd_pub.publish("OFF_")
            agc03_LED_cmd_pub.publish("OFF_")
            #talker("clear_to_go")
            #talker("clear_to_go")
            #agc03_load_out_11(11)
            #delay_loop(5)
            #agc03_load_in_11(11)
            delay_loop(5)

            #--------------------------------------------------------------------#
            #--------------------------------------------------------------------#
            #--------------------------------------------------------------------#
            #AGC03_cmd_status("I_am_gone", "clear_to_go")
            #talker("wait_a_minute")
            #talker("wait_a_minute")
            rospy.loginfo("status :------AR 11----goto Roller AR 10----------- ")
            rospy.sleep(0.5)
            agc03_LED_cmd_pub.publish("RUN1")
            agc03_LED_cmd_pub.publish("RUN1")
            agc03_buzzer_cmd_pub.publish("ch1")
            agc03_buzzer_cmd_pub.publish("ch1")
            rospy.sleep(1)
            ARFollower(11, 1.0, 0.1, 1)

            GoToPose(-1.0,1.64,-180)   #GoToPose Roller AR 10
            agc03_LED_cmd_pub.publish("RUN2")
            agc03_LED_cmd_pub.publish("RUN2")
            ARFollower(10, 0.50, 0.006, 16)
            rospy.loginfo("status :-------------END Roller 10------------------")
            agc03_buzzer_cmd_pub.publish("ch0")
            agc03_buzzer_cmd_pub.publish("ch0")
            agc03_LED_cmd_pub.publish("OFF_")
            agc03_LED_cmd_pub.publish("OFF_")
            agc03_LED_cmd_pub.publish("OFF_")
            #talker("clear_to_go")
            #talker("clear_to_go")
            #agc03_load_out_10(10)
            #delay_loop(5)
            #agc03_load_in_10(10)
            delay_loop(5)

            #--------------------------------------------------------------------#
            #--------------------------------------------------------------------#
            #--------------------------------------------------------------------#
            #AGC03_cmd_status("I_am_gone", "clear_to_go")
            #talker("wait_a_minute")
            #talker("wait_a_minute")
            rospy.loginfo("status :----AR 10------goto Roller AR 12----------- ")
            agc03_buzzer_cmd_pub.publish("ch1")
            agc03_buzzer_cmd_pub.publish("ch1")
            agc03_LED_cmd_pub.publish("RUN1")
            agc03_LED_cmd_pub.publish("RUN1")
            rospy.sleep(1)
            ARFollower(10, 0.85, 0.1, 1)

            GoToPose(-1.80,1.60,0.0)    #GoToPose Roller AR 12
            agc03_LED_cmd_pub.publish("RUN2")
            agc03_LED_cmd_pub.publish("RUN2")
            ARFollower(12, 0.5, 0.006, 16)
            rospy.loginfo("status :-------------END Roller 12------------------")
            agc03_buzzer_cmd_pub.publish("ch0")
            agc03_buzzer_cmd_pub.publish("ch0")
            agc03_LED_cmd_pub.publish("OFF_")
            agc03_LED_cmd_pub.publish("OFF_")
            agc03_LED_cmd_pub.publish("OFF_")
            #talker("clear_to_go")
            #talker("clear_to_go")
            #agc03_load_out_12(12)
            #delay_loop(5)
            #agc03_load_in_12(12)
            delay_loop(5)


            #--------------------------------------------------------------------#
            #--------------------------------------------------------------------#
            #--------------------------------------------------------------------#
            #AGC03_cmd_status("I_am_gone", "clear_to_go")
            #talker("wait_a_minute")
            #talker("wait_a_minute")
            rospy.loginfo("status :-----AR 12-----goto Roller AR 10----------- ")
            agc03_LED_cmd_pub.publish("RUN1")
            agc03_LED_cmd_pub.publish("RUN1")
            agc03_buzzer_cmd_pub.publish("ch1")
            agc03_buzzer_cmd_pub.publish("ch1")
            rospy.sleep(1)
            ARFollower(12, 1.0, 0.1, 1)

            GoToPose(-1.0,1.64,-180)   #GoToPose Roller AR 10
            agc03_LED_cmd_pub.publish("RUN2")
            agc03_LED_cmd_pub.publish("RUN2")
            ARFollower(10, 0.50, 0.006, 16)
            rospy.loginfo("status :-------------END Roller 10------------------")
            agc03_buzzer_cmd_pub.publish("ch0")
            agc03_buzzer_cmd_pub.publish("ch0")
            agc03_LED_cmd_pub.publish("OFF_")
            agc03_LED_cmd_pub.publish("OFF_")
            agc03_LED_cmd_pub.publish("OFF_")
            #talker("clear_to_go")
            #talker("clear_to_go")
            #agc03_load_out_10(10)
            delay_loop(5)
            

#############################################################################################

        #rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AGC03 task scheduler node terminated.")

