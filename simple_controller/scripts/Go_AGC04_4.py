#!/usr/bin/env python


import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from math import copysign

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
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


class GoToPose():
    def __init__(self):

        #rospy.init_node('nav_test', anonymous=False)

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60)) 

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

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)


class AGC04Roller ():
    def __init__(self, roller_command):

        self.load_done = 1
        self.unload_done = 2
        self.idle = 3
        self.unknow = 4

        self.roller_status = self.unknow

        # Set the shutdown function (stop the roller)
        rospy.on_shutdown(self.shutdown)
        
        # How often should we update the agc04 roller?
        self.rate = rospy.get_param("~rate", 20)
        r = rospy.Rate(self.rate)

        rospy.Subscriber('agc04_conveyor_status', String, self.status)
        ###################### Roller publisher ##############################################################
        self.agc04_roller_load_in_pub = rospy.Publisher('agc04_roller_load_in', Empty, queue_size=1)
        self.agc04_roller_load_out_pub = rospy.Publisher('agc04_roller_load_out', Empty, queue_size=1)
        self.agc04_roller_load_manual_in_pub = rospy.Publisher('agc04_roller_manual_load_in', Empty, queue_size=1)
        self.agc04_roller_load_manual_out_pub = rospy.Publisher('agc04_roller_manual_load_in', Empty, queue_size=1)
        self.agc04_roller_stop_pub = rospy.Publisher('agc04_roller_stop', Empty, queue_size=1)
        ######################################################################################################

        # Begin the cmd_vel publishing loop
        while not rospy.is_shutdown():
      
            # Sleep for 1/self.rate seconds

            #rospy.loginfo(" data_target_offset_x = %s  ,  %s", self.data_target_offset_x,self.data_target_offset_y)
            # if (self.data_target_offset_x == 0) and (self.data_target_offset_y == 0): 
            #     if count_timeout > 50:    # 1/self.rate x timeout = T
            #         #rospy.loginfo(" return -----------------------")
            #         return
            #     else:
            #         count_timeout = count_timeout + 1
            #         #rospy.loginfo(" count_timeout %s  = ", count_timeout)
            # else:
            #     count_timeout = 0
            #     #rospy.loginfo(" count_timeout %s  = ", count_timeout)

            rospy.loginfo("roller status : %s ", self.roller_status)
            rospy.loginfo("roller command : %s ", roller_command)
            rospy.sleep(5)

            if (roller_command == "load"): # and ((self.roller_status == self.unload_done) or (self.roller_status == self.idle)) and (self.roller_status == self.unknow)):
                rospy.loginfo("AGC04 Roller is Loading")
                self.agc04_roller_load_in_pub.publish()
                rospy.sleep(1)
                while not rospy.is_shutdown():
                    rospy.loginfo("roller status : %s ", self.roller_status)
                    #if (self.roller_status==self.load_done):
                    #    return
                    #else:
                    #    r.sleep()
                #rospy.sleep(15)
                #rospy.loginfo("Done Loading")
                return
            elif (roller_command == "unload"): # and ((self.roller_status == self.load_done) or (self.roller_status == self.idle) or (self.roller_status == self.unknow))):
                rospy.loginfo("AGC04 Roller is Unloading")
                #self.agc04_roller_load_in_pub.publish()
                rospy.sleep(5)
                #while not self.roller_status == "agc04_roller_complete_load_out":
                #    rospy.sleep(1)
                #rospy.sleep(15)
                #rospy.loginfo("Done Unloading")
                return
            elif roller_command == "stop":
                rospy.loginfo("AGC04 Roller is Stoping")
                #self.agc04_roller_load_in_pub.publish()
                rospy.sleep(5)
                #while not self.roller_status == "agc04_roller_complete_load_out":
                #    rospy.sleep(1)
                #rospy.sleep(15)
                #rospy.loginfo("Done Unloading")
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
        temp_data = msg.data
        self.roller_status = 0
        #rospy.loginfo("AGC04 Data : %s", temp_data)
        if temp_data == "agc04_roller_complete_load_in":
            self.roller_status = self.load_done
        elif temp_data == "agc04_roller_complete_load_out":
            self.roller_status = self.unload_done
        elif temp_data == "agc04_roller_idle":
            self.roller_status = self.idle
        else:
            self.roller_status = self.unknow
        #rospy.loginfo("AGC04 Status : %s", self.roller_status)

    def shutdown(self):
        rospy.loginfo("Roller Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('agc04_go', anonymous=False)

        ###################### VOICES publisher  ###################################
        voice1_on_pub = rospy.Publisher('VOICE1_ON', Empty, queue_size=1)
        voice2_on_pub = rospy.Publisher('VOICE2_ON', Empty, queue_size=1)
        voice3_on_pub = rospy.Publisher('VOICE3_ON', Empty, queue_size=1)
        voice4_on_pub = rospy.Publisher('VOICE4_ON', Empty, queue_size=1)
        voice_off_pub = rospy.Publisher('VOICE_OFF', Empty, queue_size=1)
        ###########################################################################

        # ###################### Roller publisher ##############################################################
        # agc04_roller_load_in_pub = rospy.Publisher('agc04_roller_load_in', Empty, queue_size=1)
        # agc04_roller_load_out_pub = rospy.Publisher('agc04_roller_load_out', Empty, queue_size=1)
        # agc04_roller_load_manual_in_pub = rospy.Publisher('agc04_roller_manual_load_in', Empty, queue_size=1)
        # agc04_roller_load_manual_out_pub = rospy.Publisher('agc04_roller_manual_load_in', Empty, queue_size=1)
        # agc04_roller_stop_pub = rospy.Publisher('agc04_roller_stop', Empty, queue_size=1)
        # ######################################################################################################

        navigator = GoToPose()
        #roller_control = AGC04Roller()


        while not rospy.is_shutdown():

            #ARFollower(13, 0.5, 0.05)
            #rospy.sleep(3)

            #agc04_roller_load_in_pub.publish()
            #roller_control.command("load")
            #roller_control.command("unload")
            #roller_control.command("stop")
            AGC04Roller("load")
            rospy.loginfo("Done Loading")
            rospy.sleep(1)
            rospy.spin()

            AGC04Roller("unload")
            rospy.loginfo("Done Unloading")
            rospy.sleep(1)

            AGC04Roller("stop")
            rospy.loginfo("Done Stoping")
            rospy.sleep(1)            

            # voice1_on_pub.publish()
            # ARFollower(13, 1.0, 0.05)
            # rospy.sleep(3)
            # voice_off_pub.publish()
            # rospy.sleep(1)


            # voice2_on_pub.publish()
            # # Customize the following values so they are appropriate for your location
            # position = {'x': -0.10, 'y' : 2.85}
            # quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : -0.707, 'r4' : 0.707}
            # rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y']) 
            # success = navigator.goto(position, quaternion)
            # rospy.sleep(3)
            # voice_off_pub.publish()
            # rospy.sleep(1)

            # voice1_on_pub.publish()
            # ARFollower(14, 0.49, 0.005)
            # rospy.sleep(3)
            # voice_off_pub.publish()
            # rospy.sleep(1)

            # agc04_roller_load_out_pub.publish()
            # rospy.sleep(15)

            # voice1_on_pub.publish()
            # ARFollower(14, 1.0, 0.05)
            # rospy.sleep(3)
            # voice_off_pub.publish()
            # rospy.sleep(1)

            # voice3_on_pub.publish()
            # # Customize the following values so they are appropriate for your location
            # position = {'x': 0.85, 'y' : 2.85} # position = {'x': 0.831907475474, 'y' : 2.56}
            # quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.707, 'r4' : 0.707}
            # rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y']) 
            # success = navigator.goto(position, quaternion)
            # rospy.sleep(3)

            # voice_off_pub.publish()
            # rospy.sleep(1)

            # voice1_on_pub.publish()
            # ARFollower(13, 0.50, 0.005)
            # rospy.sleep(3)
            # voice_off_pub.publish()
            # rospy.sleep(1)

        #rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AR follower node terminated.")

