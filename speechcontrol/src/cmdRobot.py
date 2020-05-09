#! /usr/bin/env python
__author__ = 'Chenghao Wang, Kai Zheng'
import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from speechcontrol.msg import array_msg

class Moving():
    def __init__(self):
        self.goal_sent = False 

        rospy.on_shutdown(self.shutdown)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, location):

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = location

	# Start moving
        self.move_base.send_goal(goal)

	#Since the map is not big, set the duration time of the turtlebot as 300s
        success = self.move_base.wait_for_result(rospy.Duration(300)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

def callback(array_msg): 
    locations = dict()
    locations['kitchen'] = Pose(Point(6.08, -1.28, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))  
    locations['room1'] = Pose(Point(--6.24, -0.36, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))  
    locations['room2'] = Pose(Point(-5.62, 3.11, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))  
    locations['room3'] = Pose(Point(-2.72, 4.09, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))  
    locations['room4'] = Pose(Point(1.42, 4.49, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))  
    locations['room5'] = Pose(Point(5.02, 4.18, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))  


    rospy.loginfo(rospy.get_caller_id() + " I will %s", array_msg.action)
    rospy.loginfo(rospy.get_caller_id() + " And I will bring the %s", array_msg.target)
    if array_msg.targetroom != '':
        rospy.loginfo(rospy.get_caller_id() + " I will go to %s", array_msg.targetroom)
    #print "OK"
    try:
        navigator = Moving()
        position = locations['kitchen']
        rospy.loginfo("Go to the kitchen.")

        success = navigator.goto(position)

        if success:
            rospy.loginfo("Get the %s", array_msg.target)
            if array_msg.action == 'retrieve':
                navigator = Moving()
                position = locations['room3']
                rospy.loginfo("Retrieve to the start room")

                backway = navigator.goto(position)
                if backway:
                    rospy.loginfo("Here is your %s", array_msg.target)
            elif array_msg.action == 'goto':
                navigator = Moving()
                position = locations['room4']
                rospy.loginfo("Go to %s", array_msg.targetroom)

                backway = navigator.goto(position)
                if backway:
                    rospy.loginfo("Here is your %s", array_msg.target)
        else:
            rospy.loginfo("The base failed to reach the desired pose")
	    # Sleep to send the log
	    rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

rospy.init_node('subscriber')
sub = rospy.Subscriber('/speech',array_msg,callback)
rospy.spin()

