#!/usr/bin/env python
import roslib
roslib.load_manifest('turtlebot_actions')

import rospy

import os
import sys
import time
import math
from turtlebot_actions.msg import *
from actionlib_msgs.msg import *


import actionlib

'''
  Very simple move action test - commands the robot to turn 45 degrees and travel 0.5 metres forward.
'''

def main():
  rospy.init_node("test_move_action_client")

  # Construct action ac
  rospy.loginfo("Starting action client...")
  action_client = actionlib.SimpleActionClient('turtlebot_move', TurtlebotMoveAction)
  action_client.wait_for_server()
  rospy.loginfo("Action client connected to action server.")

  # Call the action
  rospy.loginfo("Calling the action server...")
  action_goal = TurtlebotMoveGoal()
  action_goal.turn_distance = -math.pi/4.0
  action_goal.forward_distance = 0.5 # metres

  if action_client.send_goal_and_wait(action_goal, rospy.Duration(50.0), rospy.Duration(50.0)) == GoalStatus.SUCCEEDED:
    rospy.loginfo('Call to action server succeeded')
  else:
    rospy.logerr('Call to action server failed')


if __name__ == "__main__":
  main()
