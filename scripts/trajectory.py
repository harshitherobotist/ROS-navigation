#!/usr/bin/env python  
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import roslib
import tf
from tf import TransformListener
from control_msgs.msg import *
from math import pi
import numpy as np
import time

def move_to_goal(xGoal,yGoal):

  
   ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

   while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
           rospy.loginfo("Waiting...")

   goal = MoveBaseGoal()
   goal.target_pose.header.frame_id = "map"
   goal.target_pose.header.stamp = rospy.Time.now()

   # moving to the goal

   goal.target_pose.pose.position =  Point(xGoal,yGoal,0)

   goal.target_pose.pose.orientation.w = 1.0
   rospy.loginfo("Sending goal location ...")
   ac.send_goal(goal)
   ac.wait_for_result(rospy.Duration(11))
   

   if(ac.get_state() ==  GoalStatus.SUCCEEDED):
           rospy.loginfo("destination reached")
           return True
   else:
           rospy.loginfo("The robot failed to reach the destination")
           return False

if __name__ == '__main__':
   rospy.init_node('map_navigation', anonymous=False)
   moveit_commander.roscpp_initialize(sys.argv)
   robot = moveit_commander.RobotCommander()
   scene = moveit_commander.PlanningSceneInterface()
   scene._pub_co = rospy.Publisher('/collision_object', CollisionObject, queue_size=100)
   arm_group = moveit_commander.MoveGroupCommander("arm")
   # gripper_group = moveit_commander.MoveGroupCommander("gripper")
   # moveit_commander.move_group.MoveGroupCommander.set_planner_id(arm_group,"SPARS")
   #  rate = rospy.Rate(10.0)
   
   rospy.loginfo("Started Run!")

   x_goal = 9.8
   y_goal = 0.95
   move_to_goal(x_goal,y_goal)
   rospy.loginfo("A")

   arm_group.set_named_target("down")
   arm_group.set_planning_time(5)
   plan1 = arm_group.go()
   
   x_goal = 12.8
   y_goal = -0.200
   move_to_goal(x_goal,y_goal)

  

   x_goal = 14.25 
   y_goal = -0.76
   move_to_goal(x_goal,y_goal)

   x_goal = 11.0
   y_goal = -1.55
   move_to_goal(x_goal,y_goal)
   
   x_goal = 11.5
   y_goal = 1.2
   move_to_goal(x_goal,y_goal)

   x_goal = 14.0
   y_goal = 2.5
   move_to_goal(x_goal,y_goal)

   x_goal = 10.0
   y_goal = 0.93
   move_to_goal(x_goal,y_goal)
   rospy.loginfo("B")

   x_goal = 4.0
   y_goal = 0.87
   move_to_goal(x_goal,y_goal)

   rospy.sleep(3)
   x_goal = 6.45
   y_goal = -0.122
   move_to_goal(x_goal,y_goal)
   print("goal 5 reached")
   

   x_goal = 7.50
   y_goal = -0.62
   move_to_goal(x_goal,y_goal)

   x_goal = 6.21
   y_goal = 0.83
   move_to_goal(x_goal,y_goal)

   x_goal = 11.8
   y_goal = 0.95
   move_to_goal(x_goal,y_goal)
   rospy.loginfo("B")

   x_goal = 20.323 
   y_goal = -1.264
   move_to_goal(x_goal,y_goal)
   print("goal mid reached")

   x_goal = 25.51 
   y_goal = -2.98
   move_to_goal(x_goal,y_goal)

   x_goal = 25.71 
   y_goal = -2.98
   move_to_goal(x_goal,y_goal)
  
   x_goal = 20.323 
   y_goal = -1.264
   move_to_goal(x_goal,y_goal)
   print("goal mid reached")

   x_goal = 10.50
   y_goal = 9.38
   move_to_goal(x_goal,y_goal)

   rospy.sleep(6)
   x_goal = 10.51
   y_goal = 9.39
   move_to_goal(x_goal,y_goal)
   print("goal 3 reached")
 
   x_goal = 11.8
   y_goal = 1.20
   move_to_goal(x_goal,y_goal)

   
   x_goal = 0.0
   y_goal = 0.0
   move_to_goal(x_goal,y_goal)
   rospy.sleep(2)

   x_goal = 0.0
   y_goal = 0.0
   move_to_goal(x_goal,y_goal)
  
   rospy.spin()
           

