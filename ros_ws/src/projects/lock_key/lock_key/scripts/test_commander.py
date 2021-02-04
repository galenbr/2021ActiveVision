#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Testing python commands with commander interface

"""
from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf_conversions

class Commander:
	def __init__(self):
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group_name = "panda_arm"
		self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
															moveit_msgs.msg.DisplayTrajectory,
															queue_size=20)
		rospy.loginfo('Initialized Commander Interface')

	def move_to_pose(self,pose_goal):
		'''Move to pose in map frame.'''
		#Plan to Pose goal
		self.move_group.set_pose_target(pose_goal)
		#Execute
		plan = self.move_group.go(wait=True)
		# Ensure that there is no residual movement
		self.move_group.stop()
		# Clear targets
		self.move_group.clear_pose_targets()

if __name__=='__main__':
	panda=Commander()

	#Key
	pose_goal = geometry_msgs.msg.Pose()
	quat=tf_conversions.transformations.quaternion_from_euler(-1.5708,0.7854,0.0)
	pose_goal.orientation.x = quat[0]
	pose_goal.orientation.y = quat[1]
	pose_goal.orientation.z = quat[2]
	pose_goal.orientation.w = quat[3]
	pose_goal.position.x = 0.4
	pose_goal.position.y = -0.2
	pose_goal.position.z = 0.59

	panda.move_to_pose(pose_goal)

	#Lock
	quat=tf_conversions.transformations.quaternion_from_euler(3.1416,0.0,-0.7854)
	pose_goal.orientation.x = quat[0]
	pose_goal.orientation.y = quat[1]
	pose_goal.orientation.z = quat[2]
	pose_goal.orientation.w = quat[3]
	pose_goal.position.x = 0.3
	pose_goal.position.y = 0.0
	pose_goal.position.z = 0.59
	panda.move_to_pose(pose_goal)