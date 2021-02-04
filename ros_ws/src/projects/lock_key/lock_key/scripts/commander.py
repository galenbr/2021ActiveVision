#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Class for working with MoveIt Commander.
"""
import sys
import moveit_commander
import rospy
#Messages
import moveit_msgs.msg

class Commander:
	'''Commander interface for Panda.'''
	def __init__(self,vel_scale=0.1):
		#Initialize commander
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group_name = "panda_arm"
		self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
															moveit_msgs.msg.DisplayTrajectory,
															queue_size=20)
		#Set velocity scale
		self.set_vel_scale(vel_scale)
		rospy.loginfo('Initialized Commander Interface')

	def __repr__(self):
		'''Defines string representation of Commander object.'''
		return self.group_name + " Commander"

	def move_relative(self,pose_change_goal,goal_time=0.5):
		'''Moves end-effector by a relative displacement in panda_link8 frame.'''
		#TODO: Finish Implementation
		# Retrieve current pose in map frame using getTFClient
		current_pose=
		# Convert to EE frame, then update, then back to map??
		# Update desired pose with relative change
		pose_goal=
		# Move to desired pose
		self.move_to_pose(pose_goal,goal_time)

	def move_to_plane(self,step,axis,max_disp,max_force,max_iter=500,
					  orientation=None,goal_time=0.5):
		'''Incrementally move end-effector until it hits a plane.'''
		#TODO: Finish Implementation
		ii=1
		disp=0
		if orientation is not None:
			#Then use the given orientation
		else:
			#Use the current orientation
		#Build pose_change_step using step (EE frame) and orientation
		pose_change_step=
		#Move slightly until force or max. iterations are exceeded.
		while (force<=max_force) and (disp<=max_disp) and (ii<=max_iter):
			self.move_relative(pose_change_step,goal_time)
			disp+=
			ii+=1

		if ii<=max_iter:
			rospy.loginfo('Arrived at plane.')

	def move_to_pose(self,pose_goal,goal_time=0.5):
		'''Move to pose in map frame.'''
		#Set goal time for trajectory
		rospy.set_param('/position_joint_trajectory_controller/constraints/goal_time',goal_time)
		#Plan to Pose goal
		self.move_group.set_pose_target(pose_goal)
		#Execute
		plan = self.move_group.go(wait=True)
		# Ensure that there is no residual movement
		self.move_group.stop()
		# Clear targets
		self.move_group.clear_pose_targets()

	def rotate(self,angle,axis,max_angle,max_torque,max_iter=500):
		'''Incrementally rotate end-effector.'''
		#TODO: Finish Implementation
		if orientation is not None:
			#Then use the given orientation
		else:
			#Use the current orientation

	def set_vel_scale(self,vel_scale):
		'''Sets velocity scaling (0,1] for motion.'''
		self.move_group.set_max_velocity_scaling_factor(vel_scale)