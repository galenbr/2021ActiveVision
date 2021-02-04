#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Class for working with MoveIt Commander.
"""
import sys
import actionlib
import moveit_commander
import rospy
import tf_conversions
#Messages
import franka_gripper.msg
import geometry_msgs.msg

class Epsilon:
	'''Define Epsilon object for gripper function.'''
	def __init__(self, inner, outer):
		self.inner=inner
		self.outer=outer

class Commander:
	'''Commander interface for Panda.'''
	def __init__(self,group_name="panda_arm",vel_scale=0.1):
		#Initialize commander
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group_name = group_name
		self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
		#Set velocity scale
		self.set_vel_scale(vel_scale)
		#Define joint home positions
		self.joint_home=[0.0,-0.785,0.0,-2.356, 0.0, 1.57, 0.784]
		#Wait for action server
		self.gripper_client = actionlib.SimpleActionClient('franka_gripper/grasp', 
											  	      	   franka_gripper.msg.GraspAction)
		rospy.loginfo('Waiting for action server')
		self.gripper_client.wait_for_server()
		rospy.loginfo('Initialized Commander Interface and Action Clients')

	def __repr__(self):
		'''Defines string representation of Commander object.'''
		return self.group_name + " Commander:\n" + str(self.robot.get_current_state())

	# def move_relative(self,pose_change_goal,goal_time=0.5):
	# 	'''Moves end-effector by a relative displacement in panda_link8 frame.'''
	# 	#TODO: Finish Implementation
	# 	# Retrieve current pose in map frame using getTFClient
	# 	current_pose=
	# 	# Convert to EE frame, then update, then back to map??
	# 	# Update desired pose with relative change
	# 	pose_goal=
	# 	# Move to desired pose
	# 	self.move_to_pose(pose_goal,goal_time)

	def move_gripper(self, width, epsilon_inner=0.05, epsilon_outer=0.05, 
					 speed=0.1, force=40):
		'''Moves gripper to the specified width.'''
		epsilon=Epsilon(epsilon_inner,epsilon_outer)
		goal=franka_gripper.msg.GraspGoal(width, epsilon, speed, force)
		rospy.loginfo('Sending gripper goal')
		self.gripper_client.send_goal(goal)
		rospy.loginfo('Waiting for gripper result')
		self.gripper_client.wait_for_result()

	# def move_to_plane(self,step,axis,max_disp,max_force,max_iter=500,
	# 				  orientation=None,goal_time=0.5):
	# 	'''Incrementally move end-effector until it hits a plane.'''
	# 	#TODO: Finish Implementation
	# 	ii=1
	# 	disp=0
	# 	if orientation is not None:
	# 		#Then use the given orientation
	# 	else:
	# 		#Use the current orientation
	# 	#Build pose_change_step using step (EE frame) and orientation
	# 	pose_change_step=
	# 	#Move slightly until force or max. iterations are exceeded.
	# 	while (force<=max_force) and (disp<=max_disp) and (ii<=max_iter):
	# 		self.move_relative(pose_change_step,goal_time)
	# 		disp+=
	# 		ii+=1

	# 	if ii<=max_iter:
	# 		rospy.loginfo('Arrived at plane.')

	def move_to_joint_pos(self,joint_goal,goal_time=0.5):
		'''Move to joint specificed joint positions joint_goal[0:6] in rads.'''
		#Set goal time for trajectory
		rospy.set_param('/position_joint_trajectory_controller/constraints/goal_time',goal_time)
		# Execute
		plan=self.move_group.go(joint_goal, wait=True)
		# Stop movemoent, clear targets
		self.stop_and_clear()

	def move_to_pose(self,pose_goal,goal_time=0.5):
		'''Move to pose in map frame.'''
		#Set goal time for trajectory
		rospy.set_param('/position_joint_trajectory_controller/constraints/goal_time',goal_time)
		#Plan to Pose goal
		self.move_group.set_pose_target(pose_goal)
		#Execute
		plan = self.move_group.go(wait=True)
		# Stop movemoent, clear targets
		self.stop_and_clear()

	# def rotate(self,angle,axis,max_angle,max_torque,max_iter=500):
		'''Incrementally rotate end-effector.'''
		#TODO: Finish Implementation
		# if orientation is not None:
		# 	#Then use the given orientation
		# else:
		# 	#Use the current orientation

	def set_vel_scale(self,vel_scale):
		'''Sets velocity scaling (0,1] for motion.'''
		self.move_group.set_max_velocity_scaling_factor(vel_scale)

	def stop_and_clear(self):
		'''Stops residual motions. Clears movement targets.'''
		self.move_group.stop()
		self.move_group.clear_pose_targets()

if __name__ == '__main__':
	#Instantiate commander
	panda=Commander(group_name="arm") #Gazebo uses "arm". Physical uses "panda_arm"
	#Test __repr__ method
	print panda

	#Test move_to_pose method
	pose_goal = geometry_msgs.msg.Pose()
	goal_time = 0.5
	quat=tf_conversions.transformations.quaternion_from_euler(-1.5708, #roll
															  0.7854,  #pitch
															  0.0)     #yaw
	pose_goal.orientation.x = quat[0]
	pose_goal.orientation.y = quat[1]
	pose_goal.orientation.z = quat[2]
	pose_goal.orientation.w = quat[3]
	pose_goal.position.x = 0.4
	pose_goal.position.y = 0.2
	pose_goal.position.z = 0.3
	panda.move_to_pose(pose_goal,goal_time)

	#Test move_to_joint_pos method
	panda.move_to_joint_pos(panda.joint_home,goal_time=0.8)

	#Test move_gripper method
	panda.move_gripper(0.02)