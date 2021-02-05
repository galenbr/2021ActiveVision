#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Class for working with MoveIt Commander.
"""
import sys
import actionlib
import moveit_commander
import rospy
import tf
import tf_conversions
#Messages
from lock_key.srv import getAveWrench, getWrench
from moveit_planner.srv import GetTF, GetTFRequest, GetPose
from std_msgs.msg import Header
import franka_gripper.msg
import geometry_msgs.msg

class Commander:
	'''Commander interface for Panda.'''
	def __init__(self,group_name="panda_arm",vel_scale=0.1,gazebo=False):
		#Initialize commander
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.gazebo=gazebo
		#Select control group
		if self.gazebo:
			self.group_name="arm"
		else:
			self.group_name = group_name
		self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
		#Set velocity scale
		self.set_vel_scale(vel_scale)
		#Define joint home positions
		self.joint_home=[0.0,-0.785,0.0,-2.356, 0.0, 1.57, 0.784]
		#Wait for action server
		self.gripper_client = actionlib.SimpleActionClient('franka_gripper/grasp', 
											  	      	   franka_gripper.msg.GraspAction)
		rospy.loginfo('Waiting for actions, services, and TF listener')
		if self.gazebo:
			rospy.wait_for_service('get_pose')
			self.get_pose_client = rospy.ServiceProxy('get_pose', GetPose)
		else:
			self.gripper_client.wait_for_server()
			rospy.wait_for_service('get_transform')
			self.get_tf_client = rospy.ServiceProxy('get_transform', GetTF)
		rospy.wait_for_service('getWrench')
		rospy.wait_for_service('getAveWrench')
		self.get_wrench_client = rospy.ServiceProxy('getWrench', getWrench)
		self.get_ave_wrench_client = rospy.ServiceProxy('getAveWrench', getAveWrench)
		self.transformer=tf.TransformListener()
		rospy.loginfo('Initialized Commander Interface and Action Clients')

	def __repr__(self):
		'''Defines string representation of Commander object.'''
		return self.group_name + " Commander:\n" + str(self.robot.get_current_state())

	def get_ft(self,remove_bias=True):
		'''Reads force/torque value and optionally removes biases.'''
		if remove_bias:
			return self.get_wrench_client()-self.ft_bias
		else:
			return self.get_wrench_client()

	def get_ft_bias(self):
		'''Retrieves force/torque offset biases.'''
		self.ft_bias=self.get_ave_wrench_client()

	def get_ee_pose(self):
		'''Retrieves current EE pose in Map frame.'''
		if self.gazebo:
			return self.get_pose_client()
		else:
			# req=GetTFRequest()
			# req.from='map'
			# req.to='panda_link8'
			# return self.get_tf_client(req)
			pass

	def move_relative(self,xyz_disp=(0.0,0.0,0.1),goal_time=0.5):
		'''Moves end-effector by a relative displacement in panda_link8 frame.'''
		# Convert point to map frame
		point_goal=self.transform_ee_to_map(xyz_disp)
		# Get current orientation
		current_pose=self.get_ee_pose()
		# Build complete pose goal
		pose_goal= geometry_msgs.msg.Pose()
		pose_goal.position = point_goal.point
		pose_goal.orientation = current_pose.pose.orientation
		# Move to desired pose
		self.move_to_pose(pose_goal,goal_time)

	def move_gripper(self, width, epsilon_inner=0.05, epsilon_outer=0.05, 
					 speed=0.1, force=40):
		'''Moves gripper to the specified width.'''
		#Build goal
		goal=franka_gripper.msg.GraspGoal()
		goal.width=width
		goal.epsilon.inner=epsilon_inner
		goal.epsilon.outer=epsilon_outer
		goal.speed=speed
		goal.force=force
		#Send
		rospy.loginfo('Sending gripper goal')
		self.gripper_client.send_goal(goal)
		rospy.loginfo('Waiting for gripper result')
		self.gripper_client.wait_for_result()

	def move_home(self, goal_time):
		'''Move to home position.'''
		self.move_to_joint_pos(self.joint_home,goal_time=0.5)

	# def move_to_plane(self,step,axis,max_disp,max_force,max_iter=500,
	# 				  orientation=None,goal_time=0.5,recalculate_bias=True):
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
		self.set_goal_time(goal_time)
		# Execute
		plan=self.move_group.go(joint_goal, wait=True)
		# Stop movement, clear targets
		self.stop_and_clear()

	def move_to_pose(self,pose_goal,goal_time=0.5):
		'''Move to pose in map frame.'''
		#Set goal time for trajectory
		self.set_goal_time(goal_time)
		#Plan to Pose goal
		self.move_group.set_pose_target(pose_goal)
		#Execute
		plan = self.move_group.go(wait=True)
		# Stop movement, clear targets
		self.stop_and_clear()

	# def rotate_to_step(self,angle,axis,max_angle,max_torque,
						 #max_iter=500,recalculate_bias=True):
		'''Incrementally rotate end-effector until torque theshold is exceeded.'''
		#TODO: Finish Implementation
		# if orientation is not None:
		# 	#Then use the given orientation
		# else:
		# 	#Use the current orientation

	def set_goal_time(self,goal_time):
		'''Sets trajectory goal time.'''
		rospy.set_param('/position_joint_trajectory_controller/constraints/goal_time',goal_time)

	def set_vel_scale(self,vel_scale):
		'''Sets velocity scaling (0,1] for motion.'''
		self.move_group.set_max_velocity_scaling_factor(vel_scale)

	def stop_and_clear(self):
		'''Stops residual motions. Clears movement targets.'''
		self.move_group.stop()
		self.move_group.clear_pose_targets()

	def transform_ee_to_map(self,point):
		'''Transforms point (x,y,z) in panda_link8 to map frame. Returns PointStamped.'''
		# Initialize objects
		h = Header()
		orig_point=geometry_msgs.msg.PointStamped()
		#Define original point as PointStamped
		h.frame_id='panda_link8'
		orig_point.header=h
		orig_point.point.x=point[0]
		orig_point.point.y=point[1]
		orig_point.point.z=point[2]
		#Wait for transform to become available
		self.transformer.waitForTransform("panda_link8", "map", rospy.Time(0), rospy.Duration(4.0))
		#Calculate transformed point
		return self.transformer.transformPoint('map',orig_point)
		
if __name__ == '__main__':
	rospy.init_node('commander_node')
	#Instantiate commander
	panda=Commander(gazebo=True) #Gazebo uses "arm". Physical uses "panda_arm"
	#Test __repr__ method
	#print panda

	#Test move_to_pose method
	# pose_goal = geometry_msgs.msg.Pose()
	# goal_time = 0.5
	# quat=tf_conversions.transformations.quaternion_from_euler(-1.5708, #roll
	# 														  0.7854,  #pitch
	# 														  0.0)     #yaw
	# pose_goal.orientation.x = quat[0]
	# pose_goal.orientation.y = quat[1]
	# pose_goal.orientation.z = quat[2]
	# pose_goal.orientation.w = quat[3]
	# pose_goal.position.x = 0.4
	# pose_goal.position.y = 0.2
	# pose_goal.position.z = 0.3
	# panda.move_to_pose(pose_goal,goal_time)

	#Test move_to_joint_pos method
	#panda.move_to_joint_pos(panda.joint_home,goal_time=0.8)

	#Test move_home method
	#panda.move_home(goal_time=0.8)

	#Test move_gripper method
	#panda.move_gripper(0.02)

	#Test get_ee_pose method
	print panda.get_ee_pose()

	#Test move_relative method
	panda.move_relative(xyz_disp=(0.0,0.0,0.15),goal_time=0.5)

	print panda.get_ee_pose()