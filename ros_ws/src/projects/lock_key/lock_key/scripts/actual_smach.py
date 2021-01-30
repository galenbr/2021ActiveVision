#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SMACH for key insertion procedure
"""
from __future__ import print_function
import rospy
import smach
import actionlib
import lock_key.msg
import lock_key_msgs.msg
import franka_gripper.msg
import moveit_planner.srv
from lock_key_msgs.srv import GetLockKeyPoses, GetLockKeyPosesResponse

class Epsilon:
	'''Define Epsilon object for gripper function.'''
	def __init__(self, inner, outer):
		self.inner=inner
		self.outer=outer

def move_to_position(x,y,z,roll,pitch,yaw):
	'''Moves EE to abs. pose in map frame.'''
	# Call move absolute action
	client = actionlib.SimpleActionClient('move_abs_server', 
										  lock_key.msg.MoveAbsAction)
	rospy.loginfo('Waiting for server')
	client.wait_for_server()
	rospy.loginfo('Found Server')
	goal=lock_key.msg.MoveAbsGoal(x,y,z,roll,pitch,yaw)
	rospy.loginfo('Sending Goal')
	client.send_goal(goal)
	rospy.loginfo('Waiting for result')
	client.wait_for_result()
	# Get result from action server
	#action_result=client.get_result()
	#print(action_result)

def move_to_joint_position(j1,j2,j3,j4,j5,j6,j7):
	'''Moves arm to position in joint space (radians).'''
	rospy.wait_for_service('move_to_joint_space')
	joint_move = rospy.ServiceProxy('move_to_joint_space', 
									moveit_planner.srv.MoveJoint)
	val=[j1,j2,j3,j4,j5,j6,j7]
	execute=True
	response = joint_move(val,execute)

def move_gripper(width, epsilon_inner=0.05, epsilon_outer=0.05, speed=0.1, force=40):
	'''Moves gripper to the specified width.'''
	epsilon=Epsilon(epsilon_inner,epsilon_outer)
	client = actionlib.SimpleActionClient('franka_gripper/grasp', 
										  franka_gripper.msg.GraspAction)
	rospy.loginfo('Waiting for server')
	client.wait_for_server()
	rospy.loginfo('Found Server')
	goal=franka_gripper.msg.GraspGoal(width, epsilon, speed, force)
	rospy.loginfo('Sending Goal')
	client.send_goal(goal)
	rospy.loginfo('Waiting for result')
	client.wait_for_result()

def spiral_motion():
	'''Calls spiral motion action.'''
	client = actionlib.SimpleActionClient("spiral_motion", 
										  lock_key_msgs.msg.SpiralAction)
	rospy.loginfo('Waiting for server')
	client.wait_for_server()
	rospy.loginfo('Found Server')
	goal=lock_key_msgs.msg.SpiralGoal()
	rospy.loginfo('Sending Goal')
	client.send_goal(goal)
	rospy.loginfo('Waiting for result')
	client.wait_for_result()

def detect_plane(F_max, recalculate_bias=False):
	'''Calls insertion plane contact detection action.'''
	client = actionlib.SimpleActionClient("plane_detector_node", 
										  lock_key_msgs.msg.DetectPlaneAction)
	rospy.loginfo('Waiting for server')
	client.wait_for_server()
	rospy.loginfo('Found Server')
	goal=lock_key_msgs.msg.DetectPlaneGoal(F_max, recalculate_bias)
	rospy.loginfo('Sending Goal')
	client.send_goal(goal)
	rospy.loginfo('Waiting for result')
	client.wait_for_result()

def rotate_key(d_roll, d_pitch, d_yaw):
	'''Calls key rotation action.'''
	client = actionlib.SimpleActionClient("rotate_key", 
										  lock_key_msgs.msg.RotateKeyAction)
	rospy.loginfo('Waiting for server')
	client.wait_for_server()
	rospy.loginfo('Found Server')
	goal=lock_key_msgs.msg.RotateKeyGoal(d_roll, d_pitch, d_yaw)
	rospy.loginfo('Sending Goal')
	client.send_goal(goal)
	rospy.loginfo('Waiting for result')
	client.wait_for_result()

def get_positions():
	'''Retrieves key and lock positions from vision system.'''
	rospy.loginfo('Waiting for Service')
	rospy.wait_for_service('lock_and_key_poses')
	rospy.loginfo('Calling Service')
	service_call = rospy.ServiceProxy('lock_and_key_poses', 
									  GetLockKeyPoses)
	response = service_call()
	rospy.loginfo(response)
	return response

class GetPositions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Getting positions from vision system...')
        response=get_positions()
        # rospy.set_param("key_goal/x",response.key_point.point.x)
        # rospy.set_param("key_goal/y",response.key_point.point.y)
        # rospy.set_param("key_goal/z",response.key_point.point.z)
        rospy.set_param("padlock_goal/x",response.lock_point.point.x)
        rospy.set_param("padlock_goal/y",response.lock_point.point.y)
        # rospy.set_param("padlock_goal/z",response.lock_point.point.z)
        rospy.set_param("padlock_goal/z",0.26)
        return 'succeeded'

class NavigateToPreKey(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Navigating to Pre-Key...')
        # Retrieve goal position from param server
        goal=rospy.get_param("key_goal")
        goal['x']+=goal['pre_x_offset']
        goal['y']+=goal['pre_y_offset']
        goal['z']+=goal['pre_z_offset']
        # Call movement action
        move_to_position(goal['x'],goal['y'],goal['z'],
                         goal['roll'],goal['pitch'],goal['yaw'])
        rospy.sleep(3)
        return 'succeeded'

class NavigateToKey(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Navigating to Key...')
        # Retrieve goal position from param server
        goal=rospy.get_param("key_goal")
        # Call movement action
        move_to_position(goal['x'],goal['y'],goal['z'],
                         goal['roll'],goal['pitch'],goal['yaw'])
        rospy.sleep(3)
        return 'succeeded'

class NavigateToPostKey(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Navigating to Post-Key...')
        # Retrieve goal position from param server
        goal=rospy.get_param("key_goal")
        goal['x']+=goal['post_x_offset']
        goal['y']+=goal['post_y_offset']
        goal['z']+=goal['post_z_offset']
        # Call movement action
        move_to_position(goal['x'],goal['y'],goal['z'],
                         goal['roll'],goal['pitch'],goal['yaw'])
        rospy.sleep(3)
        return 'succeeded'

class CloseGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Closing Gripper')
        move_gripper(0.02)
        return 'succeeded'

class NavigateToPreLockFar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Navigating to Pre-Lock Far...')
        # Retrieve goal position from param server
        goal=rospy.get_param("padlock_goal")
        goal['x']+=goal['pre_x_offset_far']
        goal['y']+=goal['pre_y_offset_far']
        goal['z']+=goal['pre_z_offset_far']
        # Call movement action
        move_to_position(goal['x'],goal['y'],goal['z'],
                         goal['roll'],goal['pitch'],goal['yaw'])
        rospy.sleep(3)
        return 'succeeded'

class NavigateToPreLockClose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Navigating to Pre-Lock Close...')
        # Retrieve goal position from param server
        goal=rospy.get_param("padlock_goal")
        goal['x']+=goal['pre_x_offset_close']
        goal['y']+=goal['pre_y_offset_close']
        goal['z']+=goal['pre_z_offset_close']
        goal['x']+=goal['x_misalignment']
        goal['y']+=goal['y_misalignment']
        # Call movement action
        move_to_position(goal['x'],goal['y'],goal['z'],
                         goal['roll'],goal['pitch'],goal['yaw'])
        rospy.sleep(3)
        return 'succeeded'

class PlaneDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
		rospy.loginfo('Starting Plane Detection...')
		rospy.loginfo('Retrieving parameters')
		F_max=rospy.get_param("spiral/Ft")
		detect_plane(F_max,recalculate_bias=True)
		return 'succeeded'

class SpiralMotion(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
		rospy.loginfo('Starting Spiral Motion...')
		spiral_motion()
		return 'succeeded'

class FinalInsert(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
		rospy.loginfo('Starting Final Insertion...')
		rospy.loginfo('Retrieving parameters')
		F_max=rospy.get_param("spiral/Fd")
		detect_plane(F_max,recalculate_bias=False)
		return 'succeeded'

class RotateKey(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
		rospy.loginfo('Rotating key...')
		rospy.loginfo('Retrieving parameters')
		d_roll=rospy.get_param("padlock_goal/key_roll")
		d_pitch=rospy.get_param("padlock_goal/key_pitch")
		d_yaw=rospy.get_param("padlock_goal/key_yaw")
		rotate_key(d_roll,d_pitch,d_yaw)
		return 'succeeded'

class NavigateToPostLock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Navigating to Post-Lock ...')
        # Retrieve goal position from param server
        goal=rospy.get_param("padlock_goal")
        goal['x']+=goal['post_x_offset']
        goal['y']+=goal['post_y_offset']
        goal['z']+=goal['post_z_offset']
        # Call movement action
        move_to_position(goal['x'],goal['y'],goal['z'],
                         goal['roll'],goal['pitch'],goal['yaw'])
        return 'succeeded'

class OpenGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Opening Gripper')
        move_gripper(0.1)
        return 'succeeded'

class NavigateToHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Navigating to Home...')
        # Retrieve goal position from param server
        goal=rospy.get_param("home")
        move_to_joint_position(goal['j1'],goal['j2'],goal['j3'],goal['j4'],
        	                   goal['j5'],goal['j6'],goal['j7'])
        return 'succeeded'

def main():
	rospy.init_node('test_smach')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['complete'])

	# Open the container
	with sm:
	    # Add states to the container
		smach.StateMachine.add('GetPositions', GetPositions(), 
	                           transitions={'succeeded':'NavigateToPreKey',  
	                                        'failed':'GetPositions'})
		smach.StateMachine.add('NavigateToPreKey', NavigateToPreKey(), 
	                           transitions={'succeeded':'NavigateToKey',  
	                                        'failed':'NavigateToPreKey'})
		smach.StateMachine.add('NavigateToKey', NavigateToKey(), 
	                           transitions={'succeeded':'CloseGripper',  
	                                        'failed':'NavigateToKey'})
		smach.StateMachine.add('CloseGripper', CloseGripper(), 
	                           transitions={'succeeded':'NavigateToPostKey',  
	                                        'failed':'CloseGripper'})
		smach.StateMachine.add('NavigateToPostKey', NavigateToPostKey(), 
	                           transitions={'succeeded':'NavigateToPreLockFar',  
	                                        'failed':'NavigateToPostKey'})
		smach.StateMachine.add('NavigateToPreLockFar', NavigateToPreLockFar(), 
	                           transitions={'succeeded':'NavigateToPreLockClose', 
	                                        'failed':'NavigateToPreLockFar'})
		smach.StateMachine.add('NavigateToPreLockClose', NavigateToPreLockClose(), 
	                           transitions={'succeeded':'SpiralInsert_SM',
	                                        'failed':'NavigateToPreLockClose'})

		# Create the sub SMACH state machine
		sm_sub = smach.StateMachine(outcomes=['succeeded','failed'])
		with sm_sub:
			smach.StateMachine.add('PlaneDetection', PlaneDetection(), 
									transitions={'succeeded':'SpiralMotion'})
			smach.StateMachine.add('SpiralMotion', SpiralMotion(), 
									transitions={'succeeded':'FinalInsert'})
			smach.StateMachine.add('FinalInsert', FinalInsert(), 
									transitions={'succeeded':'RotateKey'})												 
			smach.StateMachine.add('RotateKey', RotateKey(), 
									transitions={'succeeded':'succeeded',  
												 'failed':'failed'})

		smach.StateMachine.add('SpiralInsert_SM', sm_sub, 
	                           transitions={'succeeded':'OpenGripper',  
	                                        'failed':'NavigateToPreLockFar'})

		smach.StateMachine.add('OpenGripper', OpenGripper(), 
	                           transitions={'succeeded':'NavigateToPostLock',  
	                                        'failed':'OpenGripper'})
		smach.StateMachine.add('NavigateToPostLock', NavigateToPostLock(), 
	                           transitions={'succeeded':'NavigateToHome',
	                                        'failed':'NavigateToPostLock'})
		smach.StateMachine.add('NavigateToHome', NavigateToHome(), 
	                           transitions={'succeeded':'complete',  
	                                        'failed':'NavigateToHome'})

	# Execute the state machine
	outcome = sm.execute()

if __name__=='__main__':
	main()