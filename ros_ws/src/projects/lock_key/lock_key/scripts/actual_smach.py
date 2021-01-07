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
import franka_gripper.msg
import moveit_planner.srv

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
	goal=franka_gripper.msg.GraspGoal(width, epsilon, speed,force)
	rospy.loginfo('Sending Goal')
	client.send_goal(goal)
	rospy.loginfo('Waiting for result')
	client.wait_for_result()

def spiral_insert(Ft, Fd, Fi, delta_max):
	'''Calls spiral insert action.'''
	client = actionlib.SimpleActionClient("spiral_insert_key", 
										  lock_key.msg.SpiralInsertAction)
	rospy.loginfo('Waiting for server')
	client.wait_for_server()
	rospy.loginfo('Found Server')
	goal=lock_key.msg.SpiralInsertGoal(Ft, Fd, Fi, delta_max)
	rospy.loginfo('Sending Goal')
	client.send_goal(goal)
	rospy.loginfo('Waiting for result')
	client.wait_for_result()

class NavigateToPreKey(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Navigating to Pre-Key...')
        # Retrieve goal position from param server
        goal=rospy.get_param("key_goal")
        goal['z']+=goal['z_offset']
        # Call movement action
        move_to_position(goal['x'],goal['y'],goal['z'],
                         goal['roll'],goal['pitch'],goal['yaw'])
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
        return 'succeeded'

class NavigateToPostKey(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Navigating to Post-Key...')
        # Retrieve goal position from param server
        goal=rospy.get_param("key_goal")
        goal['z']+=goal['z_offset']
        # Call movement action
        move_to_position(goal['x'],goal['y'],goal['z'],
                         goal['roll'],goal['pitch'],goal['yaw'])
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
        goal['z']+=goal['z_offset_far']
        # Call movement action
        move_to_position(goal['x'],goal['y'],goal['z'],
                         goal['roll'],goal['pitch'],goal['yaw'])
        return 'succeeded'

class NavigateToPreLockClose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Navigating to Pre-Lock Close...')
        # Retrieve goal position from param server
        goal=rospy.get_param("padlock_goal")
        goal['x']+=goal['x_misalignment']
        goal['y']+=goal['y_misalignment']
        goal['z']+=goal['z_offset_close']
        # Call movement action
        move_to_position(goal['x'],goal['y'],goal['z'],
                         goal['roll'],goal['pitch'],goal['yaw'])
        return 'succeeded'

class SpiralInsert(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
		rospy.loginfo('Starting Spiral Insert...')
		rospy.loginfo('Retrieving parameters')
		spiral_params=rospy.get_param("spiral")
		spiral_insert(spiral_params['Ft'], spiral_params['Fd'], 
			          spiral_params['Fi'], spiral_params['delta_max'])
		return 'succeeded'

class NavigateToPostLock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Navigating to Post-Lock ...')
        # Retrieve goal position from param server
        goal=rospy.get_param("padlock_goal")
        goal['z']+=goal['z_offset_far']
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
	                           transitions={'succeeded':'SpiralInsert',
	                                        'failed':'NavigateToPreLockClose'})
	    smach.StateMachine.add('SpiralInsert', SpiralInsert(), 
	                           transitions={'succeeded':'OpenGripper',
	                                        'failed':'SpiralInsert'})
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