#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Testing python commands for SMACH

roslaunch lock_key sim.launch
rosrun lock_key move_abs_node
python test_move.py

"""
from __future__ import print_function
import rospy
import smach
import actionlib
import lock_key.msg
from math import pi

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

class NavigateToKey(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Navigating to Key...')
        move_to_position(0.3,0.1,0.5,pi,0.0,0.0)
        return 'succeeded'

class NavigateToLock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Navigating to Lock...')
        move_to_position(0.3,-0.1,0.6,pi,0.0,0.0)
        return 'succeeded'

def main():
	rospy.init_node('test_smach')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['complete'])

	# Open the container
	with sm:
	    # Add states to the container
	    smach.StateMachine.add('NavigateToKey', NavigateToKey(), 
	                           transitions={'succeeded':'NavigateToLock',  
	                                        'failed':'NavigateToKey'})
	    smach.StateMachine.add('NavigateToLock', NavigateToLock(), 
	                           transitions={'succeeded':'complete', 
	                                        'failed':'NavigateToLock'})

	# Execute the state machine
	outcome = sm.execute()

if __name__=='__main__':
	main()