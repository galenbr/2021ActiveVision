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
from lock_key_msgs.srv import GetLockKeyPoses, GetLockKeyPosesResponse

# def move_to_position(x,y,z,roll,pitch,yaw):
# 	'''Moves EE to abs. pose in map frame.'''
# 	# Call move absolute action
# 	client = actionlib.SimpleActionClient('move_abs_server', 
# 										  lock_key.msg.MoveAbsAction)
# 	rospy.loginfo('Waiting for server')
# 	client.wait_for_server()
# 	rospy.loginfo('Found Server')
# 	goal=lock_key.msg.MoveAbsGoal(x,y,z,roll,pitch,yaw)
# 	rospy.loginfo('Sending Goal')
# 	client.send_goal(goal)
# 	rospy.loginfo('Waiting for result')
# 	client.wait_for_result()
# 	# Get result from action server
# 	#action_result=client.get_result()
# 	#print(action_result)

def get_positions():
	'''Retrieves key and lock positions.'''
	rospy.loginfo('Waiting for Service')
	rospy.wait_for_service('lock_and_key_poses')
	rospy.loginfo('Calling Service')
	service_call = rospy.ServiceProxy('lock_and_key_poses', 
									  GetLockKeyPoses)
	response = service_call()
	rospy.loginfo(response)

class GetPositions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
    def execute(self, userdata):
        rospy.loginfo('Getting positions...')
        get_positions()
        return 'succeeded'

# class NavigateToKey(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeeded','failed'])
#     def execute(self, userdata):
#         rospy.loginfo('Navigating to Key...')
#         move_to_position(0.4,-0.2,0.25,-pi/2,pi/4,0.0)
#         rospy.sleep(4.0)
#         return 'succeeded'

# class NavigateToLock(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeeded'])
#     def execute(self, userdata):
#         rospy.loginfo('Navigating to Lock...')
#         move_to_position(0.3,0.0,0.59,pi,0.0,-0.7854)
#         rospy.sleep(4.0)
#         return 'succeeded'

def main():
	rospy.init_node('test_smach')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['complete'])

	# Open the container
	with sm:
	    # Add states to the container
	    smach.StateMachine.add('GetPositions', GetPositions(), 
	                           transitions={'succeeded':'complete'})
	    # smach.StateMachine.add('NavigateToKey', NavigateToKey(), 
	    #                        transitions={'succeeded':'NavigateToLock',  
	    #                                     'failed':'NavigateToKey'})
	    # smach.StateMachine.add('NavigateToLock', NavigateToLock(), 
	    #                        transitions={'succeeded':'complete'})

	# Execute the state machine
	outcome = sm.execute()

if __name__=='__main__':
	main()