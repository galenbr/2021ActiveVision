#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Testing python commands for finding lock and key
"""
from __future__ import print_function
import rospy
import smach
import lock_key_msgs.srv


def finder():
    '''Returns key and lock in map frame.'''
    rospy.wait_for_service('lock_and_key_poses')
    service_call = rospy.ServiceProxy('lock_and_key_poses', 
                                        lock_key_msgs.srv.GetLockKeyPoses)
    response = service_call()
    rospy.loginfo(response)

class FindLockKey(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        rospy.loginfo('Finding lock and key...')
        finder()
        return 'succeeded'

def main():
	rospy.init_node('test_smach_finder')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['complete'])

	# Open the container
	with sm:
	    # Add states to the container
	    smach.StateMachine.add('FindLockKey', FindLockKey(), 
	                           transitions={'succeeded':'complete',  
	                                        'failed':'complete'})

	# Execute the state machine
	outcome = sm.execute()

if __name__=='__main__':
	main()