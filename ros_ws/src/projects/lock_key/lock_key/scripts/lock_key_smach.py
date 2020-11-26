#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
State machine for key-in-lock pipeline.
"""

import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class DetectKeyPose(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded','failed'])

	def execute(self, userdata):
		rospy.loginfo('Detecting Key Location')
		rospy.wait_for_service("findKeyServer")
		return result

class NavigateToKey(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded','failed'])

	def execute(self, userdata):
		rospy.loginfo('Navigating to Key')
		rospy.wait_for_service("...")
		return result

class GraspKey(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded','failed'])

	def execute(self, userdata):
		rospy.loginfo('Grasping Key')
		rospy.wait_for_service("...")
		return result

class NavigateToLock(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded','failed'])

	def execute(self, userdata):
		rospy.loginfo('Navigating to Lock')
		rospy.wait_for_service("...")
		return result

class MoveToInsertionPlane(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded','failed'])

	def execute(self, userdata):
		rospy.loginfo('Moving to Insertion Plane')
		rospy.wait_for_service("...")
		return result

class SpiralSearch(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded','failed'])

	def execute(self, userdata):
		rospy.loginfo('Beginning spiral search')
		rospy.wait_for_service("...")
		return result

class MoveDownward(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded','failed','stuck'])

	def execute(self, userdata):
		rospy.loginfo('Moving downward')
		rospy.wait_for_service("...")
		return result

class Jiggle(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded','still_stuck'])

	def execute(self, userdata):
		rospy.loginfo('Jiggling key')
		rospy.wait_for_service("...")
		return result

class OpenGripper(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded','failed'])

	def execute(self, userdata):
		rospy.loginfo('Opening gripper')
		rospy.wait_for_service("...")
		return result

class RestartFromHome(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded','failed'])

	def execute(self, userdata):
		rospy.loginfo('Move to home to restart procedure.')
		rospy.wait_for_service("...")
		return result

class NavigateToHome(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded'])

	def execute(self, userdata):
		rospy.loginfo('Move to home to finish procedure.')
		rospy.wait_for_service("...")
		return result

def main():
	rospy.init_node('place_key_in_lock')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['complete'])

	# Open the container
	with sm:
	    # Add states to the container
	    smach.StateMachine.add('DetectKeyPose', DetectKeyPose(), 
	                           transitions={'succeeded':'NavigateToKey', 
	                                        'failed':'RestartFromHome'})
	    smach.StateMachine.add('NavigateToKey', NavigateToKey(), 
	                           transitions={'succeeded':'GraspKey',  
	                                        'failed':'RestartFromHome'})
	    smach.StateMachine.add('GraspKey', GraspKey(), 
	                           transitions={'succeeded':'NavigateToLock', 
	                                        'failed':'RestartFromHome'})
	    smach.StateMachine.add('NavigateToLock', NavigateToLock(), 
	                           transitions={'succeeded':'InsertKey', 
	                                        'failed':'RestartFromHome'})

	    # Create sub SMACH state machine for key insertion
		key_sub_sm = smach.StateMachine(outcomes=['succeeded','failed'])
		# Open the sub-container
		with key_sub_sm:
			# Add states to the sub-container
		    smach.StateMachine.add('MoveToInsertionPlane', MoveToInsertionPlane(), 
		                           transitions={'succeeded':'SpiralSearch', 
		                                        'failed':'NavigateToLock'})
		    smach.StateMachine.add('SpiralSearch', SpiralSearch(), 
		                           transitions={'succeeded':'MoveDownward', 
		                                        'failed':'NavigateToLock'})
		    smach.StateMachine.add('MoveDownward', MoveDownward(), 
		                           transitions={'succeeded':'complete',
		                           				'failed':'failed',
		                           				'stuck':'Jiggle'})
		    smach.StateMachine.add('Jiggle', Jiggle(), 
		                           transitions={'succeeded':'MoveDownward',
		                           				'still_stuck':'Jiggle',})

		# Add more states to the higher level container
	    smach.StateMachine.add('InsertKey', key_sub_sm, 
	                           transitions={'succeeded':'OpenGripper', 
	                                        'failed':'NavigateToHome'})
	    smach.StateMachine.add('OpenGripper', OpenGripper(), 
	                           transitions={'succeeded':'NavigateToHome', 
	                                        'failed':'OpenGripper'})
	    smach.StateMachine.add('RestartFromHome', RestartFromHome(), 
	                           transitions={'succeeded':'DetectKeyPose',
	                           				'failed':'RestartFromHome'})
	    smach.StateMachine.add('NavigateToHome', NavigateToHome(), 
	                           transitions={'succeeded':'complete'})
	#Add subscriptions
	#rospy.Subscriber(topic,type,callback)

	# Execute SMACH plan
	# outcome = sm.execute()

	# Required for introspection ##
	#Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	# Execute the state machine
	outcome = sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()
	## End required section ##

if __name__ == '__main__':
    main()