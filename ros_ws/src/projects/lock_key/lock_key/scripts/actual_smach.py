#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
State machine for key-in-lock pipeline.
"""

from __future__ import print_function
import rospy
import smach
import smach_ros
import actionlib
import lock_key.msg

class NavigateToKey(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded','failed'])

	def execute(self, userdata):
		rospy.loginfo('Navigating to Key')
		# Retrieve goal position from param server
		goal_x=rospy.get_param("key_goal_x")
		goal_y=rospy.get_param("key_goal_y")
		goal_z=rospy.get_param("key_goal_z")
		goal_roll=0.0
		goal_pitch=0.0
		goal_yaw=0.0
		# Call move absolute action
		client = actionlib.SimpleActionClient('move_abs_server', 
											  lock_key.msg.MoveAbsAction)
		client.wait_for_server()
		goal=lock_key.msg.MoveAbsAction(goal_x,goal_y,goal_z,
										goal_roll,goal_pitch,goal_yaw)
		client.send_goal(goal)
		client.wait_for_result()
		# Get result from action server
		action_result=client.get_result()
		# Define smach result here (should be updated)
		result='succeeded'
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
		# Retrieve goal position from param server
		goal_x=rospy.get_param("padlock_goal_x")
		goal_y=rospy.get_param("padlock_goal_y")
		goal_z=rospy.get_param("padlock_goal_z")
		goal_roll=0.0
		goal_pitch=0.0
		goal_yaw=0.0
		# Call move absolute action
		client = actionlib.SimpleActionClient('move_abs_server', 
											  lock_key.msg.MoveAbsAction)
		client.wait_for_server()
		goal=lock_key.msg.MoveAbsAction(goal_x,goal_y,goal_z,
										goal_roll,goal_pitch,goal_yaw)
		client.send_goal(goal)
		client.wait_for_result()
		# Get result from action server
		action_result=client.get_result()
		# Define smach result here (should be updated)
		result='succeeded'
		return result

class OpenGripper(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded','failed'])

	def execute(self, userdata):
		rospy.loginfo('Opening Gripper')
		rospy.wait_for_service("...")
		return result

class RestartFromHome(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded','failed'])

	def execute(self, userdata):
		rospy.loginfo('Navigating to Lock')
		# Retrieve goal position from param server
		goal_x=rospy.get_param("home_goal_x")
		goal_y=rospy.get_param("home_goal_y")
		goal_z=rospy.get_param("home_goal_z")
		goal_roll=0.0
		goal_pitch=0.0
		goal_yaw=0.0
		# Call move absolute action
		client = actionlib.SimpleActionClient('move_abs_server', 
											  lock_key.msg.MoveAbsAction)
		client.wait_for_server()
		goal=lock_key.msg.MoveAbsAction(goal_x,goal_y,goal_z,
										goal_roll,goal_pitch,goal_yaw)
		client.send_goal(goal)
		client.wait_for_result()
		# Get result from action server
		action_result=client.get_result()
		# Define smach result here (should be updated)
		result='succeeded'
		return result

class NavigateToHome(smach.State):
	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded','failed'])

	def execute(self, userdata):
		rospy.loginfo('Navigating to Lock')
		# Retrieve goal position from param server
		goal_x=rospy.get_param("home_goal_x")
		goal_y=rospy.get_param("home_goal_y")
		goal_z=rospy.get_param("home_goal_z")
		goal_roll=0.0
		goal_pitch=0.0
		goal_yaw=0.0
		# Call move absolute action
		client = actionlib.SimpleActionClient('move_abs_server', 
											  lock_key.msg.MoveAbsAction)
		client.wait_for_server()
		goal=lock_key.msg.MoveAbsAction(goal_x,goal_y,goal_z,
										goal_roll,goal_pitch,goal_yaw)
		client.send_goal(goal)
		client.wait_for_result()
		# Get result from action server
		action_result=client.get_result()
		# Define smach result here (should be updated)
		result='succeeded'
		return result

def main():
	rospy.init_node('place_key_in_lock')

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['complete'])

	# Open the container
	with sm:
	    # Add states to the container
	    smach.StateMachine.add('NavigateToKey', NavigateToKey(), 
	                           transitions={'succeeded':'GraspKey',  
	                                        'failed':'RestartFromHome'})
	    smach.StateMachine.add('GraspKey', GraspKey(), 
	                           transitions={'succeeded':'NavigateToLock', 
	                                        'failed':'RestartFromHome'})
	    smach.StateMachine.add('NavigateToLock', NavigateToLock(), 
	                           transitions={'succeeded':'InsertKey', 
	                                        'failed':'RestartFromHome'})
	    smach.StateMachine.add('OpenGripper', OpenGripper(), 
	                           transitions={'succeeded':'NavigateToHome', 
	                                        'failed':'OpenGripper'})
	    smach.StateMachine.add('RestartFromHome', RestartFromHome(), 
	                           transitions={'succeeded':'NavigateToKey',
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