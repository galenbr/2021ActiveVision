#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SMACH for key insertion procedure
"""
from __future__ import print_function
import rospy
import smach
import actionlib
import commander
import tf_conversions
#Messages
import geometry_msgs.msg
import lock_key.msg
import lock_key_msgs.msg
#Services
from lock_key_msgs.srv import GetLockKeyPoses, GetLockKeyPosesResponse
import moveit_planner.srv

def move_to_pose_goal(x,y,z,roll,pitch,yaw,goal_time=0.5):
	'''Moves EE to abs. pose in map frame. Wrapper for panda commander.'''
	global panda
	pose_goal = geometry_msgs.msg.Pose()
	quat=tf_conversions.transformations.quaternion_from_euler(roll,pitch,yaw)
	pose_goal.orientation.x = quat[0]
	pose_goal.orientation.y = quat[1]
	pose_goal.orientation.z = quat[2]
	pose_goal.orientation.w = quat[3]
	pose_goal.position.x = x
	pose_goal.position.y = y
	pose_goal.position.z = z

	panda.move_to_pose(pose_goal,goal_time)

def rotate_key(d_roll, d_pitch, d_yaw):
	'''Calls key rotation action. No torque feedback.'''
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
        #Define offsets
        ee_to_link8_offset=0.103
        x_offset=0.009 #Half of finger width
        height_safety_offset_key=0.008
        height_safety_offset_lock=0.02

        rospy.set_param("key_goal/x",response.key_point.point.x-x_offset)
        rospy.set_param("key_goal/y",response.key_point.point.y-ee_to_link8_offset)
        # rospy.set_param("key_goal/z",0.1485)
        rospy.set_param("key_goal/z",response.key_point.point.z+height_safety_offset_key)
        rospy.set_param("padlock_goal/x",response.lock_point.point.x-x_offset)
        rospy.set_param("padlock_goal/y",response.lock_point.point.y)
        rospy.set_param("padlock_goal/z",response.lock_point.point.z+ee_to_link8_offset+height_safety_offset_lock)
        # rospy.set_param("padlock_goal/z",0.116+ee_to_link8_offset)
        return 'succeeded'

class NavigateToPreKey(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        global panda
        rospy.loginfo('Navigating to Pre-Key...')
        # Retrieve goal position from param server
        goal=rospy.get_param("key_goal")
        goal['x']+=goal['pre_x_offset']
        goal['y']+=goal['pre_y_offset']
        goal['z']+=goal['pre_z_offset']
        #Make scale faster
        panda.set_vel_scale(0.2)
        # Call movement action
        move_to_pose_goal(goal['x'],goal['y'],goal['z'],
                          goal['roll'],goal['pitch'],goal['yaw'],goal_time=5.0)
        rospy.sleep(1)
        return 'succeeded'

class NavigateToKey(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        global panda
        rospy.loginfo('Navigating to Key...')
        #Make scale slower
        panda.set_vel_scale(0.1)
        panda.move_to_plane(step=0.0018,axis='z',max_disp=0.2,max_force=2.0,max_iter=500,
			    		    orientation=None,step_goal_time=0.5,recalculate_bias=True,
                            hold_xyz=[True,False,True])
        # # Retrieve goal position from param server
        # goal=rospy.get_param("key_goal")
        # # Call movement action
        # move_to_pose_goal(goal['x'],goal['y'],goal['z'],
        #                   goal['roll'],goal['pitch'],goal['yaw'],goal_time=3.0)
        rospy.sleep(1)
        return 'succeeded'

class NavigateToPostKey(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        global panda
        rospy.loginfo('Navigating to Post-Key...')
        #Make scale faster
        panda.set_vel_scale(0.15)
        # Retrieve goal position from param server
        goal=rospy.get_param("key_goal")
        goal['x']+=goal['post_x_offset']
        goal['y']+=goal['post_y_offset']
        goal['z']+=goal['post_z_offset']
        # Call movement action
        move_to_pose_goal(goal['x'],goal['y'],goal['z'],
                          goal['roll'],goal['pitch'],goal['yaw'],goal_time=3.0)
        rospy.sleep(1)
        return 'succeeded'

class NavigateToPostKeyRotated(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        global panda
        rospy.loginfo('Navigating to Post-Key Rotated...')
        #Make scale slower
        panda.set_vel_scale(0.1)
        # Retrieve goal position from param server
        goal=rospy.get_param("key_goal")
        goal['x']+=goal['post_x_offset']
        goal['y']+=goal['post_y_offset']
        goal['z']+=goal['post_z_offset']
        padlock_goal=rospy.get_param("padlock_goal")
        # Call movement action
        move_to_pose_goal(goal['x'],goal['y'],goal['z'],
                          padlock_goal['roll'],padlock_goal['pitch'],padlock_goal['yaw'],goal_time=4.0)
        rospy.sleep(1)
        return 'succeeded'

class CloseGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
    	global panda
        rospy.loginfo('Closing Gripper')
        panda.move_gripper(0.02)
        return 'succeeded'

class NavigateToPreLockFar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        global panda
        rospy.loginfo('Navigating to Pre-Lock Far...')
        #Make scale faster
        panda.set_vel_scale(0.15)
        # Retrieve goal position from param server
        goal=rospy.get_param("padlock_goal")
        goal['x']+=goal['pre_x_offset_far']
        goal['y']+=goal['pre_y_offset_far']
        goal['z']+=goal['pre_z_offset_far']
        # Call movement action
        move_to_pose_goal(goal['x'],goal['y'],goal['z'],
                          goal['roll'],goal['pitch'],goal['yaw'],goal_time=4.0)
        rospy.sleep(1)
        return 'succeeded'

class NavigateToPreLockClose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        global panda
        rospy.loginfo('Navigating to Pre-Lock Close...')
        #Make scale slower
        panda.set_vel_scale(0.1)
        # Retrieve goal position from param server
        goal=rospy.get_param("padlock_goal")
        goal['x']+=goal['pre_x_offset_close']
        goal['y']+=goal['pre_y_offset_close']
        goal['z']+=goal['pre_z_offset_close']
        move_to_pose_goal(goal['x'],goal['y'],goal['z'],
                          goal['roll'],goal['pitch'],goal['yaw'],goal_time=3.0)
        rospy.sleep(1)
        return 'succeeded'

class PlaneDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
		rospy.loginfo('Starting Plane Detection...')
		rospy.loginfo('Retrieving parameters')
		F_max=rospy.get_param("spiral/Ft")
		max_disp=rospy.get_param("spiral/delta_max")
		delta_step=rospy.get_param("spiral/delta_step")
		panda.move_to_plane(step=delta_step,axis='z',max_disp=max_disp,max_force=F_max,max_iter=500,
			    		    orientation=None,step_goal_time=0.5,recalculate_bias=True,hold_xyz=[True,True,False])
		return 'succeeded'

class SpiralMotion(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        global panda
        rospy.loginfo('Starting Spiral Motion...')
        a=rospy.get_param("spiral/a")
        b=rospy.get_param("spiral/b")
        min_spiral_force=rospy.get_param("spiral/min_spiral_force")
        tx_limit=rospy.get_param("spiral/Tx")
        ty_limit=rospy.get_param("spiral/Ty")
        F_max=rospy.get_param("spiral/Fd")
        spiral_rot=rospy.get_param("spiral/rot")
        nmax=rospy.get_param("spiral/nmax")
        panda.spiral(spiral_a=a,spiral_b=b,min_spiral_force=min_spiral_force,tx_limit=tx_limit,ty_limit=ty_limit,
                        spiral_rot=spiral_rot,spiral_fd=F_max,max_iter=nmax,step_goal_time=0.5)
        return 'succeeded'

class FinalInsert(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
		global panda
		rospy.loginfo('Starting Final Insertion...')
		rospy.loginfo('Retrieving parameters')
		F_max=rospy.get_param("spiral/Fi")
		max_disp=rospy.get_param("spiral/delta_max")
		delta_step=rospy.get_param("spiral/delta_step")
		panda.move_to_plane(step=delta_step,axis='z',max_disp=max_disp,max_force=F_max,max_iter=500,
			    		    orientation=None,step_goal_time=0.5,recalculate_bias=True,hold_xyz=[True,True,False])
		return 'succeeded'

class RotateKey(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
		global panda
		rospy.loginfo('Rotating key...')
		rospy.loginfo('Retrieving parameters')
		d_roll=rospy.get_param("padlock_goal/key_roll")
		d_pitch=rospy.get_param("padlock_goal/key_pitch")
		d_yaw=rospy.get_param("padlock_goal/key_yaw")
		# rotate_key(d_roll,d_pitch,d_yaw)
		panda.rotate_to_torque(step_angle=-0.0174533,axis='z',max_angle=d_yaw,max_torque=1.0,
			   			       max_iter=500,step_goal_time=0.5,recalculate_bias=True)

        #Rotate key back
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
        move_to_pose_goal(goal['x'],goal['y'],goal['z'],
                          goal['roll'],goal['pitch'],goal['yaw'],goal_time=2.0)
        return 'succeeded'

class OpenGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
    	global panda
        rospy.loginfo('Opening Gripper')
        panda.move_gripper(0.1)
        return 'succeeded'

class NavigateToHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
    def execute(self, userdata):
        global panda
        #Make scale faster
        panda.set_vel_scale(0.2)
        rospy.loginfo('Navigating to Home...')
        # Retrieve goal position from param server
        panda.move_home(goal_time=1.0)
        return 'succeeded'

def main():
	global panda
	rospy.init_node('actual_smach_node')

	#Initialize panda controller
	panda=commander.Commander(group_name="panda_arm",vel_scale=0.1,gazebo=False)

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
	                           transitions={'succeeded':'NavigateToPostKeyRotated',  
	                                        'failed':'NavigateToPostKey'})
		smach.StateMachine.add('NavigateToPostKeyRotated', NavigateToPostKeyRotated(), 
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