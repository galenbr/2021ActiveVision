#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS Service Server for retrieving lock and key positions.
"""
from __future__ import print_function

from lock_key_msgs.srv import GetLockKeyPoses, GetLockKeyPosesResponse
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from collections import deque
import rospy
import tf
from tf2_ros import TransformException

nsamples=30
key_point={'x':deque(maxlen=nsamples),
		   'y':deque(maxlen=nsamples),
		   'z':deque(maxlen=nsamples)}
lock_point={'x':deque(maxlen=nsamples),
		    'y':deque(maxlen=nsamples),
		    'z':deque(maxlen=nsamples)}

def key_callback(key_msg):
	global key_point
	key_point['x'].append(key_msg.point.x)
	key_point['y'].append(key_msg.point.y)
	key_point['z'].append(key_msg.point.z)

def lock_callback(lock_msg):
	global lock_point
	lock_point['x'].append(lock_msg.point.x)
	lock_point['y'].append(lock_msg.point.y)
	lock_point['z'].append(lock_msg.point.z)

def ave(data):
	return sum(data)/len(data)

def get_poses(req):
	'''Returns poses of lock and key from vision system.'''
	global key_point, lock_point
	pose_retrieved=False
	while not pose_retrieved:
		try:
			#Build PointStamped Messaged for lock and Key
			key_response=PointStamped()
			key_response.point.x=ave(key_point['x'])
			key_response.point.y=ave(key_point['y'])
			key_response.point.z=ave(key_point['z'])
			lock_response=PointStamped()
			lock_response.point.x=ave(lock_point['x'])
			lock_response.point.y=ave(lock_point['y'])
			lock_response.point.z=ave(lock_point['z'])
			h = Header()
			h.stamp = rospy.Time.now()
			h.frame_id='camera_color_optical_frame'
			key_response.header=h
			lock_response.header=h
			#Transform msgs to map frame
			transformer=tf.TransformListener()
			transformer.waitForTransform(h.frame_id, "map", rospy.Time(0), rospy.Duration(5.0))
			key_response=transformer.transformPoint('map',key_response)
			lock_response=transformer.transformPoint('map',lock_response)
			#Define response
			response=GetLockKeyPosesResponse()
			response.key_point=key_response
			response.lock_point=lock_response
			pose_retrieved=True
		#If call fails due to "Transform lookup in past" error, then try again
		except TransformException:
			pass
	return response

def lock_key_pose_server():
	rospy.init_node('lock_and_key_poses_server')
	rospy.Subscriber("/lock_point", PointStamped, lock_callback)
	rospy.Subscriber("/key_point", PointStamped, key_callback)
	s = rospy.Service('lock_and_key_poses', GetLockKeyPoses, get_poses)
	print("Ready to retrieve poses.")
	rospy.spin()

if __name__ == "__main__":
	lock_key_pose_server()