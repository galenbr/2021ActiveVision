# -*- coding: utf-8 -*-
"""
ROS Service Server for retrieving lock and key positions.
"""
from __future__ import print_function

from lock_key_msgs.srv import GetLockKeyPoses
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import rospy
import tf

key_point={'x':[],'y':[],'z':[]}
lock_point={'x':[],'y':[],'z':[]}

def key_callback(key_msg):
	global key_point
	key_point['x']=key_msg.point.x
	key_point['y']=key_msg.point.y
	key_point['z']=key_msg.point.z

def lock_callback(lock_msg):
	global lock_point
	lock_point['x']=lock_msg.point.x
	lock_point['y']=lock_msg.point.y
	lock_point['z']=lock_msg.point.z

def ave(data):
	return sum(data)/len(data)

def get_poses(req):
	'''Returns poses of lock and key from vision system.'''
	key_point_samples={'x':[],'y':[],'z':[]}
	lock_point_samples={'x':[],'y':[],'z':[]}
	nsamples=30
	ii=0
	#Get average point for each
	while ii<=nsamples:
		rospy.Subscriber("/lock_point", PointStamped, lock_callback)
		rospy.Subscriber("/key_point", PointStamped, key_callback)
		for idx in ['x','y','z']:
			key_point_samples[idx].append(key_point[idx])
			lkey_point_samples[idx].append(lock_point[idx])
		ii+=1
		rospy.sleep(1/30)

	#Build PointStamped Messaged for lock and Key
	key_response=PointStamped()
	key_response.point.x=ave(key_point_samples['x'])
	key_response.point.y=ave(key_point_samples['y'])
	key_response.point.z=ave(key_point_samples['z'])
	lock_response=PointStamped()
	lock_response.point.x=ave(lock_point_samples['x'])
	lock_response.point.y=ave(lock_point_samples['y'])
	lock_response.point.z=ave(lock_point_samples['z'])
	h = std_msgs.msg.Header()
	h.stamp = rospy.Time.now()
	h.frame_id='camera_color_optical_frame'
	key_response.header=h
	lock_response.header=h
	#Transform msgs to /map frame
	key_response=tf.TransformerROS.transformPoint('/map',key_response)
	lock_response=tf.TransformerROS.transformPoint('/map',lock_response)
	#Define response
	response=GetLockKeyPosesResponse()
	response.key_point=key_response
	response.lock_point=lock_response
	return response

def lock_key_pose_server():
	rospy.init_node('lock_and_key_poses_server')
	s = rospy.Service('lock_and_key_poses', GetLockKeyPoses, get_poses)
	print("Ready to retrieve poses.")
	rospy.spin()

if __name__ == "__main__":
	lock_key_pose_server()