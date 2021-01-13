# -*- coding: utf-8 -*-
"""
ROS Service Server for retrieving lock and key positions.
"""
from __future__ import print_function

from lock_key_msgs.srv import GetLockKeyPoses
from geometry_msgs.msg import PointStamped
import rospy

def key_callback():
	pass

def lock_callback():
	pass

def get_poses(req):
    rospy.Subscriber("/lock_point", PointStamped, lock_callback)
    rospy.Subscriber("/key_point", PointStamped, key_callback)
    response=GetLockKeyPosesResponse()
    #Fillout response details...
    return response

def lock_key_pose_server():
    rospy.init_node('lock_and_key_poses_server')
    s = rospy.Service('lock_and_key_poses', GetLockKeyPoses, get_poses)
    print("Ready to retrieve poses.")
    rospy.spin()

if __name__ == "__main__":
    lock_key_pose_server()