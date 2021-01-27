#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Testing point transform

rosrun tf static_transform_publisher 1.0 0.0 0.0 0.0 0.0 0.0 base_link map 10
"""
from __future__ import print_function
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
import tf

def main():
	rospy.init_node('test_tf_point')
	rospy.loginfo('Initialized Node.')
	
	transformer=tf.TransformListener()
	transformer.waitForTransform("base_link", "map", rospy.Time(0), rospy.Duration(4.0))
	rospy.loginfo('Transforming point')
	#Define original point
	h = Header()
	key_point=PointStamped()
	# h.stamp = rospy.Time.now()
	h.frame_id='base_link'
	key_point.header=h
	key_point.point.x=1.0
	key_point.point.y=1.0
	key_point.point.z=1.0
	rospy.loginfo('Original point:')
	rospy.loginfo(key_point)
	#Calculate transformed point
	key_point_transformed=PointStamped()
	key_point_transformed=transformer.transformPoint('map',key_point)
	rospy.loginfo('Transformed point:')
	rospy.loginfo(key_point_transformed)

if __name__=='__main__':
	main()