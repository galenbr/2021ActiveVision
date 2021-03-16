#!/usr/bin/env python

from __future__ import print_function
from rrbot_pcl.srv import skelMsg, skelMsgResponse
from skimage.morphology import skeletonize
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import rospy
import roslib; roslib.load_manifest('rrbot_pcl')

def handle_skel_msg(req):
    print("Received req")
    bridge = CvBridge()
    req.img.encoding='bgr8'
    cv_image = bridge.imgmsg_to_cv2(req.img, desired_encoding='passthrough')
    # img_msg = bridge.cv2_to_imgmsg(cv_image, encoding='passthrough')
    # return img_msg
    return skelMsgResponse()

def fourier_param_server():
    rospy.init_node('skeletonization_node')
    skeletonizationServer = rospy.Service('skeletonization', skelMsg, handle_skel_msg)
    print("Skeletonization Service Ready")
    rospy.spin()

if __name__ == "__main__":
    fourier_param_server()