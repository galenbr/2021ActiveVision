#! /usr/bin/env python
"""Data capture node for the Deep CNN network"""

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from std_srvs.srv import Empty
import argparse
import glob
import datetime
import time
import numpy as np
import cv2 as cv
import csv
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64
from relaxed_ik.msg import EEPoseGoals





running_joints_val = []
depth_image_data = None
state_error_data = None
value_to_move = [0.0, 0.0, 0.0, 0.0, 0.0, 1.66, 0.0]
ee_pose_data = []
i = 0
def generate(frame,joint_data,ee_pose_data):
    global i
    i += 1
    
    # frame = np.frombuffer(img_data.data, dtype=np.uint8).reshape(img_data.height, img_data.width, -1)
    # cv.namedWindow("win",cv.WINDOW_AUTOSIZE)
    cv.imwrite('/home/ajay/kaam/DR/catkin_ws_dr/src/mer_lab/ros_ws/src/projects/vision_controlled/relaxed_ik/dataset/img%s.jpg'%i,frame)
    # jointValues = data.position
    # q1 = jointValues[0]
    # q2 = jointValues[1]
    with open('/home/ajay/kaam/DR/catkin_ws_dr/src/mer_lab/ros_ws/src/projects/vision_controlled/relaxed_ik/dataset/joint%s.csv'%i, 'w') as csvFile:
        writer = csv.writer(csvFile, quoting=csv.QUOTE_NONE)
        writer.writerow(joint_data)
        csvFile.close()
    with open('/home/ajay/kaam/DR/catkin_ws_dr/src/mer_lab/ros_ws/src/projects/vision_controlled/relaxed_ik/dataset/ee_pose%s.csv'%i, 'w') as csvFile:
        writer = csv.writer(csvFile, quoting=csv.QUOTE_NONE)
        writer.writerow(ee_pose_data)
        csvFile.close()
    print('Generating datsets sample -- ',i)
    time.sleep(0.20)


start_flag = 0


def state_error(data):
    global depth_image_data, start_flag
   
    if start_flag:
        running_joints_val = data.actual.positions

        framed = np.frombuffer(depth_image_data.data, dtype=np.float32).reshape(depth_image_data.height, depth_image_data.width)
   
        framed = framed / 3.0
        framed = framed * 255
        framed = framed.astype(np.uint8)

        generate(framed, running_joints_val, ee_pose_data)
        start_flag = 0
        del ee_pose_data[:]

def cb_depth_image(data):
    # print("in data")
    global depth_image_data

    depth_image_data = data
    
def cb_ee_pose(data):
    global ee_pose_data, start_flag
    ee_pose_data.append(data.ee_poses[0].position.x)
    ee_pose_data.append(data.ee_poses[0].position.y)
    ee_pose_data.append(data.ee_poses[0].position.z)
    ee_pose_data.append(data.ee_poses[0].orientation.w)
    ee_pose_data.append(data.ee_poses[0].orientation.x)
    ee_pose_data.append(data.ee_poses[0].orientation.y)
    ee_pose_data.append(data.ee_poses[0].orientation.z)
    while True:
        print("waiting for joint values to settle down\n")
        time.sleep(1.5)
        start_flag = 1
        break
    

if __name__ == '__main__':
    rospy.init_node('data_capture_node', anonymous=True)
    print("init done")
    while (not rospy.is_shutdown()):

        rospy.Subscriber(name='/relaxed_ik/ee_pose_goals', data_class=EEPoseGoals,
                         callback=cb_ee_pose,
                         queue_size=10)
        
        rospy.Subscriber(name='/camera_link/depth/image_raw', data_class=numpy_msg(Image),
                         callback=cb_depth_image,
                         queue_size=10)

        rospy.Subscriber(name='/panda/arm_controller/state', data_class=JointTrajectoryControllerState,
                         callback=state_error,
                         queue_size=10)

        

        rospy.spin()

# print datetime.datetime.now()