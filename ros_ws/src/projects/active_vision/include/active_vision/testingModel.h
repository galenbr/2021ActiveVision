#ifndef TESTING_MODEL
#define TESTING_MODEL

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

// OpenCV specific includes
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

//Gazebo specific includes
#include <gazebo_msgs/SetModelState.h>

// Typedef for convinience
typedef pcl::PointCloud<pcl::PointXYZRGB> ptCldColor;

// Class to store data of environment and its processing
class environment{
private:
  ros::Rate r{10};
  ros::Publisher pubKinectPose;
  ros::Subscriber subKinectPtCld;
  ros::Subscriber subKinectRGB;
  ros::Subscriber subKinectDepth;
  int flag[3];

public:
  ptCldColor ptCldLast;  // Point cloud to store the environment
  cv_bridge::CvImageConstPtr ptrRgbLast;    // RGB image from camera
  cv_bridge::CvImageConstPtr ptrDepthLast;  // Depth map from camera

  environment(ros::NodeHandle *nh);

  // Callback function to point cloud subscriber
  void cbPtCld (const ptCldColor::ConstPtr& msg);

  // Callback function to RGB image subscriber
  void cbImgRgb (const sensor_msgs::ImageConstPtr& msg);

  // Callback function to RGB image subscriber
  void cbImgDepth (const sensor_msgs::ImageConstPtr& msg);

  // Function to move the kinect. Args: Array of X,Y,Z,Yaw,Pitch,Roll
  void moveKinect(float pose[6]);

  // Function to read the kinect data.
  void readKinect();
};

// A test function to check if the "moveKinect" function is working
void testKinectMovement(environment &av);

// A test function to check if the "readKinect" function is working
void testKinectRead(environment &av);

#endif