#ifndef TESTING_MODEL
#define TESTING_MODEL

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <array>
#include <string>
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
#include <pcl/visualization/cloud_viewer.h>

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
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
  int flag[3];

public:
  ptCldColor::Ptr ptrPtCldLast;             // Point cloud to store the environment
  ptCldColor::ConstPtr cPtrPtCldLast;       // Constant pointer
  cv_bridge::CvImageConstPtr ptrRgbLast;    // RGB image from camera
  cv_bridge::CvImageConstPtr ptrDepthLast;  // Depth map from camera

  ptCldColor::Ptr ptrPtCldTemp;             // Point cloud to store temporarily
  ptCldColor::ConstPtr cPtrPtCldTemp;       // Constant pointer
  ptCldColor::Ptr ptrPtCldEnv;              // Point cloud to store fused data
  ptCldColor::ConstPtr cPtrPtCldEnv;        // Constant pointer

  std::vector<float> lastKinectPose;

  environment(ros::NodeHandle *nh);

  // Callback function to point cloud subscriber
  void cbPtCld (const ptCldColor::ConstPtr& msg);

  // Callback function to RGB image subscriber
  void cbImgRgb (const sensor_msgs::ImageConstPtr& msg);

  // Callback function to RGB image subscriber
  void cbImgDepth (const sensor_msgs::ImageConstPtr& msg);

  // Function to move the kinect. Args: Array of X,Y,Z,Roll,Pitch,Yaw
  void moveKinect(std::vector<float> pose);

  // Function to read the kinect data.
  void readKinect();

  // Function to Fuse last data with existing data
  void fuseLastData();
};

// A test function to check if the "moveKinect" function is working
void testKinectMovement(environment &av);

// A test function to check if the "readKinect" function is working
void testKinectRead(environment &av);

// A test function to check fusing of data
void testPtCldFuse(environment &av);

#endif
