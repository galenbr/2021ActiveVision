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
#include <ros/package.h>
#include <tf/transform_datatypes.h>

// OpenCV specific includes
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/generate.h>
#include <pcl/common/random.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>

//Gazebo specific includes
#include <gazebo_msgs/SetModelState.h>

// Typedef for convinience
typedef pcl::PointCloud<pcl::PointXYZRGB> ptCldColor;

void rbgPtCldViewer(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::string name,int vp);

Eigen::Affine3f homoMatTranspose(Eigen::Affine3f tf);

// Class to store data of environment and its processing
class environment{
private:
  ros::Rate r{10};
  ros::Publisher pubKinectPose;
  ros::Subscriber subKinectPtCld;
  ros::Subscriber subKinectRGB;
  ros::Subscriber subKinectDepth;

  pcl::PassThrough<pcl::PointXYZRGB> pass;         // Passthrough filter
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;      // VoxelGrid object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;      // Segmentation object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;   // Extracting object
  pcl::ConvexHull<pcl::PointXYZRGB> cvHull;        // Convex hull object
  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;  // Prism object

  Eigen::MatrixXf projectionMat;
  Eigen::Affine3f tfKinOptGaz;     // Transform : Kinect Optical Frame to Kinect Gazebo frame
  Eigen::Affine3f tfGazWorld;      // Transform : Kinect Gazebo Frame to Gazebo World frame

  int flag[3];
  int scale;
  float fingerZOffset;

  std::string path;                 // Path the active vision package

public:
  cv_bridge::CvImageConstPtr ptrRgbLast{new cv_bridge::CvImage};    // RGB image from camera
  cv_bridge::CvImageConstPtr ptrDepthLast{new cv_bridge::CvImage};  // Depth map from camera

  ptCldColor::Ptr ptrPtCldLast{new ptCldColor};                     // Point cloud to store the environment
  ptCldColor::ConstPtr cPtrPtCldLast{ptrPtCldLast};                 // Constant pointer

  ptCldColor::Ptr ptrPtCldTemp{new ptCldColor};                     // Point cloud to store temporarily
  ptCldColor::ConstPtr cPtrPtCldTemp{ptrPtCldTemp};                 // Constant pointer

  ptCldColor::Ptr ptrPtCldEnv{new ptCldColor};                      // Point cloud to store fused data
  ptCldColor::ConstPtr cPtrPtCldEnv{ptrPtCldEnv};                   // Constant pointer

  ptCldColor::Ptr ptrPtCldTable{new ptCldColor};                    // Point cloud to store table data
  ptCldColor::ConstPtr cPtrPtCldTable{ptrPtCldTable};               // Constant pointer

  ptCldColor::Ptr ptrPtCldHull{new ptCldColor};                     // Point cloud to store convex hull data
  ptCldColor::ConstPtr cPtrPtCldHull{ptrPtCldHull};                 // Constant pointer

  ptCldColor::Ptr ptrPtCldObject{new ptCldColor};                   // Point cloud to store object data
  ptCldColor::ConstPtr cPtrPtCldObject{ptrPtCldObject};             // Constant pointer

  ptCldColor::Ptr ptrPtCldUnexp{new ptCldColor};                    // Point cloud to store unexplored points data
  ptCldColor::ConstPtr cPtrPtCldUnexp{ptrPtCldUnexp};               // Constant pointer

  ptCldColor::Ptr ptrPtCldGripper{new ptCldColor};                  // Point cloud to store gripper (hand+fingers)
  ptCldColor::ConstPtr cPtrPtCldGripper{ptrPtCldGripper};           // Constant pointer

  ptCldColor::Ptr ptrPtCldGrpHnd{new ptCldColor};                   // Point cloud to store gripper hand
  ptCldColor::ConstPtr cPtrPtCldGrpHnd{ptrPtCldGrpHnd};             // Constant pointer

  ptCldColor::Ptr ptrPtCldGrpRfgr{new ptCldColor};                  // Point cloud to store gripper right finger
  ptCldColor::ConstPtr cPtrPtCldGrpRfgr{ptrPtCldGrpRfgr};           // Constant pointer

  ptCldColor::Ptr ptrPtCldGrpLfgr{new ptCldColor};                  // Point cloud to store gripper left finger
  ptCldColor::ConstPtr cPtrPtCldGrpLfgr{ptrPtCldGrpLfgr};           // Constant pointer

  pcl::ModelCoefficients::Ptr tableCoeff{new pcl::ModelCoefficients()};
  pcl::PointIndices::Ptr tableIndices{new pcl::PointIndices()};
  pcl::PointIndices::Ptr objectIndices{new pcl::PointIndices()};

  std::vector<float> lastKinectPose;
  std::vector<float> minUnexp;
  std::vector<float> maxUnexp;

  //Physical properties of the gripper
  double gripperWidth;
  double lowerGripperWidth;

  environment(ros::NodeHandle *nh);

  // Callback function to point cloud subscriber
  void cbPtCld (const ptCldColor::ConstPtr& msg);

  // Callback function to RGB image subscriber
  void cbImgRgb (const sensor_msgs::ImageConstPtr& msg);

  // Callback function to RGB image subscriber
  void cbImgDepth (const sensor_msgs::ImageConstPtr& msg);

  // Load Gripper Hand and Finger file
  void loadGripper();

  // Update gripper based on finger width
  void updateGripper(float width);

  // Function to move the kinect. Args: Array of X,Y,Z,Roll,Pitch,Yaw
  void moveKinect(std::vector<float> pose);

  // Function to read the kinect data.
  void readKinect();

  // Function to Fuse last data with existing data
  void fuseLastData();

  // Extracting the major plane (Table) and object
  void dataExtract();

  // Generating unexplored point cloud
  void genUnexploredPtCld();

  // Updating the unexplored point cloud
  void updateUnexploredPtCld();
};

// A test function to check if the "moveKinect" function is working
void testKinectMovement(environment &av);

// A test function to check if the "readKinect" function is working
void testKinectRead(environment &av,int flag);

// A test function to check fusing of data
void testPtCldFuse(environment &av, int flag);

// A test function to extract table and object data
void testDataExtract(environment &av, int flag);

// A test function to generate unexplored point cloud
void testGenUnexpPtCld(environment &av, int flag);

// A test function to update unexplored point cloud
void testUpdateUnexpPtCld(environment &av, int flag);

// A test function to load and update gripper
void testGripper(environment &av, int flag, float width);

#endif
