#ifndef GRASPSYNTHESIS
#define GRASPSYNTHESIS

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <chrono>
#include <random>
#include <boost/make_shared.hpp>

// OpenCV Specific Includes
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS Includes
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/master.h>
#include <active_vision/toolVisualization.h>
#include <active_vision/graspSRV.h>
#include <active_vision/graspVisSRV.h>
#include <std_srvs/Empty.h>
#include <moveit_planner/Inv.h>

// PCL Includes
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>

// Typedef for convinience
typedef pcl::PointCloud<pcl::PointXYZRGB> ptCldColor;
typedef pcl::PointCloud<pcl::Normal> ptCldNormal;

// Funstion to transpose a homogenous matrix
Eigen::Affine3f homoMatTranspose(const Eigen::Affine3f& tf);

// Get Rotation Part of a Affine3f
Eigen::Vector3f getEuler(const Eigen::Affine3f& tf);

// Get Translational Part of a Affine3f
Eigen::Vector3f getTranslation(const Eigen::Affine3f& tf);

Eigen::Affine3f calcTfFromNormal(pcl::Normal normal, pcl::PointXYZRGB point);
void calcTfFromNormal(pcl::Normal normal, pcl::PointXYZRGB point, Eigen::Matrix3f &rot, Eigen::Vector3f &trans);

// Check if franka can reach the point
bool checkFrankReach(ros::ServiceClient &IKClient, geometry_msgs::Pose &p);

// Structure to store one grasp related data
struct graspPoint{
  float quality;
  float distance;
  float lineDistance;
  float gripperWidth;
  pcl::PointXYZRGB p1;
  pcl::PointXYZRGB p2;
  std::vector<float> pose;    // Note: This is not the final gripper pose
  float addnlPitch;
  graspPoint();
};

// Function to compare grasp point for sorting
bool compareGrasp(graspPoint A, graspPoint B);

class graspSynthesis{
private:
  ptCldColor::Ptr ptrPtCldObject{new ptCldColor};    ptCldColor::ConstPtr cPtrPtCldObject{ptrPtCldObject};
  ptCldNormal::Ptr ptrObjNormal{new ptCldNormal};    ptCldNormal::ConstPtr cPtrObjNormal{ptrObjNormal};
  ptCldColor::Ptr ptrPtCldUnexp{new ptCldColor};     ptCldColor::ConstPtr cPtrPtCldUnexp{ptrPtCldUnexp};
  ptCldColor::Ptr ptrPtCldTemp{new ptCldColor};      ptCldColor::ConstPtr cPtrPtCldTemp{ptrPtCldTemp};
  
  // PtCld: Gripper related
  ptCldColor::Ptr ptrPtCldGrpHnd{new ptCldColor};    ptCldColor::ConstPtr cPtrPtCldGrpHnd{ptrPtCldGrpHnd};
  ptCldColor::Ptr ptrPtCldGrpCam{new ptCldColor};    ptCldColor::ConstPtr cPtrPtCldGrpCam{ptrPtCldGrpCam};
  ptCldColor::Ptr ptrPtCldGrpRfgr{new ptCldColor};   ptCldColor::ConstPtr cPtrPtCldGrpRfgr{ptrPtCldGrpRfgr};
  ptCldColor::Ptr ptrPtCldGrpLfgr{new ptCldColor};   ptCldColor::ConstPtr cPtrPtCldGrpLfgr{ptrPtCldGrpLfgr};
  ptCldColor::Ptr ptrPtCldGripper{new ptCldColor};   ptCldColor::ConstPtr cPtrPtCldGripper{ptrPtCldGripper};

  // PtCld: Points colliding with gripper
  ptCldColor::Ptr ptrPtCldCollided{new ptCldColor};  ptCldColor::ConstPtr cPtrPtCldCollided{ptrPtCldCollided};
  ptCldColor::Ptr ptrPtCldCollCheck{new ptCldColor}; ptCldColor::ConstPtr cPtrPtCldCollCheck{ptrPtCldCollCheck};

  pcl::PassThrough<pcl::PointXYZRGB> pass;                 // Passthrough filter
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;              // VoxelGrid object
  pcl::CropBox<pcl::PointXYZRGB> cpBox;                    // Crop box object
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne; // Normal Estimation

  std::vector<bool> useForGrasp;  // Maps to object point cloud and stores if each point can be used in grasp synthesis or not

  bool graspCurvatureConstraint;
  bool graspSurPatchConstraint;
  
  std::vector<graspPoint> graspsPossible;           // List of possible grasps
  pcl::PointXYZRGB minPtGrp[4], maxPtGrp[4];        // Min and Max x,y,z co-ordinates of the gripper
  pcl::PointXYZRGB minPtCol[6], maxPtCol[6];        // Min and Max x,y,z co-ordinates of the gripper used for collision check
  pcl::PointXYZRGB minPtObj, maxPtObj;              
  pcl::PointXYZRGB centroidObj;

  bool thinObject;
  double voxelGridSize;
  double maxGripperWidth;                           // Gripper max width (Actual is 8 cm)
  double minGraspQuality;                           // Min grasp quality threshold
  int selectedGrasp;                                // Index of the selected grasp
  Eigen::Affine3f tfGripper;                        // Transform : For gripper based on grasp points found
  float fingerZOffset;                              // Z axis offset between gripper hand and finger

  std::string path;
  std::string simulationMode;

  ptCldVis::Ptr viewer = NULL;
  std::vector<int> vp;
  keyboardEvent keyPress;

  ros::ServiceClient IKClient;

public:

  bool debugCollisionCheck = false;

  graspSynthesis(ros::NodeHandle *nh);

  // Load Gripper Hand and Finger file
  void loadGripper();

  // Update gripper
  // 0 -> Visualization
  // 1 -> Axis Collision Check
  // 2 -> Gripper Collision Check
  void updateGripper(int index ,int choice);

  // Estimating the contact patch
  bool isContactPatchOk(long int ptIdx);

  void preprocessing();

  // Given a grasp point pair find the gripper orientation
  void findGripperPose(int index);

  // Check if grasp is viable for arm
  bool checkGraspArm(graspPoint &graspData);

  // Check if a pose is viable for a arm
  bool checkArmPose(Eigen::Matrix4f tfMat, geometry_msgs::Pose &p);

  // Finding normals and pairs of grasp points from object point cloud
  void calcGraspPairs();

  // Collision check for gripper and unexplored point cloud
  void collisionCheck();

  // Grasp synthesis callback
  bool graspCallback(active_vision::graspSRV::Request  &request,
                     active_vision::graspSRV::Response &response);

  // Grasp visualize callback
  bool graspVisCallback(active_vision::graspVisSRV::Request &request,
                        active_vision::graspVisSRV::Response &response);
  
  // Visualize the gripper model
  bool viewGripperCallback(std_srvs::Empty::Request &request,
                           std_srvs::Empty::Response &response);
};

#endif