#ifndef TESTING_MODEL
#define TESTING_MODEL

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>

// OpenCV specific includes
// NOT USED (JUST FOR REFERENCE) (ALSO UPDATE IN CMAKELISTS.txt to use it)
// #include <image_transport/image_transport.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/generate.h>
#include <pcl/common/random.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/normal_3d.h>

//Gazebo specific includes
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>

// Typedef for convinience
typedef pcl::PointCloud<pcl::PointXYZRGB> ptCldColor;
typedef pcl::PointCloud<pcl::Normal> ptCldNormal;
typedef pcl::visualization::PCLVisualizer ptCldVis;

// Structure to store one grasp related data
struct graspPoint{
  float quality;
  float gripperWidth;
  pcl::PointXYZRGB p1;
  pcl::PointXYZRGB p2;
  std::vector<float> pose;    // Note: This is not the final gripper pose
  float addnlPitch;
};

bool compareGrasp(graspPoint A, graspPoint B);

void rbgVis(ptCldVis::Ptr viewer, ptCldColor::ConstPtr cloud, std::string name,int vp);

void rbgNormalVis(ptCldVis::Ptr viewer, ptCldColor::ConstPtr cloud, ptCldNormal::ConstPtr normal, std::string name,int vp);

Eigen::Affine3f homoMatTranspose(Eigen::Affine3f tf);

// Class to store data of environment and its processing
class environment{
private:
  ros::Rate r;                          // ROS sleep rate
  ros::Publisher pubKinectPose;         // Publisher : Kinect pose
  ros::Subscriber subKinectPtCld;       // Subscriber : Kinect pointcloud
  // NOT USED (JUST FOR REFERENCE)
  /*ros::Subscriber subKinectRGB;       // Subscriber : Kinect RGB
  ros::Subscriber subKinectDepth;       // Subscriber : Kinect DepthMap */
  ros::ServiceClient gazeboSpawnModel;  // Service : Spawn Model
  ros::ServiceClient gazeboDeleteModel; // Service : Delete Model

  pcl::PassThrough<pcl::PointXYZRGB> pass;         // Passthrough filter
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;      // VoxelGrid object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;      // Segmentation object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;   // Extracting object
  pcl::ConvexHull<pcl::PointXYZRGB> cvHull;        // Convex hull object
  pcl::CropHull<pcl::PointXYZRGB> cpHull;          // Crop hull object
  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;   // Prism object
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;  // Normal Estimation

  Eigen::MatrixXf projectionMat;   // Camera projection matrix
  Eigen::Affine3f tfKinOptGaz;     // Transform : Kinect Optical Frame to Kinect Gazebo frame
  Eigen::Affine3f tfGazWorld;      // Transform : Kinect Gazebo Frame to Gazebo World frame
  Eigen::Affine3f tfGripper;       // Transform : For gripper based on grasp points found

  int readFlag[3];                 // Flag used to read data from kinect only when needed
  float fingerZOffset;             // Z axis offset between gripper hand and finger

  std::string path;                                     // Path the active vision package
  std::vector<std::vector<std::string>> objectDict;     // List of objects which can be spawned

public:
  // NOT USED (JUST FOR REFERENCE)
  /* cv_bridge::CvImageConstPtr ptrRgbLast{new cv_bridge::CvImage};    // RGB image from camera
  cv_bridge::CvImageConstPtr ptrDepthLast{new cv_bridge::CvImage};  // Depth map from camera */

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

  ptCldColor::Ptr ptrPtCldCollision{new ptCldColor};                // Point cloud to store collision points
  ptCldColor::ConstPtr cPtrPtCldCollision{ptrPtCldCollision};       // Constant pointer

  ptCldNormal::Ptr ptrObjNormal{new ptCldNormal};                   // Point cloud to store object normals
  ptCldNormal::ConstPtr cPtrObjNormal{ptrObjNormal};                // Constant pointer

  pcl::ModelCoefficients::Ptr tableCoeff{new pcl::ModelCoefficients()};
  pcl::PointIndices::Ptr tableIndices{new pcl::PointIndices()};
  pcl::PointIndices::Ptr objectIndices{new pcl::PointIndices()};

  std::vector<pcl::Vertices> hullVertices;
  std::vector<double> lastKinectPoseCartesian;      // Last Kinect pose where it was moved in cartesian co-ordiantes
  std::vector<double> lastKinectPoseViewsphere;     // Last Kinect pose where it was moved in viewsphere co-ordinates
  std::vector<double> minUnexp;                     // Min x,y,z of unexplored pointcloud generated
  std::vector<double> maxUnexp;                     // Max x,y,z of unexplored pointcloud generated
  std::vector<graspPoint> graspsPossible;           // List of possible grasps

  double voxelGridSize;                             // Voxel Grid size
  std::vector<double> tableCentre;                  // Co-ordinates of table centre
  int scale;                                        // Scale value for unexplored point cloud generation
  double maxGripperWidth;                           // Gripper max width (Actual is 8 cm)
  double minGraspQuality;                           // Min grasp quality threshold
  int selectedGrasp;                                // Index of the selected grasp

  environment(ros::NodeHandle *nh);

  // 1A: Callback function to point cloud subscriber
  void cbPtCld (const ptCldColor::ConstPtr& msg);

  // 2: Spawning objects in gazebo on the table centre for a given RPY
  void spawnObject(int objectID, float R, float P, float Y);

  // 3: Deleting objects in gazebo
  void deleteObject(int objectID);

  // 4: Load Gripper Hand and Finger file
  void loadGripper();

  // 5: Update gripper
  void updateGripper(int index ,int choice);

  // 6A: Function to move the kinect. Args: Array of X,Y,Z,Roll,Pitch,Yaw
  void moveKinectCartesian(std::vector<double> pose);

  // 6B: Funtion to move the Kinect in a viewsphere which has the table cente as its centre
  void moveKinectViewsphere(std::vector<double> pose);

  // 7: Function to read the kinect data.
  void readKinect();

  // 8: Function to Fuse last data with existing data
  void fuseLastData();

  // 9: Extracting the major plane (Table) and object
  void dataExtract();

  // 10: Generating unexplored point cloud
  void genUnexploredPtCld();

  // 11: Updating the unexplored point cloud
  void updateUnexploredPtCld();

  // 12: Finding pairs of grasp points from object point cloud
  void graspsynthesis();

  // 13: Given a grasp point pair find the gripper orientation
  void findGripperPose(int index);

  // 14: Collision check for gripper and unexplored point cloud
  void collisionCheck();
};

#endif
