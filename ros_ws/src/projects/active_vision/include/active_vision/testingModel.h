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
};

bool compareGrasp(graspPoint A, graspPoint B);

void rbgVis(ptCldVis::Ptr viewer, ptCldColor::ConstPtr cloud, std::string name,int vp);

void rbgNormalVis(ptCldVis::Ptr viewer, ptCldColor::ConstPtr cloud, ptCldNormal::ConstPtr normal, std::string name,int vp);

Eigen::Affine3f homoMatTranspose(Eigen::Affine3f tf);

// Class to store data of environment and its processing
class environment{
private:
  ros::Rate r;
  ros::Publisher pubKinectPose;
  ros::Subscriber subKinectPtCld;
  ros::Subscriber subKinectRGB;
  ros::Subscriber subKinectDepth;
  ros::ServiceClient gazeboSpawnModel;
  ros::ServiceClient gazeboDeleteModel;

  pcl::PassThrough<pcl::PointXYZRGB> pass;         // Passthrough filter
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;      // VoxelGrid object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;      // Segmentation object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;   // Extracting object
  pcl::ConvexHull<pcl::PointXYZRGB> cvHull;        // Convex hull object
  pcl::CropHull<pcl::PointXYZRGB> cpHull;          // Crop hull object
  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;   // Prism object
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;  // Normal Estimation
  pcl::search::Search<pcl::PointXYZRGB>::Ptr KdTree{new pcl::search::KdTree<pcl::PointXYZRGB>};  // Used in normal estimation

  Eigen::MatrixXf projectionMat;
  Eigen::Affine3f tfKinOptGaz;     // Transform : Kinect Optical Frame to Kinect Gazebo frame
  Eigen::Affine3f tfGazWorld;      // Transform : Kinect Gazebo Frame to Gazebo World frame
  Eigen::Vector4f objCentroid;

  int flag[3];
  int scale;                       // Scale value for unexplored point cloud generation
  float fingerZOffset;             // Z axis offset between gripper hand and finger

  std::string path;                                     // Path the active vision package
  std::vector<std::vector<std::string>> objectDict;     // List of objects which can be spawned

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
  std::vector<double> tableCentre;                  // Co-ordinates of table centre

  double maxGripperWidth;                           // Max gripper width
  double minGripperWidth;                           // Min gripper width
  double minGraspQuality;
  std::vector<graspPoint> graspsPossible;           // List of possible grasps

  environment(ros::NodeHandle *nh);

  // 1: Callback function to point cloud subscriber
  void cbPtCld (const ptCldColor::ConstPtr& msg);

  // 2: Callback function to RGB image subscriber
  void cbImgRgb (const sensor_msgs::ImageConstPtr& msg);

  // 3: Callback function to RGB image subscriber
  void cbImgDepth (const sensor_msgs::ImageConstPtr& msg);

  // 4: Spawning objects in gazebo on the table centre for a given RPY
  void spawnObject(int objectID, float R, float P, float Y);

  // 5: Deleting objects in gazebo
  void deleteObject(int objectID);

  // 6: Load Gripper Hand and Finger file
  void loadGripper();

  // 7: Update gripper based on finger width
  void updateGripper(float width,int choice);

  // 8A: Function to move the kinect. Args: Array of X,Y,Z,Roll,Pitch,Yaw
  void moveKinectCartesian(std::vector<double> pose);

  // 8B: Funtion to move the Kinect in a viewsphere which has the table cente as its centre
  void moveKinectViewsphere(std::vector<double> pose);

  // 9: Function to read the kinect data.
  void readKinect();

  // 10: Function to Fuse last data with existing data
  void fuseLastData();

  // 11: Extracting the major plane (Table) and object
  void dataExtract();

  // 12: Generating unexplored point cloud
  void genUnexploredPtCld();

  // 13: Updating the unexplored point cloud
  void updateUnexploredPtCld();

  // 14: Collision check for gripper and unexplored point cloud
  void collisionCheck(float width);

  // 15: Finding pairs of grasp points from object point cloud
  void graspsynthesis();
};

// 1: A test function to check if the "moveKinect" function is working
void testKinectMovement(environment &av);

// 2: A test function to check if the "readKinect" function is working
void testKinectRead(environment &av,int flag);

// 3: A test function to check fusing of data
void testPtCldFuse(environment &av, int flag);

// 4: A test function to extract table and object data
void testDataExtract(environment &av, int flag);

// 5: A test function to generate unexplored point cloud
void testGenUnexpPtCld(environment &av, int flag);

// 6: A test function to update unexplored point cloud
void testUpdateUnexpPtCld(environment &av, int flag);

// 7: A test function to load and update gripper
void testGripper(environment &av, int flag, float width);

// 8: A test function to check the collision check algorithm
void testCollision(environment &av, int flag, float width);

// 9: A test function spawn and delete objects in gazebo
void testSpawnDeleteObj(environment &av);

// 10: A test function to move the kinect in a viewsphere continuously
void testMoveKinectInViewsphere(environment &av);

// 11: Grasp synthesis test function
void testGraspsynthesis(environment &av, int flag);

#endif
