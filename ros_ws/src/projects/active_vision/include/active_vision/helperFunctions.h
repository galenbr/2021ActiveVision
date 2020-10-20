#ifndef HELPER_FUNCTIONS
#define HELPER_FUNCTIONS

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <chrono>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>

// OpenCV specific includes
// NOT USED (JUST FOR REFERENCE) (Update in CMAKELISTS.txt and PACKAGE.XML to use it)
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
#include <tuple>

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

Eigen::Affine3f homoMatTranspose(Eigen::Affine3f tf);

Eigen::Affine3f transformGazWorld(std::vector<double> lastKinectPoseCartesian);

gazebo_msgs::ModelState kinectCartesianModel(std::vector<double> pose);

gazebo_msgs::ModelState kinectViewSphereModel(std::vector<double> pose, std::vector<double> tableCentre);

void fuseData(Eigen::Affine3f tfGazWorld, Eigen::Affine3f tfKinOptGaz, ptCldColor::Ptr ptrPtCldInput, ptCldColor::Ptr ptrPtCldOld, ptCldColor::Ptr ptrPtCldOutput, double voxelGridSize = 0.01);

void extractObj(ptCldColor::ConstPtr cPtrTotalPtCloud, ptCldColor::Ptr ptrExtracted, ptCldColor::Ptr tableOutput, ptCldColor::Ptr hullOutput, pcl::PointXYZRGB *minPtObj, pcl::PointXYZRGB *maxPtObj, double voxelGridSize = 0.01);

void updateunexploredPtCld(Eigen::Affine3f tfGazWorld, Eigen::Affine3f tfKinOptGaz, Eigen::MatrixXf projectionMat, ptCldColor::Ptr ptrPtCldUnexp, ptCldColor::Ptr ptrPtCldLast, ptCldColor::ConstPtr cPtrPtCldTable, pcl::PointXYZRGB minPtObj, pcl::PointXYZRGB maxPtObj, ptCldColor::Ptr unexpOutput, ptCldColor::Ptr collisionOutput, double voxelGridSizeUnexp = 0.02);

void graspsynthesis(std::vector<graspPoint> &graspsPossible, ptCldColor::Ptr ptrPtCldObject, std::vector<double> tableCentre, double minGraspQuality, double maxGripperWidth, double voxelGridSize = 0.01);

std::vector<float> genGripperPose(std::vector<graspPoint> graspsPossible, int index);

#endif