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

//Fuses the Input data with the Output data without modifying either
ptCldColor::Ptr fuseData(std::vector<double> lastKinectPoseCartesian, Eigen::Affine3f tfKinOptGaz, ptCldColor::Ptr ptrPtCldInput, ptCldColor::Ptr ptrPtCldOld, double voxelGridSize = 0.01){
    ptCldColor::Ptr ptrPtCldTemp(new ptCldColor);
    ptCldColor::ConstPtr cPtrPtCldTemp(ptrPtCldTemp);
    ptCldColor::Ptr ptrPtCldOutput(new ptCldColor);
    if(ptrPtCldOld->width > 0){
      ptrPtCldOutput = (*ptrPtCldOld).makeShared();
    }
    // Transform : Kinect Gazebo Frame to Gazebo World frame
    Eigen::Affine3f tfGazWorld = pcl::getTransformation(lastKinectPoseCartesian[0],lastKinectPoseCartesian[1],lastKinectPoseCartesian[2],\
                                        lastKinectPoseCartesian[3],lastKinectPoseCartesian[4],lastKinectPoseCartesian[5]);

    // Apply transformation
    Eigen::Affine3f tf = tfGazWorld * tfKinOptGaz;
    pcl::transformPointCloud(*ptrPtCldInput, *ptrPtCldTemp, tf);

    // Downsample using voxel grid
    pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
    voxelGrid.setInputCloud(cPtrPtCldTemp);
    voxelGrid.setLeafSize(voxelGridSize, voxelGridSize, voxelGridSize);
    voxelGrid.filter(*ptrPtCldTemp);

    // Use registration to further align the point pointclouds
    // Skipping this for now as using simulation

    // Fuse the two pointclouds (except for the first time) and downsample again
    ptCldColor::ConstPtr cPtrPtCldOutput(ptrPtCldOutput);
    if (ptrPtCldOutput->width == 0) {
      *ptrPtCldOutput = *ptrPtCldTemp;
    }else{
      *ptrPtCldOutput += *ptrPtCldTemp;
      voxelGrid.setInputCloud(cPtrPtCldOutput);
      voxelGrid.setLeafSize(voxelGridSize, voxelGridSize, voxelGridSize);
      voxelGrid.filter(*ptrPtCldOutput);
    }

    // Using pass through filter to remove ground plane
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cPtrPtCldOutput);
    pass.setFilterFieldName("z"); pass.setFilterLimits(0.2,10);
    pass.filter(*ptrPtCldOutput);

    ptrPtCldTemp->clear();
    return ptrPtCldOutput;
  }