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
  float quality{0};
  float gripperWidth{0.05};
  pcl::PointXYZRGB p1;
  pcl::PointXYZRGB p2;
  std::vector<float> pose{0,0,0,0,0,0};    // Note: This is not the final gripper pose
  float addnlPitch{0};
};

bool compareGrasp(graspPoint A, graspPoint B){
  return(A.quality > B.quality);
}

// 6A: Function to move the kinect. Args: Array of X,Y,Z,Roll,Pitch,Yaw
gazebo_msgs::ModelState kinectCartesianModel(std::vector<double> pose){
    //Create Matrix3x3 from Euler Angles
    tf::Matrix3x3 rotMat;
    rotMat.setEulerYPR(pose[5], pose[4], pose[3]);

    // Convert into quaternion
    tf::Quaternion quat;
    rotMat.getRotation(quat);

    // Converting it to the required gazebo format
    gazebo_msgs::ModelState ModelState;
    ModelState.model_name = "Kinect";           // This should be the name of kinect in gazebo
    ModelState.reference_frame = "world";
    ModelState.pose.position.x = pose[0];
    ModelState.pose.position.y = pose[1];
    ModelState.pose.position.z = pose[2];
    ModelState.pose.orientation.x = quat.x();
    ModelState.pose.orientation.y = quat.y();
    ModelState.pose.orientation.z = quat.z();
    ModelState.pose.orientation.w = quat.w();

    return ModelState;
}

// 6B: Funtion to move the Kinect in a viewsphere which has the table cente as its centre
// R (Radius)
// Theta (Polar Angle) -> 0 to 2*PI
// Phi (Azhimuthal angle) -> 0 to PI/2
gazebo_msgs::ModelState kinectViewSphereModel(std::vector<double> pose, std::vector<double> tableCentre){
    //Create Matrix3x3 from Euler Angles
    tf::Matrix3x3 rotMat;
    rotMat.setEulerYPR(M_PI+pose[1], M_PI/2-pose[2], 0);

    // Convert into quaternion
    tf::Quaternion quat;
    rotMat.getRotation(quat);

    // Converting it to the required gazebo format
    gazebo_msgs::ModelState ModelState;
    ModelState.model_name = "Kinect";           // This should be the name of kinect in gazebo
    ModelState.reference_frame = "world";
    ModelState.pose.position.x = tableCentre[0]+pose[0]*sin(pose[2])*cos(pose[1]);
    ModelState.pose.position.y = tableCentre[1]+pose[0]*sin(pose[2])*sin(pose[1]);
    ModelState.pose.position.z = tableCentre[2]+pose[0]*cos(pose[2]);
    ModelState.pose.orientation.x = quat.x();
    ModelState.pose.orientation.y = quat.y();
    ModelState.pose.orientation.z = quat.z();
    ModelState.pose.orientation.w = quat.w();

    return ModelState;
}

//Fuses the Input data with the Output data without modifying either
ptCldColor::Ptr fuseData(std::vector<double> lastKinectPoseCartesian, Eigen::Affine3f tfKinOptGaz, ptCldColor::Ptr ptrPtCldInput, ptCldColor::Ptr ptrPtCldOld, double voxelGridSize){
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

// 12: Finding normals and pairs of grasp points from object point cloud
std::vector<graspPoint> graspsynthesis(ptCldColor::Ptr ptrPtCldObject, std::vector<double> tableCentre, double minGraspQuality, double maxGripperWidth, double voxelGridSize){
    // Generating the normals for the object point cloud
    pcl::search::Search<pcl::PointXYZRGB>::Ptr KdTree{new pcl::search::KdTree<pcl::PointXYZRGB>};
    ptCldColor::ConstPtr cPtrPtCldObject(ptrPtCldObject);
    ptCldNormal::Ptr ptrObjNormal(new ptCldNormal);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cPtrPtCldObject);
    ne.setSearchMethod(KdTree);
    ne.setKSearch(10);
    ne.compute(*ptrObjNormal);

    std::vector<graspPoint> graspsPossible;   // Clear the vector

    graspPoint graspTemp;
    Eigen::Vector3f vectA, vectB;
    double A,B;

    for (int i = 0; i < ptrPtCldObject->size()-1; i++){
      for (int j = i+1; j < ptrPtCldObject->size(); j++){
        graspTemp.p1 = ptrPtCldObject->points[i];
        graspTemp.p2 = ptrPtCldObject->points[j];

        // Vector connecting the two grasp points and its distance
        vectA = graspTemp.p1.getVector3fMap() - graspTemp.p2.getVector3fMap();
        vectB = graspTemp.p2.getVector3fMap() - graspTemp.p1.getVector3fMap();
        graspTemp.gripperWidth = vectA.norm() + voxelGridSize; // Giving a tolerance based on voxel grid size

        // If grasp width is greater than the limit then skip the rest
        if (graspTemp.gripperWidth > maxGripperWidth){
          continue;
        }

        // Using normals to find the angle
        A = std::min(pcl::getAngle3D(vectA,ptrObjNormal->points[i].getNormalVector3fMap()),
                     pcl::getAngle3D(vectA,ptrObjNormal->points[j].getNormalVector3fMap()))*180/M_PI;
        B = std::min(pcl::getAngle3D(vectB,ptrObjNormal->points[i].getNormalVector3fMap()),
                     pcl::getAngle3D(vectB,ptrObjNormal->points[j].getNormalVector3fMap()))*180/M_PI;

        graspTemp.quality = 180 - ( A + B );

        // If grasp quality is less than the min requirement then skip the rest
        if (graspTemp.quality < minGraspQuality){
          continue;
        }

        // Push this into the vector
        graspsPossible.push_back(graspTemp);
      }
    }
    std::sort(graspsPossible.begin(),graspsPossible.end(),compareGrasp);

    // For thin objects grasp pair would not be feasible, so each point is considered as a grasp pair
    // Adding these grasps in the end
    Eigen::Vector3f xyPlaneA(0,0,1);
    Eigen::Vector3f xyPlaneB(0,0,-1);
    for(int i = 0; i < ptrPtCldObject->size(); i++){

      A = std::min(pcl::getAngle3D(xyPlaneA,ptrObjNormal->points[i].getNormalVector3fMap()),
                   pcl::getAngle3D(xyPlaneB,ptrObjNormal->points[i].getNormalVector3fMap()))*180/M_PI;

      // If the point is too close to table and its normal vector is along z axis this skip it
      if (A > 45 && ptrPtCldObject->points[i].z < tableCentre[2]+0.02){
        continue;
      }
      graspTemp.p1 = ptrPtCldObject->points[i];
      // Translating it along the +ve normal vector
      graspTemp.p1.x += (voxelGridSize)/2*ptrObjNormal->points[i].normal_x;
      graspTemp.p1.y += (voxelGridSize)/2*ptrObjNormal->points[i].normal_y;
      graspTemp.p1.z += (voxelGridSize)/2*ptrObjNormal->points[i].normal_z;

      graspTemp.p2 = ptrPtCldObject->points[i];
      // Translating it along the -ve normal vector
      graspTemp.p2.x -= (voxelGridSize)/2*ptrObjNormal->points[i].normal_x;
      graspTemp.p2.y -= (voxelGridSize)/2*ptrObjNormal->points[i].normal_y;
      graspTemp.p2.z -= (voxelGridSize)/2*ptrObjNormal->points[i].normal_z;

      graspTemp.gripperWidth = voxelGridSize;
      graspTemp.quality = 180;
      graspsPossible.push_back(graspTemp);
    }
    return graspsPossible;
}

//13 For determining a specific gripper pose
std::vector<float> genGripperPose(std::vector<graspPoint> graspsPossible, int index){
    Eigen::Vector3f xAxis,yAxis,zAxis;
    Eigen::Vector3f xyPlane(0,0,1);

    yAxis = graspsPossible[index].p1.getVector3fMap() - graspsPossible[index].p2.getVector3fMap(); yAxis.normalize();
    zAxis = yAxis.cross(xyPlane);
    xAxis = yAxis.cross(zAxis);

    tf::Matrix3x3 rotMat;
    double Roll,Pitch,Yaw;
    rotMat.setValue(xAxis[0],yAxis[0],zAxis[0],
                    xAxis[1],yAxis[1],zAxis[1],
                    xAxis[2],yAxis[2],zAxis[2]);
    rotMat.getRPY(Roll,Pitch,Yaw);

    std::vector<float> pose = {0,0,0,0,0,0};
    pose[0] = (graspsPossible[index].p1.x + graspsPossible[index].p2.x)/2;
    pose[1] = (graspsPossible[index].p1.y + graspsPossible[index].p2.y)/2;
    pose[2] = (graspsPossible[index].p1.z + graspsPossible[index].p2.z)/2;
    pose[3] = Roll; pose[4] = Pitch; pose[5] = Yaw;

    return pose;
}