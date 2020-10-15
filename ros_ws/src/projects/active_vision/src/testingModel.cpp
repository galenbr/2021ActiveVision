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

// Fuction to view a rgb point cloud
void rbgVis(ptCldVis::Ptr viewer, ptCldColor::ConstPtr cloud, std::string name,int vp){
  viewer->setBackgroundColor(0.5,0.5,0.5,vp);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud,rgb,name,vp);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,name,vp);
}

// Fuction to view a rgb point cloud with normals
void rbgNormalVis(ptCldVis::Ptr viewer, ptCldColor::ConstPtr cloud, ptCldNormal::ConstPtr normal, std::string name,int vp){
  viewer->setBackgroundColor(0.5,0.5,0.5,vp);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud,rgb,name,vp);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,name,vp);
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud,normal,5,0.01,name+"_normal",vp);
}

// Funstion to transpose a homogenous matrix
Eigen::Affine3f homoMatTranspose(Eigen::Affine3f tf){
  Eigen::Affine3f tfTranspose;
  tfTranspose.setIdentity();
  tfTranspose.matrix().block<3,3>(0,0) = tf.rotation().transpose();
  tfTranspose.matrix().block<3,1>(0,3) = -1*tf.rotation().transpose()*tf.translation();
  return(tfTranspose);
}

// Class to store data of environment and its processing
class environment{
private:
  ros::Rate r{10};
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

  Eigen::MatrixXf projectionMat;
  Eigen::Affine3f tfKinOptGaz;     // Transform : Kinect Optical Frame to Kinect Gazebo frame
  Eigen::Affine3f tfGazWorld;      // Transform : Kinect Gazebo Frame to Gazebo World frame
  Eigen::Affine3f tfGripper;       // Transform : For gripper based on grasp points found
  Eigen::Vector4f objCentroid;

  int flag[3] = {};
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
  int selectedGrasp;                                // Selected grasp

  environment(ros::NodeHandle *nh){
    pubKinectPose = nh->advertise<gazebo_msgs::ModelState> ("/gazebo/set_model_state", 1);
    subKinectPtCld = nh->subscribe ("/camera/depth/points", 1, &environment::cbPtCld, this);
    subKinectRGB = nh->subscribe ("/camera/color/image_raw", 1, &environment::cbImgRgb, this);
    subKinectDepth = nh->subscribe ("/camera/depth/image_raw", 1, &environment::cbImgDepth, this);
    gazeboSpawnModel = nh->serviceClient< gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
    gazeboDeleteModel = nh->serviceClient< gazebo_msgs::DeleteModel> ("/gazebo/delete_model");

    // Transform : Kinect Optical Frame to Kinect Gazebo frame
    tfKinOptGaz = pcl::getTransformation(0,0,0,-M_PI/2,0,-M_PI/2);

    // Camera projection matrix
    projectionMat.resize(3,4);
    projectionMat << 554.254691191187, 0.0, 320.5, -0.0,
                    0.0, 554.254691191187, 240.5, 0.0,
                    0.0, 0.0, 1.0, 0.0;

    // Scale for generating unexplored point cloud
    scale = 3.0;

    // Gripper finger Z-Axis offset from Gripper hand
    fingerZOffset = 0.0584;

    // Gripper properties
    minGripperWidth = 0.005;
    maxGripperWidth = 0.075;    // Actual is 8 cm
    minGraspQuality = 150;
    selectedGrasp = -1;

    // Path to the active_vision package folder
    path = ros::package::getPath("active_vision");

    // Dictionary of objects to be spawned
    objectDict = {{"drillAV","Cordless Drill"},
                  {"squarePrismAV","Square Prism"},
                  {"rectPrismAV","Rectangular Prism"},
                  {"bowlAV","Bowl"}};

    // Table Centre (X,Y,Z);
    tableCentre = {1.5,0,1};
  }

  // 1: Callback function to point cloud subscriber
  void cbPtCld (const ptCldColor::ConstPtr& msg){
    if (flag[0]==1) {
      *ptrPtCldLast = *msg;
      flag[0] = 0;
    }
  }

  // 2: Callback function to RGB image subscriber
  void cbImgRgb (const sensor_msgs::ImageConstPtr& msg){
    if (flag[1]==1) {
      ptrRgbLast = cv_bridge::toCvShare(msg);
      flag[1] = 0;
    }
  }

  // 3: Callback function to RGB image subscriber
  void cbImgDepth (const sensor_msgs::ImageConstPtr& msg){
    if (flag[2]==1) {
      ptrDepthLast = cv_bridge::toCvShare(msg);
      flag[2] = 0;
    }
  }

  // 4: Spawning objects in gazebo on the table centre for a given RPY
  void spawnObject(int objectID, float R, float P, float Y){
    gazebo_msgs::SpawnModel spawnObj;
    geometry_msgs::Pose pose;

    //Create Matrix3x3 from Euler Angles
    tf::Matrix3x3 m_rot;
    m_rot.setEulerYPR(Y, P, R);

    // Convert into quaternion
    tf::Quaternion quat;
    m_rot.getRotation(quat);

    pose.position.x = tableCentre[0];
    pose.position.y = tableCentre[1];
    pose.position.z = tableCentre[2];
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    spawnObj.request.model_name = objectDict[objectID][1];

    std::ifstream ifs(path+"/models/"+objectDict[objectID][0]+"/model.sdf");
    std::string sdfFile( (std::istreambuf_iterator<char>(ifs)),
                         (std::istreambuf_iterator<char>()));
    spawnObj.request.model_xml = sdfFile;

    spawnObj.request.reference_frame = "world";
    spawnObj.request.initial_pose = pose;

    gazeboSpawnModel.call(spawnObj);
  }

  // 5: Deleting objects in gazebo
  void deleteObject(int objectID){
    gazebo_msgs::DeleteModel deleteObj;
    deleteObj.request.model_name = objectDict[objectID][1];

    gazeboDeleteModel.call(deleteObj);
  }

  // 6: Load Gripper Hand and Finger file
  void loadGripper(){
    std::string pathToHand = path+"/models/gripperAV/hand1.ply";
    std::string pathToFinger = path+"/models/gripperAV/finger1.ply";
    // Gripper Hand
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(pathToHand, *ptrPtCldGrpHnd) == -1){
      PCL_ERROR ("Couldn't read file hand.ply \n");
    }
    // Gripper Left Finger
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(pathToFinger, *ptrPtCldGrpLfgr) == -1){
      PCL_ERROR ("Couldn't read file finger.ply \n");
    }

    // Gripper Right Finger
    pcl::transformPointCloud(*ptrPtCldGrpLfgr, *ptrPtCldGrpRfgr, pcl::getTransformation(0,0,0,0,0,M_PI));

    std::cout << "Ignore the PLY reader error on 'face' and 'rgb'." << std::endl;
  }

  // 7: Update gripper
  // 0 -> Hand + Left finger + Right finger
  // 1 -> Hand only
  // 2 -> Left Finger only
  // 3 -> Right FInger only
  void updateGripper(int index ,int choice){
    if (choice == 0) {
      // Adding the gripper hand
      *ptrPtCldGripper=*ptrPtCldGrpHnd;

      // Translating the left finger and adding
      pcl::transformPointCloud(*ptrPtCldGrpLfgr, *ptrPtCldTemp,
                              pcl::getTransformation(0,graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));
      *ptrPtCldGripper += *ptrPtCldTemp;

      // Translating the right finger and adding
      pcl::transformPointCloud(*ptrPtCldGrpRfgr, *ptrPtCldTemp,
                              pcl::getTransformation(0,-graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));
      *ptrPtCldGripper += *ptrPtCldTemp;
    } else if (choice == 1) {
      *ptrPtCldGripper = *ptrPtCldGrpHnd;
    } else if (choice == 2){
      // Translating the left finger and setting
      pcl::transformPointCloud(*ptrPtCldGrpLfgr, *ptrPtCldTemp,
                               pcl::getTransformation(0,graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));
      *ptrPtCldGripper = *ptrPtCldTemp;
    } else if (choice == 3){
      // Translating the right finger and setting
      pcl::transformPointCloud(*ptrPtCldGrpRfgr, *ptrPtCldTemp,
                               pcl::getTransformation(0,-graspsPossible[index].gripperWidth/2,fingerZOffset,0,0,0));
      *ptrPtCldGripper = *ptrPtCldTemp;
    }

    tfGripper = pcl::getTransformation(graspsPossible[index].pose[0],graspsPossible[index].pose[1],
                                       graspsPossible[index].pose[2],graspsPossible[index].pose[3],
                                       graspsPossible[index].pose[4],graspsPossible[index].pose[5])*
                pcl::getTransformation(0,0,0,0,graspsPossible[index].addnlPitch,0)*
                pcl::getTransformation(0,0,-0.0447-fingerZOffset,0,0,0);
    pcl::transformPointCloud(*ptrPtCldGripper, *ptrPtCldGripper, tfGripper);

    ptrPtCldTemp->clear();
  }

  // 8A: Function to move the kinect. Args: Array of X,Y,Z,Roll,Pitch,Yaw
  void moveKinectCartesian(std::vector<double> pose){
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

    // Publishing it to gazebo
    pubKinectPose.publish(ModelState);
    ros::spinOnce();
    sleep(1);

    // Storing the kinect pose
    lastKinectPoseCartesian = pose;
  }

  // 8B: Funtion to move the Kinect in a viewsphere which has the table cente as its centre
  // R (Radius)
  // Theta (Polar Angle) -> 0 to 2*PI
  // Phi (Azhimuthal angle) -> 0 to PI/2
  void moveKinectViewsphere(std::vector<double> pose){
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

    // Publishing it to gazebo
    pubKinectPose.publish(ModelState);
    ros::spinOnce();
    sleep(1);

    // Storing the kinect pose
    lastKinectPoseViewsphere = pose;
    lastKinectPoseCartesian = {ModelState.pose.position.x,
                               ModelState.pose.position.y,
                               ModelState.pose.position.z,
                               0,M_PI/2-pose[2],M_PI+pose[1]};
  }

  // 9: Function to read the kinect data.
  void readKinect(){
    flag[0] = 1; flag[1] = 1; flag[2] = 1;
    while (flag[0]==1 || flag[1]==1 || flag[2]==1) {
      ros::spinOnce();
      r.sleep();
    }
  }

  // 10: Function to Fuse last data with existing data
  void fuseLastData(){
    ptrPtCldTemp->clear();
    // Transform : Kinect Gazebo Frame to Gazebo World frame
    tfGazWorld = pcl::getTransformation(lastKinectPoseCartesian[0],lastKinectPoseCartesian[1],lastKinectPoseCartesian[2],\
                                        lastKinectPoseCartesian[3],lastKinectPoseCartesian[4],lastKinectPoseCartesian[5]);

    // Apply transformation
    Eigen::Affine3f tf = tfGazWorld * tfKinOptGaz;
    pcl::transformPointCloud(*ptrPtCldLast, *ptrPtCldTemp, tf);

    // Downsample using voxel grid
    voxelGrid.setInputCloud(cPtrPtCldTemp);
    voxelGrid.setLeafSize(0.005, 0.005, 0.005);
    voxelGrid.filter(*ptrPtCldTemp);

    // Use registration to further align the point pointclouds
    // Skipping this for now as using simulation

    // Fuse the two pointclouds (except for the first time) and downsample again
    if (ptrPtCldEnv->width == 0) {
      *ptrPtCldEnv = *ptrPtCldTemp;
    }else{
      *ptrPtCldEnv += *ptrPtCldTemp;
      voxelGrid.setInputCloud(cPtrPtCldEnv);
      voxelGrid.setLeafSize(0.005, 0.005, 0.005);
      voxelGrid.filter(*ptrPtCldEnv);
    }

    // Using pass through filter to remove ground plane
    pass.setInputCloud(cPtrPtCldEnv);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.2,10);
    pass.filter(*ptrPtCldEnv);

    ptrPtCldTemp->clear();
  }

  // 11: Extracting the major plane (Table) and object
  void dataExtract(){
    // Find the major plane and get its coefficients and indices
    seg.setInputCloud(cPtrPtCldEnv);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    seg.segment(*tableIndices,*tableCoeff);

    if (tableIndices->indices.size () == 0){
      std::cerr << "No table found in the environment" << std::endl;
      return;
    }

    // Seperating the table and storing its point
    extract.setInputCloud(cPtrPtCldEnv);
    extract.setIndices(tableIndices);
    extract.setNegative(false);
    extract.filter(*ptrPtCldTable);

    // Using convex hull to get the table boundary which would be like a rectangle
    cvHull.setInputCloud(cPtrPtCldTable);
    cvHull.setDimension(2);
    cvHull.reconstruct(*ptrPtCldHull);

    // Double checking the hull dimensions
    if (cvHull.getDimension() != 2){
      std::cerr << "Convex hull dimension != 2" << std::endl;
      return;
    }

    // Using polygonal prism and hull the extract object above the table
    prism.setInputCloud(cPtrPtCldEnv);
    prism.setInputPlanarHull(cPtrPtCldHull);
    prism.setHeightLimits(-1.5f,-0.01f);         // Z height (min, max) in m
    prism.segment(*objectIndices);

    // Using extract to get the point cloud
    extract.setInputCloud(cPtrPtCldEnv);
    extract.setNegative(false);
    extract.setIndices(objectIndices);
    extract.filter(*ptrPtCldObject);
  }

  // 12: Generating unexplored point cloud
  void genUnexploredPtCld(){
    // Finding the region covered by the object
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D(*ptrPtCldObject, minPt, maxPt);
    if (ptrPtCldUnexp->width != 0){
      std::cerr << "Unexplored point cloud already created. Not creating new one." << std::endl;
      return;
    }

    // Setting the min and max limits based on the object dimension and scale.
    // Note: Z scale is only used on +z axis
    minUnexp.push_back(minPt.x-(scale-1)*(maxPt.x-minPt.x)/2);
    minUnexp.push_back(minPt.y-(scale-1)*(maxPt.y-minPt.y)/2);
    minUnexp.push_back(minPt.z);
    maxUnexp.push_back(maxPt.x+(scale-1)*(maxPt.x-minPt.x)/2);
    maxUnexp.push_back(maxPt.y+(scale-1)*(maxPt.y-minPt.y)/2);
    maxUnexp.push_back(maxPt.z+(scale-1)*(maxPt.z-minPt.z)/2);

    // Considering a a point for every 1 cm and then downsampling it to 3 cm
    float nPts = (maxUnexp[0]-minUnexp[0])*(maxUnexp[1]-minUnexp[1])*(maxUnexp[2]-minUnexp[2])*1000000;
    // std::cout << minUnexp[0] << " " << minUnexp[1] << " " << minUnexp[2] << std::endl;
    // std::cout << maxUnexp[0] << " " << maxUnexp[1] << " " << maxUnexp[2] << std::endl;

    pcl::common::CloudGenerator<pcl::PointXYZRGB, pcl::common::UniformGenerator<float>> generator;
    std::uint32_t seed = static_cast<std::uint32_t> (time (nullptr));
    pcl::common::UniformGenerator<float>::Parameters x_params(minUnexp[0], maxUnexp[0], seed++);
    generator.setParametersForX(x_params);
    pcl::common::UniformGenerator<float>::Parameters y_params(minUnexp[1], maxUnexp[1], seed++);;
    generator.setParametersForY(y_params);
    pcl::common::UniformGenerator<float>::Parameters z_params(minUnexp[2], maxUnexp[2], seed++);;
    generator.setParametersForZ(z_params);
    int result = generator.fill(int(nPts), 1, *ptrPtCldUnexp);
    if (result != 0) {
      std::cerr << "Error creating unexplored point cloud." << std::endl;
      return;
    }

    voxelGrid.setInputCloud(cPtrPtCldUnexp);
    voxelGrid.setLeafSize(0.01, 0.01, 0.01);
    voxelGrid.filter(*ptrPtCldUnexp);
  }

  // 13: Updating the unexplored point cloud
  void updateUnexploredPtCld(){
    // Transforming the point cloud to Kinect frame from world frame
    Eigen::Affine3f tf = tfGazWorld*tfKinOptGaz;
    Eigen::Affine3f tfTranspose = homoMatTranspose(tf);
    pcl::transformPointCloud(*ptrPtCldUnexp, *ptrPtCldTemp, tfTranspose);

    Eigen::Vector4f ptTemp;
    Eigen::Vector3f proj;
    pcl::PointIndices::Ptr occludedIndices(new pcl::PointIndices());
    int projIndex;

    // Looping through all the points and finding occluded ones.
    // Using the camera projection matrix to project 3D point to camera plane
    for (int i = 0; i < ptrPtCldTemp->width; i++){
      ptTemp = ptrPtCldTemp->points[i].getVector4fMap();
      proj = projectionMat*ptTemp;
      proj = proj/proj[2];
      proj[0] = round(proj[0])-1;
      proj[1] = round(proj[1])-1;
      projIndex = proj[1]*(ptrPtCldLast->width)+proj[0];
      // If the z value of unexplored pt is greater than the corresponding
      // projected point in Kinect Raw data then that point is occluded.
      if (ptrPtCldLast->points[projIndex].z <= ptTemp[2]){
        occludedIndices->indices.push_back(i);
      }
    }

    // Only keeping the occluded points
    extract.setInputCloud(cPtrPtCldUnexp);
    extract.setIndices(occludedIndices);
    extract.setNegative(false);
    extract.filter(*ptrPtCldUnexp);

    ptrPtCldTemp->clear();
  }

  // 14: Finding normals and pairs of grasp points from object point cloud
  void graspsynthesis(){
    // Generating the normals for the object point cloud
    pcl::search::Search<pcl::PointXYZRGB>::Ptr KdTree{new pcl::search::KdTree<pcl::PointXYZRGB>};
    ne.setInputCloud(cPtrPtCldObject);
    ne.setSearchMethod(KdTree);
    ne.setKSearch(10);
    ne.compute(*ptrObjNormal);

    graspsPossible.clear();   // Clear the vector
    selectedGrasp = -1;

    graspPoint graspTemp;
    Eigen::Vector3f vectA, vectB;
    double A,B;

    for (int i = 0; i < ptrPtCldObject->size(); i++){
      for (int j = i+1; j < ptrPtCldObject->size(); j++){
        graspTemp.p1 = ptrPtCldObject->points[i];
        graspTemp.p2 = ptrPtCldObject->points[j];

        // Vector connecting the two grasp points and its distance
        vectA = graspTemp.p1.getVector3fMap() - graspTemp.p2.getVector3fMap();
        vectB = graspTemp.p2.getVector3fMap() - graspTemp.p1.getVector3fMap();
        graspTemp.gripperWidth = vectA.norm()+0.005; // Giving a 5mm tolerance

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
  }

  // 15: Given a grasp point pair find the gripper orientation
  void findGripperPose(int index){

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

    graspsPossible[index].pose = pose;
  }

  // 16: Collision check for gripper and unexplored point cloud
  void collisionCheck(){

    cvHull.setInputCloud(cPtrPtCldGripper);
    cvHull.setDimension(3);

    cpHull.setInputCloud(ptrPtCldUnexp);
    cpHull.setHullCloud(ptrPtCldHull);
    cpHull.setDim(3);
    cpHull.setCropOutside(true);

    bool stop = false;
    int nOrientations = 8;
    // Loop through all the possible grasps available
    for(int i = 0; (i < graspsPossible.size()) && (stop == false); i++){
      findGripperPose(i);
      // Check for each orientation
      for(int j = 0; (j < nOrientations) && (stop == false); j++){
        graspsPossible[i].addnlPitch = j*(2*M_PI)/nOrientations;
        // Get concave hull of the gripper fingers and hand in sequence and check
        for(int k = 3; k >= 1; k--){
          ptrPtCldCollision->clear();    // Reset the collision cloud
          updateGripper(i,k);
          cvHull.reconstruct(*ptrPtCldHull,hullVertices);
          cpHull.setHullIndices(hullVertices);
          cpHull.filter(*ptrPtCldCollision);
          // std::cout << i << " Orientation " << j << " " << "Object " << k << " Ptcld size " << ptrPtCldCollision->size() << std::endl;
          // If collision detected then exit this loop and check next orientation
          if(ptrPtCldCollision->size() > 0){
            break;
          }
        }
        // If this doesn't have collision, this grasp is OK. So exit the loop. No more orientation or grasp check required
        if(ptrPtCldCollision->size() == 0){
          selectedGrasp = i;
          // std::cout << "Setting selected grasp to " << selectedGrasp << std::endl;
          stop = true;
        }
      }
    }
  }
};

// 1: A test function spawn and delete objects in gazebo
void testSpawnDeleteObj(environment &av){
  std::cout << "*** In object spawn and delete testing function ***" << std::endl;
  int flag = 0;
  for (int i = 0; i < 4; i++) {
    av.spawnObject(i,0,0,0);
    std::cout << "Object " << i+1 << " spawned. Enter any key to continue. "; std::cin >> flag;
    av.deleteObject(i);
    std::cout << "Object " << i+1 << " deleted." << std::endl;
    sleep(1);
  }
  std::cout << "*** End ***" << std::endl;
}

// 2A: A test function to check if the "moveKinect" functions are working
void testKinectMovement(environment &av){
  std::cout << "*** In kinect movement testing function ***" << std::endl;
  int flag = 0;
  do {
    std::cout << "Enter your choice 1:Cartesian, 2:Viewsphere, 0:Exit : "; std::cin >> flag;
    if (flag == 1) {
      std::vector<double> pose(6);
      std::cout << "Enter kinect pose data" << std::endl;
      std::cout << "X : ";      std::cin >> pose[0];
      std::cout << "Y : ";      std::cin >> pose[1];
      std::cout << "Z : ";      std::cin >> pose[2];
      std::cout << "Roll : ";   std::cin >> pose[3];
      std::cout << "Pitch : ";  std::cin >> pose[4];
      std::cout << "Yaw : ";    std::cin >> pose[5];

      av.moveKinectCartesian(pose);
      std::cout << "Kinect moved" << std::endl;
      sleep(1);  // Wait for 1sec

    } else if(flag == 2){
      std::vector<double> pose(3);
      std::cout << "Enter viewsphere co-ordinates with centre at (" <<
                    av.tableCentre[0] << "," <<
                    av.tableCentre[1] << "," <<
                    av.tableCentre[2] << ")" << std::endl;
      std::cout << "R (Radius) : ";                         std::cin >> pose[0];
      std::cout << "Theta (Polar Angle) (0->2*PI) : ";      std::cin >> pose[1];
      std::cout << "Phi (Azhimuthal Angle) (0->PI/2): ";    std::cin >> pose[2];

      av.moveKinectViewsphere(pose);
      std::cout << "Kinect moved" << std::endl;
      sleep(1);  // Wait for 1sec
    }
  } while(flag != 0);
  std::cout << "*** End ***" << std::endl;
}

// 2B: A test function to move the kinect in a viewsphere continuously
void testMoveKinectInViewsphere(environment &av){
  std::cout << "*** In Kinect move in viewsphere testing function ***" << std::endl;
  for (double i = M_PI/2; i > 0;) {
    for (double j = 0; j < 2*M_PI;) {
      av.moveKinectViewsphere({1.4,j,i});
      j += 2*M_PI/10;
    }
    std::cout << (int)((M_PI/2-i)/(M_PI/2/10)+1)<< " out of 10 completed." << std::endl;
    i -= M_PI/2/10;
  }
  std::cout << "*** End ***" << std::endl;
}

// 3: A test function to check if the "readKinect" function is working
void testKinectRead(environment &av, int flag){
  std::cout << "*** In kinect data read testing function ***" << std::endl;
  av.spawnObject(0,0,0,0);

  int row; int col;
  std::cout << "Enter pixel value to print data for" << std::endl;
  std::cout << "Row (0-479) : "; std::cin >> row;
  std::cout << "Col (0-639) : "; std::cin >> col;
  av.readKinect();

  std::cout << "Printing values for pixel ( " << row << " , " << col << " )"<< std::endl;
  std::cout << "PCD (XYZRGB) : " << av.ptrPtCldLast->points.at(row*(av.ptrPtCldLast->width)+col) << std::endl;
  std::cout << "Color (BGR) : " << av.ptrRgbLast->image.at<cv::Vec3b>(row,col) << std::endl;
  std::cout << "Depth (Z) : " << av.ptrDepthLast->image.at<float>(row,col) << std::endl;

  if (flag==1) {
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp(0);
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(0,0,-1,0,0,1,0,-1,0);

    // Adding the point cloud
    rbgVis(viewer,av.cPtrPtCldLast,"Raw Data",vp);

    std::cout << "Close windows to continue" << std::endl;
    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
    cv::imshow("Color Feed", av.ptrRgbLast->image);
    cv::imshow("Depth Feed", av.ptrDepthLast->image);
    cv::waitKey(0);
  }
  av.deleteObject(0);
  std::cout << "*** End ***" << std::endl;
}

// 4: A test function to check fusing of data
void testPtCldFuse(environment &av, int flag){
  std::cout << "*** In point cloud data fusion testing function ***" << std::endl;
  av.spawnObject(0,0,0,0);

  // Setting up the point cloud visualizer
  ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
  viewer->initCameraParameters();
  int vp[4] = {};
  viewer->createViewPort(0.0,0.5,0.5,1.0,vp[0]);
  viewer->createViewPort(0.5,0.5,1.0,1.0,vp[1]);
  viewer->createViewPort(0.0,0.0,0.5,0.5,vp[2]);
  viewer->createViewPort(0.5,0.0,1.0,0.5,vp[3]);
  viewer->addCoordinateSystem(1.0);
  viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

  // 4 kinect position to capture and fuse
  std::vector<std::vector<double>> kinectPoses = {{1.4,-M_PI,M_PI/3},
                                                  {1.4,-M_PI/2,M_PI/3},
                                                  {1.4,0,M_PI/3},
                                                  {1.4,M_PI/2,M_PI/3}};

  for (int i = 0; i < 4; i++) {
    av.moveKinectViewsphere(kinectPoses[i]);
    av.readKinect();
    av.fuseLastData();
    if (flag == 1){
      rbgVis(viewer,av.cPtrPtCldEnv,"Fuse "+std::to_string(i),vp[i]);
    }
  }
  if (flag == 1){
    std::cout << "Close viewer to continue." << std::endl;
    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  av.deleteObject(0);
  std::cout << "*** End ***" << std::endl;
}

// 5: A test function to extract table and object data
void testDataExtract(environment &av, int flag){
  std::cout << "*** In table and object extraction testing function ***" << std::endl;
  av.spawnObject(0,0,0,0);

  std::vector<double> kinectPose = {1.4,-M_PI,M_PI/3};
  av.moveKinectViewsphere(kinectPose);
  av.readKinect();
  av.fuseLastData();
  av.dataExtract();

  if(flag==1){
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp[2] = {};
    viewer->createViewPort(0.0,0.0,0.5,1.0,vp[0]);
    viewer->createViewPort(0.5,0.0,1.0,1.0,vp[1]);
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

    // ADding the point clouds
    rbgVis(viewer,av.cPtrPtCldTable,"Table",vp[0]);
    rbgVis(viewer,av.cPtrPtCldObject,"Object",vp[1]);
    std::cout << "Showing the table and object extacted. Close viewer to continue" << std::endl;

    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  av.deleteObject(0);
  std::cout << "*** End ***" << std::endl;
}

// 6: A test function to generate unexplored point cloud
void testGenUnexpPtCld(environment &av, int flag){
  std::cout << "*** In unexplored point cloud generation testing function ***" << std::endl;
  av.spawnObject(0,0,0,0);

  std::vector<double> kinectPose = {1.4,-M_PI,M_PI/3};
  av.moveKinectViewsphere(kinectPose);
  av.readKinect();
  av.fuseLastData();
  av.dataExtract();
  av.genUnexploredPtCld();

  if(flag==1){
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp(0);
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(2,1,3,-1,-1,-1,-1,-1,1);

    // Adding the point clouds
    rbgVis(viewer,av.cPtrPtCldObject,"Object",vp);
    rbgVis(viewer,av.cPtrPtCldUnexp,"Unexplored pointcloud",vp);
    std::cout << "Showing the object extacted and unexplored point cloud generated. Close viewer to continue" << std::endl;
    while (!viewer->wasStopped()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  av.deleteObject(0);
  std::cout << "*** End ***" << std::endl;
}

// 7: A test function to update unexplored point cloud
void testUpdateUnexpPtCld(environment &av, int flag){
  std::cout << "*** In unexplored point cloud update testing function ***" << std::endl;
  av.spawnObject(0,0,0,0);

  // Setting up the point cloud visualizer
  ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
  viewer->initCameraParameters();
  int vp[4] = {};
  viewer->createViewPort(0.0,0.5,0.5,1.0,vp[0]);
  viewer->createViewPort(0.5,0.5,1.0,1.0,vp[1]);
  viewer->createViewPort(0.0,0.0,0.5,0.5,vp[2]);
  viewer->createViewPort(0.5,0.0,1.0,0.5,vp[3]);
  viewer->addCoordinateSystem(1.0);
  viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

  // 4 kinect position to capture and fuse
  std::vector<std::vector<double>> kinectPoses = {{1.4,-M_PI,M_PI/3},
                                                  {1.4,-M_PI/2,M_PI/3},
                                                  {1.4,0,M_PI/3},
                                                  {1.4,M_PI/2,M_PI/3}};

  for (int i = 0; i < 4; i++) {
    av.moveKinectViewsphere(kinectPoses[i]);
    av.readKinect();
    av.fuseLastData();
    av.dataExtract();
    if (i==0){
      av.genUnexploredPtCld();
    }
    av.updateUnexploredPtCld();
    if (flag == 1){
      // rbgVis(viewer,av.cPtrPtCldEnv,"Env "+std::to_string(i),vp[i]);
      rbgVis(viewer,av.ptrPtCldUnexp,"Unexp "+std::to_string(i),vp[i]);
    }
  }
  if (flag == 1){
    std::cout << "Close viewer to continue." << std::endl;
    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  av.deleteObject(0);
  std::cout << "*** End ***" << std::endl;
}

// 8: A test function to load and update gripper
void testGripper(environment &av, int flag, float width){
  std::cout << "*** In gripper testing function ***" << std::endl;

  graspPoint graspTemp; // Creating a dummy grasp
  av.graspsPossible.push_back(graspTemp);

  av.loadGripper();
  av.updateGripper(0,0);

  if (flag == 1) {
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp = {};
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    viewer->addCoordinateSystem(0.1);
    viewer->setCameraPosition(0.5,0,0,-1,0,0,0,0,1);

    // Adding the point cloud
    rbgVis(viewer,av.cPtrPtCldGripper,"Gripper",vp);

    std::cout << "Showing the gripper. Close viewer to continue" << std::endl;

    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  std::cout << "*** End ***" << std::endl;
}

// 9: Grasp synthesis test function
void testGraspsynthesis(environment &av, int flag){
  std::cout << "*** In grasp synthesis testing function ***" << std::endl;
  av.spawnObject(0,0,0,0);

  // 4 kinect position
  std::vector<std::vector<double>> kinectPoses = {{1.4,-M_PI,M_PI/3},
                                                  {1.4,-M_PI/2,M_PI/3},
                                                  {1.4,0,M_PI/3},
                                                  {1.4,M_PI/2,M_PI/3}};

  for (int i = 0; i < 4; i++) {
    av.moveKinectViewsphere(kinectPoses[i]);
    av.readKinect();
    av.fuseLastData();
    av.dataExtract();
  }

  std::cout << "Min grasp quality threshold is " << av.minGraspQuality << std::endl;
  av.graspsynthesis();

  std::cout << "No. of grasp pairs found : " << av.graspsPossible.size() << std::endl;
  if (av.graspsPossible.size() > 5){
    std::cout << "Top 5 grasp pairs are : " << std::endl;
    for (int i = 0; i < 5; i++){
      std::cout << i + 1 << " " <<
                   av.graspsPossible[i].p1 << " " <<
                   av.graspsPossible[i].p2 << " " <<
                   av.graspsPossible[i].quality << " " <<
                   av.graspsPossible[i].gripperWidth << std::endl;
    }
  }

  if(flag==1){
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp(0);
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

    // Adding the point clouds
    rbgVis(viewer,av.cPtrPtCldEnv,"Environment",vp);
    if (av.graspsPossible.size() > 3){
      for (int i = 0; i < 3; i++){
        viewer->addSphere<pcl::PointXYZRGB>(av.graspsPossible[i].p1,0.0050,0.0,0.0,(i+1.0)/3.0,"GP_"+std::to_string(i)+"_A",vp);
        viewer->addSphere<pcl::PointXYZRGB>(av.graspsPossible[i].p2,0.0050,0.0,0.0,(i+1.0)/3.0,"GP_"+std::to_string(i)+"_B",vp);
      }
    }
    std::cout << "Showing the object and top 3 grasp pairs. Close viewer to continue" << std::endl;
    while (!viewer->wasStopped()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  av.deleteObject(0);
  std::cout << "*** End ***" << std::endl;
}

// 10A: A test function to check the collision check algorithm with dummy data
void testCollisionDummy(environment &av, bool result, int flag){
  std::cout << "*** In dummy collision testing function ***" << std::endl;

  graspPoint graspTemp; // Creating a dummy grasp
  av.graspsPossible.push_back(graspTemp);

  av.loadGripper();

  // Creating a dummy unexplored point cloud
  av.ptrPtCldUnexp->clear();
  pcl::common::CloudGenerator<pcl::PointXYZRGB, pcl::common::UniformGenerator<float>> generator;
  std::uint32_t seed = static_cast<std::uint32_t> (time (nullptr));
  pcl::common::UniformGenerator<float>::Parameters x_params(0, 0.03, seed++);
  generator.setParametersForX(x_params);
  pcl::common::UniformGenerator<float>::Parameters y_params(-0.15, 0.15, seed++);
  generator.setParametersForY(y_params);
  if(result == true){
    pcl::common::UniformGenerator<float>::Parameters z_params(-0.15, -0.01, seed++);
    generator.setParametersForZ(z_params);
  }else{
    pcl::common::UniformGenerator<float>::Parameters z_params(-0.15, 0.15, seed++);
    generator.setParametersForZ(z_params);
  }

  int dummy = generator.fill(5000, 1, *av.ptrPtCldUnexp);
  // Setting color to blue
  for (int i = 0; i < av.ptrPtCldUnexp->size(); i++) {
    av.ptrPtCldUnexp->points[i].b = 200;
  }

  av.collisionCheck();
  if (av.selectedGrasp == -1) {
    std::cout << "No grasp orientation for the grasp points found. Showing the last tested grasp." << std::endl;
  }

  // Setting color to red
  for (int i = 0; i < av.ptrPtCldCollision->size(); i++) {
    av.ptrPtCldCollision->points[i].b = 0;
    av.ptrPtCldCollision->points[i].r = 200;
  }

  if (flag == 1) {
    // Setting up the point cloud visualizer
    ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp = {};
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    // viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

    rbgVis(viewer,av.ptrPtCldUnexp,"Unexp",vp);
    rbgVis(viewer,av.ptrPtCldCollision,"Collision",vp);
    av.updateGripper(0,0);    // Only for visulization purpose
    rbgVis(viewer,av.ptrPtCldGripper,"Gripper",vp);
    std::cout << "Showing the Gripper(Black), Unexplored(Blue), Collision(Red) points. Close viewer to continue" << std::endl;

    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  std::cout << "*** End ***" << std::endl;
}

// 10B: A test function to check the collision check algorithm with object and grasp points
void testCollision(environment &av, int flag){
  std::cout << "*** In collision testing function ***" << std::endl;
  av.spawnObject(0,0,0,0);
  av.loadGripper();

  // 4 kinect poses
  std::vector<std::vector<double>> kinectPoses = {{1.4,-M_PI,M_PI/3},
                                                  {1.4,-M_PI/2,M_PI/3},
                                                  {1.4,0,M_PI/3},
                                                  {1.4,M_PI/2,M_PI/3}};

  for (int i = 0; i < 4; i++) {
    av.moveKinectViewsphere(kinectPoses[i]);
    av.readKinect();
    av.fuseLastData();
    av.dataExtract();
    if (i==0){
      av.genUnexploredPtCld();
    }
    av.updateUnexploredPtCld();
  }

  std::cout << "Min grasp quality threshold is " << av.minGraspQuality << std::endl;
  av.graspsynthesis();
  std::cout << "Grasp synthesis complete." << std::endl;

  av.collisionCheck();
  if(av.selectedGrasp == -1){
    std::cout << "No grasp orientation for the grasp points found." << std::endl;
  }else{
    if (flag == 1) {
      // Setting up the point cloud visualizer
      ptCldVis::Ptr viewer(new ptCldVis ("PCL Viewer"));
      viewer->initCameraParameters();
      int vp = {};
      viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
      viewer->addCoordinateSystem(1.0);
      viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

      rbgVis(viewer,av.cPtrPtCldEnv,"Environment",vp);
      av.updateGripper(av.selectedGrasp,0);    // Only for visulization purpose
      rbgVis(viewer,av.ptrPtCldGripper,"Gripper",vp);
      std::cout << "Showing the object and selected gripper position. Close viewer to continue" << std::endl;

      while (!viewer->wasStopped ()){
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds(100000));
      }
    }
  }
  av.deleteObject(0);
  std::cout << "*** End ***" << std::endl;
}

int main (int argc, char** argv){

  // Initialize ROS
  ros::init (argc, argv, "Testing_Model");
  ros::NodeHandle nh;

  environment activeVision(&nh);
  sleep(1); // Delay to ensure all publishers and subscribers are connected

  int choice;
  std::cout << "Available choices for test functions : " << std::endl;
  std::cout << "1  : Spawn and delete 4 four objects on the table." << std::endl;
  std::cout << "2  : Load and view the gripper model." << std::endl;
  std::cout << "3  : Move the kinect to a custom position." << std::endl;
  std::cout << "4  : Continuously move the kinect in a viewsphere with centre on the table." << std::endl;
  std::cout << "5  : Read and view the data from kinect." << std::endl;
  std::cout << "6  : Read and fuse the data from 4 different viewpoints." << std::endl;
  std::cout << "7  : Extract the table and object from point cloud." << std::endl;
  std::cout << "8  : Generate the initial unexplored pointcloud based on the object." << std::endl;
  std::cout << "9  : Update the unexplored pointcloud based on 4 different viewpoints." << std::endl;
  std::cout << "10 : Grasp synthesis after fusing 4 viewpoints." << std::endl;
  std::cout << "11 : Selecting a grasp after collision check after grasp synthesis." << std::endl;
  std::cout << "Enter your choice : "; cin >> choice;
  switch(choice){
    case 1:
      testSpawnDeleteObj(activeVision);
      break;
    case 2:
      testGripper(activeVision,1,0.05);
      break;
    case 3:
      testKinectMovement(activeVision);
      break;
    case 4:
      testMoveKinectInViewsphere(activeVision);
      break;
    case 5:
      testKinectRead(activeVision,1);
      break;
    case 6:
      testPtCldFuse(activeVision,1);
      break;
    case 7:
      testDataExtract(activeVision,1);
      break;
    case 8:
      testGenUnexpPtCld(activeVision,1);
      break;
    case 9:
      testUpdateUnexpPtCld(activeVision,1);
      break;
    case 10:
      testGraspsynthesis(activeVision,1);
      break;
    case 11:
      testCollision(activeVision,1);
      break;
    default:
      std::cout << "Invalid choice." << std::endl;
  }
  // testCollisionDummy(activeVision,false,1);
}

/*
Notes:
-> POint cloud XYZRGB data type : std::vector< pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >
-> 640 elements in rach row of the matrix.
-> Transformation of KinectOpticalFrame wrt KinectGazeboFrame (RPY) - (-90 0 -90)
*/
