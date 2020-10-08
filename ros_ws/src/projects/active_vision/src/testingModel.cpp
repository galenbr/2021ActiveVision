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
#include <pcl/filters/crop_hull.h>

//Gazebo specific includes
#include <gazebo_msgs/SetModelState.h>

// Typedef for convinience
typedef pcl::PointCloud<pcl::PointXYZRGB> ptCldColor;

// Fuction to view a rgb point cloud
void rbgPtCldViewer(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::string name,int vp){
  viewer->setBackgroundColor(0.5,0.5,0.5,vp);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud,rgb,name,vp);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,name,vp);
}

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

  pcl::PassThrough<pcl::PointXYZRGB> pass;         // Passthrough filter
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;      // VoxelGrid object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;      // Segmentation object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;   // Extracting object
  pcl::ConvexHull<pcl::PointXYZRGB> cvHull;        // Convex hull object
  pcl::CropHull<pcl::PointXYZRGB> cpHull;          // Crop hull object
  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;  // Prism object

  Eigen::MatrixXf projectionMat;
  Eigen::Affine3f tfKinOptGaz;     // Transform : Kinect Optical Frame to Kinect Gazebo frame
  Eigen::Affine3f tfGazWorld;      // Transform : Kinect Gazebo Frame to Gazebo World frame

  int flag[3] = {};
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

  ptCldColor::Ptr ptrPtCldCollision{new ptCldColor};                // Point cloud to store collision points
  ptCldColor::ConstPtr cPtrPtCldCollision{ptrPtCldCollision};       // Constant pointer

  pcl::ModelCoefficients::Ptr tableCoeff{new pcl::ModelCoefficients()};
  pcl::PointIndices::Ptr tableIndices{new pcl::PointIndices()};
  pcl::PointIndices::Ptr objectIndices{new pcl::PointIndices()};
  std::vector<pcl::Vertices> hullVertices;

  std::vector<float> lastKinectPose;
  std::vector<float> minUnexp;
  std::vector<float> maxUnexp;

  //Physical properties of the gripper
  double gripperWidth = 0.2;
  double lowerGripperWidth = 0.02;

  environment(ros::NodeHandle *nh){
    pubKinectPose = nh->advertise<gazebo_msgs::ModelState> ("/gazebo/set_model_state", 1);
    subKinectPtCld = nh->subscribe ("/camera/depth/points", 1, &environment::cbPtCld, this);
    subKinectRGB = nh->subscribe ("/camera/color/image_raw", 1, &environment::cbImgRgb, this);
    subKinectDepth = nh->subscribe ("/camera/depth/image_raw", 1, &environment::cbImgDepth, this);

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

    // Path to the active_vision package folder
    path = ros::package::getPath("active_vision");
  }

  // Callback function to point cloud subscriber
  void cbPtCld (const ptCldColor::ConstPtr& msg){
    if (flag[0]==1) {
      *ptrPtCldLast = *msg;
      flag[0] = 0;
    }
  }

  // Callback function to RGB image subscriber
  void cbImgRgb (const sensor_msgs::ImageConstPtr& msg){
    if (flag[1]==1) {
      ptrRgbLast = cv_bridge::toCvShare(msg);
      flag[1] = 0;
    }
  }

  // Callback function to RGB image subscriber
  void cbImgDepth (const sensor_msgs::ImageConstPtr& msg){
    if (flag[2]==1) {
      ptrDepthLast = cv_bridge::toCvShare(msg);
      flag[2] = 0;
    }
  }

  // Load Gripper Hand and Finger file
  void loadGripper(){
    std::string pathToHand = path+"/models/gripper/hand1.ply";
    std::string pathToFinger = path+"/models/gripper/finger1.ply";
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

  // Update gripper based on finger width
  // 0 : Hand + Left finger + Right finger
  // 1 : Hand only
  // 2 : Left Finger only
  // 3 : Right FInger only
  void updateGripper(float width,int choice){
    if (choice == 0) {
      // Adding the gripper hand
      *ptrPtCldGripper=*ptrPtCldGrpHnd;

      // Translating the left finger and adding
      pcl::transformPointCloud(*ptrPtCldGrpLfgr, *ptrPtCldTemp, pcl::getTransformation(0,width/2,fingerZOffset,0,0,0));
      *ptrPtCldGripper += *ptrPtCldTemp;

      // Translating the right finger and adding
      pcl::transformPointCloud(*ptrPtCldGrpRfgr, *ptrPtCldTemp, pcl::getTransformation(0,-width/2,fingerZOffset,0,0,0));
      *ptrPtCldGripper += *ptrPtCldTemp;
    } else if (choice == 1) {
      *ptrPtCldGripper=*ptrPtCldGrpHnd;
    } else if (choice == 2){
      // Translating the left finger and setting
      pcl::transformPointCloud(*ptrPtCldGrpLfgr, *ptrPtCldTemp, pcl::getTransformation(0,width/2,fingerZOffset,0,0,0));
      *ptrPtCldGripper = *ptrPtCldTemp;
    } else if (choice == 3){
      // Translating the right finger and setting
      pcl::transformPointCloud(*ptrPtCldGrpRfgr, *ptrPtCldTemp, pcl::getTransformation(0,-width/2,fingerZOffset,0,0,0));
      *ptrPtCldGripper = *ptrPtCldTemp;
    }
  }

  // Function to move the kinect. Args: Array of X,Y,Z,Roll,Pitch,Yaw
  void moveKinect(std::vector<float> pose){
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
    lastKinectPose = pose;
  }

  // Function to read the kinect data.
  void readKinect(){
    flag[0] = 1; flag[1] = 1; flag[2] = 1;
    while (flag[0]==1 || flag[1]==1 || flag[2]==1) {
      ros::spinOnce();
      r.sleep();
    }
  }

  // Function to Fuse last data with existing data
  void fuseLastData(){
    // Transform : Kinect Gazebo Frame to Gazebo World frame
    tfGazWorld = pcl::getTransformation(lastKinectPose[0],lastKinectPose[1],lastKinectPose[2],\
                                        lastKinectPose[3],lastKinectPose[4],lastKinectPose[5]);

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
  }

  // Extracting the major plane (Table) and object
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
    prism.setHeightLimits(-1.5f, -0.01f);         // Z height (min, max) in m
    prism.segment(*objectIndices);

    // Using extract to get the point cloud
    extract.setInputCloud(cPtrPtCldEnv);
    extract.setNegative(false);
    extract.setIndices(objectIndices);
    extract.filter(*ptrPtCldObject);
  }

  // Generating unexplored point cloud
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

  // Updating the unexplored point cloud
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
  }

  // Collision check for gripper and unexplored point cloud
  void collisionCheck(float width){
    ptrPtCldCollision->clear();             // Reset the collision cloud

    cvHull.setInputCloud(cPtrPtCldGripper);
    cvHull.setDimension(3);

    cpHull.setInputCloud(ptrPtCldUnexp);
    cpHull.setHullCloud(ptrPtCldHull);
    cpHull.setDim(3);
    cpHull.setCropOutside(true);

    // Get concave hull of the gripper hand, fingers in sequence and check
    for (int i = 1; i <= 3; i++) {
      updateGripper(width,i);
      cvHull.reconstruct(*ptrPtCldHull,hullVertices);
      cpHull.setHullIndices(hullVertices);
      cpHull.filter(*ptrPtCldTemp);
      *ptrPtCldCollision += *ptrPtCldTemp;
    }
  }
};

// A test function to check if the "moveKinect" function is working
void testKinectMovement(environment &av){
  std::cout << "*** In kinect movement testing function ***" << std::endl;
  int flag = 0;
  do {
    std::vector<float> pose(6);
    std::cout << "Enter kinect pose data" << std::endl;
    std::cout << "X : ";      std::cin >> pose[0];
    std::cout << "Y : ";      std::cin >> pose[1];
    std::cout << "Z : ";      std::cin >> pose[2];
    std::cout << "Roll : ";   std::cin >> pose[3];
    std::cout << "Pitch : ";  std::cin >> pose[4];
    std::cout << "Yaw : ";    std::cin >> pose[5];

    av.moveKinect(pose);
    std::cout << "Kinect moved" << std::endl;
    sleep(1);  // Wait for 1sec
    std::cout << "Do you want to continue (1/0) : "; std::cin >> flag;
  } while(flag == 1);
  std::cout << "*** End ***" << std::endl;
}

// A test function to check if the "readKinect" function is working
void testKinectRead(environment &av, int flag){
  std::cout << "*** In kinect data read testing function ***" << std::endl;

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
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp(0);
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(0,0,-1,0,0,1,0,-1,0);

    // Adding the point cloud
    rbgPtCldViewer(viewer,av.cPtrPtCldLast,"Raw Data",vp);

    std::cout << "Close windows to continue" << std::endl;
    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
    cv::imshow("Color Feed", av.ptrRgbLast->image);
    cv::imshow("Depth Feed", av.ptrDepthLast->image);
    cv::waitKey(0);
  }
  std::cout << "*** End ***" << std::endl;
}

// A test function to check fusing of data
void testPtCldFuse(environment &av, int flag){
  std::cout << "*** In point cloud data fusion testing function ***" << std::endl;
  // Setting up the point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("PCL Viewer"));
  viewer->initCameraParameters();
  int vp[4] = {};
  viewer->createViewPort(0.0,0.5,0.5,1.0,vp[0]);
  viewer->createViewPort(0.5,0.5,1.0,1.0,vp[1]);
  viewer->createViewPort(0.0,0.0,0.5,0.5,vp[2]);
  viewer->createViewPort(0.5,0.0,1.0,0.5,vp[3]);
  viewer->addCoordinateSystem(1.0);
  viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

  // 4 kinect position to capture and fuse
  std::vector<std::vector<float>> kinectPoses = {{0.25,0,1.75,0,0.55,0},
                                                 {1.5,-1.25,1.75,0,0.55,1.57},
                                                 {2.75,0,1.75,0,0.55,3.14},
                                                 {1.5,1.25,1.75,0,0.55,-1.57}};

  for (int i = 0; i < 4; i++) {
    av.moveKinect(kinectPoses[i]);
    av.readKinect();
    av.fuseLastData();
    if (flag == 1){
      rbgPtCldViewer(viewer,av.cPtrPtCldEnv,"Fuse "+std::to_string(i),vp[i]);
    }
  }
  if (flag == 1){
    std::cout << "Close viewer to continue." << std::endl;
    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  std::cout << "*** End ***" << std::endl;
}

// A test function to extract table and object data
void testDataExtract(environment &av, int flag){
  std::cout << "*** In table and object extraction testing function ***" << std::endl;

  std::vector<float> kinectPose = {0.25,0,1.75,0,0.55,0};
  av.moveKinect(kinectPose);
  av.readKinect();
  av.fuseLastData();
  av.dataExtract();

  if(flag==1){
    // Setting up the point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp[2] = {};
    viewer->createViewPort(0.0,0.0,0.5,1.0,vp[0]);
    viewer->createViewPort(0.5,0.0,1.0,1.0,vp[1]);
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

    // ADding the point clouds
    rbgPtCldViewer(viewer,av.cPtrPtCldTable,"Table",vp[0]);
    rbgPtCldViewer(viewer,av.cPtrPtCldObject,"Object",vp[1]);
    std::cout << "Showing the table and object extacted. Close viewer to continue" << std::endl;

    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  std::cout << "*** End ***" << std::endl;
}

// A test function to generate unexplored point cloud
void testGenUnexpPtCld(environment &av, int flag){
  std::cout << "*** In unexplored point cloud generation testing function ***" << std::endl;

  std::vector<float> kinectPose = {0.25,0,1.75,0,0.55,0};
  av.moveKinect(kinectPose);
  av.readKinect();
  av.fuseLastData();
  av.dataExtract();
  av.genUnexploredPtCld();

  if(flag==1){
    // Setting up the point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp(0);
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(2,1,3,-1,-1,-1,-1,-1,1);

    // Adding the point clouds
    rbgPtCldViewer(viewer,av.cPtrPtCldObject,"Object",vp);
    rbgPtCldViewer(viewer,av.cPtrPtCldUnexp,"Unexplored pointcloud",vp);
    std::cout << "Showing the object extacted and unexplored point cloud generated. Close viewer to continue" << std::endl;
    while (!viewer->wasStopped()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  std::cout << "*** End ***" << std::endl;
}

// A test function to update unexplored point cloud
void testUpdateUnexpPtCld(environment &av, int flag){
  std::cout << "*** In unexplored point cloud update testing function ***" << std::endl;

  // Setting up the point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("PCL Viewer"));
  viewer->initCameraParameters();
  int vp[4] = {};
  viewer->createViewPort(0.0,0.5,0.5,1.0,vp[0]);
  viewer->createViewPort(0.5,0.5,1.0,1.0,vp[1]);
  viewer->createViewPort(0.0,0.0,0.5,0.5,vp[2]);
  viewer->createViewPort(0.5,0.0,1.0,0.5,vp[3]);
  viewer->addCoordinateSystem(1.0);
  viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

  // 4 kinect position to capture and fuse
  std::vector<std::vector<float>> kinectPoses = {{0.25,0,1.75,0,0.55,0},
                                                 {1.5,-1.25,1.75,0,0.55,1.57},
                                                 {2.75,0,1.75,0,0.55,3.14},
                                                 {1.5,1.25,1.75,0,0.55,-1.57}};

   for (int i = 0; i < 4; i++) {
     av.moveKinect(kinectPoses[i]);
     av.readKinect();
     av.fuseLastData();
     av.dataExtract();
     if (i==0){
       av.genUnexploredPtCld();
     }
     av.updateUnexploredPtCld();
     if (flag == 1){
       // rbgPtCldViewer(viewer,av.cPtrPtCldEnv,"Env "+std::to_string(i),vp[i]);
       rbgPtCldViewer(viewer,av.ptrPtCldUnexp,"Unexp "+std::to_string(i),vp[i]);
     }
   }
   if (flag == 1){
     std::cout << "Close viewer to continue." << std::endl;
     while (!viewer->wasStopped ()){
       viewer->spinOnce(100);
       boost::this_thread::sleep (boost::posix_time::microseconds(100000));
     }
   }
  std::cout << "*** End ***" << std::endl;
}

// A test function to load and update gripper
void testGripper(environment &av, int flag, float width){
  std::cout << "*** In gripper testing function ***" << std::endl;

  av.loadGripper();
  av.updateGripper(width,0);

  if (flag == 1) {
    // Setting up the point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp = {};
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    viewer->addCoordinateSystem(0.1);
    viewer->setCameraPosition(0.5,0,0,-1,0,0,0,0,1);

    // Adding the point cloud
    rbgPtCldViewer(viewer,av.cPtrPtCldGripper,"Gripper",vp);

    std::cout << "Showing the gripper. Close viewer to continue" << std::endl;

    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  std::cout << "*** End ***" << std::endl;
}

// A test function to check the collision check algorithm
void testCollision(environment &av, int flag, float width){
  std::cout << "*** In collision testing function ***" << std::endl;

  av.loadGripper();

  // Creating a dummy unexplored point cloud
  av.ptrPtCldUnexp->clear();
  pcl::common::CloudGenerator<pcl::PointXYZRGB, pcl::common::UniformGenerator<float>> generator;
  std::uint32_t seed = static_cast<std::uint32_t> (time (nullptr));
  pcl::common::UniformGenerator<float>::Parameters x_params(0, 0.03, seed++);
  generator.setParametersForX(x_params);
  pcl::common::UniformGenerator<float>::Parameters y_params(-0.15, 0.15, seed++);;
  generator.setParametersForY(y_params);
  pcl::common::UniformGenerator<float>::Parameters z_params(-0.15, 0.15, seed++);;
  generator.setParametersForZ(z_params);
  int result = generator.fill(5000, 1, *av.ptrPtCldUnexp);
  // Setting color to blue
  for (int i = 0; i < av.ptrPtCldUnexp->size(); i++) {
    av.ptrPtCldUnexp->points[i].b = 200;
  }

  av.collisionCheck(width);

  // Setting color to red
  for (int i = 0; i < av.ptrPtCldCollision->size(); i++) {
    av.ptrPtCldCollision->points[i].b = 0;
    av.ptrPtCldCollision->points[i].r = 200;
  }

  if (flag == 1) {
    // Setting up the point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("PCL Viewer"));
    viewer->initCameraParameters();
    int vp = {};
    viewer->createViewPort(0.0,0.0,1.0,1.0,vp);
    // viewer->addCoordinateSystem(1.0);
    viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);

    rbgPtCldViewer(viewer,av.ptrPtCldUnexp,"Unexp",vp);
    rbgPtCldViewer(viewer,av.ptrPtCldCollision,"Collision",vp);
    av.updateGripper(width,0);    // Only for visulization purpose
    rbgPtCldViewer(viewer,av.ptrPtCldGripper,"Gripper",vp);
    std::cout << "Showing the Gripper(Black), Unexplored(Blue), Collision(Red) points. Close viewer to continue" << std::endl;

    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
  std::cout << "*** End ***" << std::endl;
}

int main (int argc, char** argv){

  // Initialize ROS
  ros::init (argc, argv, "Testing_Model");
  ros::NodeHandle nh;

  environment activeVision(&nh);
  sleep(1); // Delay to ensure all publishers and subscribers are connected

  // testKinectMovement(activeVision);
  // testKinectRead(activeVision,1);
  // testPtCldFuse(activeVision,1);
  // testDataExtract(activeVision,1);
  // testGenUnexpPtCld(activeVision,1);
  // testUpdateUnexpPtCld(activeVision,1);
  // testGripper(activeVision,1,0.05);
  testCollision(activeVision,1,0.05);
}

/*
Notes:
-> POint cloud XYZRGB data type : std::vector< pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >
-> 640 elements in rach row of the matrix.
-> Transformation of KinectOpticalFrame wrt KinectGazeboFrame (RPY) - (-90 0 -90)
*/
