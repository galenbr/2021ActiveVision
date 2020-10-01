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

// Fuction to view a rgb point cloud
pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,int num){
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer "+std::to_string(num)));
  viewer->setBackgroundColor (0.5, 0.5, 0.5);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->setCameraPosition(3,2,4,-1,-1,-1,-1,-1,1);
  return (viewer);
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
  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;  // Prism object
  int flag[3] = {};

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

  pcl::ModelCoefficients::Ptr tableCoeff{new pcl::ModelCoefficients()};
  pcl::PointIndices::Ptr tableIndices{new pcl::PointIndices()};
  pcl::PointIndices::Ptr objectIndices{new pcl::PointIndices()};

  std::vector<float> lastKinectPose;

  environment(ros::NodeHandle *nh){
    pubKinectPose = nh->advertise<gazebo_msgs::ModelState> ("/gazebo/set_model_state", 1);
    subKinectPtCld = nh->subscribe ("/camera/depth/points", 1, &environment::cbPtCld, this);
    subKinectRGB = nh->subscribe ("/camera/color/image_raw", 1, &environment::cbImgRgb, this);
    subKinectDepth = nh->subscribe ("/camera/depth/image_raw", 1, &environment::cbImgDepth, this);
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
    // Transform : Kinect Optical Frame to Kinect Gazebo frame
    Eigen::Affine3f tfKinOptGaz;
    tfKinOptGaz = pcl::getTransformation(0,0,0,-M_PI/2,0,-M_PI/2);

    // Transform : Kinect Gazebo Frame to Gazebo World frame
    Eigen::Affine3f tfGazWorld;
    tfGazWorld = pcl::getTransformation(lastKinectPose[0],lastKinectPose[1],lastKinectPose[2],\
                                        lastKinectPose[3],lastKinectPose[4],lastKinectPose[5]);

    // Apply transformation
    Eigen::Affine3f tf = tfGazWorld * tfKinOptGaz;
    pcl::transformPointCloud (*ptrPtCldLast, *ptrPtCldTemp, tf);

    // Downsample using voxel grid
    voxelGrid.setInputCloud(cPtrPtCldTemp);
    voxelGrid.setLeafSize(0.005, 0.005, 0.005);
    voxelGrid.filter(*ptrPtCldTemp);

    // Use registration to further align the point pointclouds
    // Skipping this for now as using simulation

    // Fuse the two pointclouds (except for the first time) and downsample again
    if (ptrPtCldEnv->width = 0) {
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
};

// A test function to check if the "moveKinect" function is working
void testKinectMovement(environment &av){
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
}

// A test function to check if the "readKinect" function is working
void testKinectRead(environment &av){
  // Setting up the point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer;

  int row; int col;
  std::cout << "Enter pixel value to print data for" << std::endl;
  std::cout << "Row (0-479) : "; std::cin >> row;
  std::cout << "Col (0-639) : "; std::cin >> col;
  av.readKinect();

  std::cout << "Printing values for pixel ( " << row << " , " << col << " )"<< std::endl;
  std::cout << "PCD (XYZRGB) : " << av.ptrPtCldLast->points.at(row*(av.ptrPtCldLast->width)+col) << std::endl;
  std::cout << "Color (BGR) : " << av.ptrRgbLast->image.at<cv::Vec3b>(row,col) << std::endl;
  std::cout << "Depth (Z) : " << av.ptrDepthLast->image.at<float>(row,col) << std::endl;

  viewer = rgbVis(av.cPtrPtCldLast,1);
  std::cout << "Close windows to continue" << std::endl;
  while (!viewer->wasStopped ()){
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds(100000));
  }
  cv::imshow("Color Feed", av.ptrRgbLast->image);
  cv::imshow("Depth Feed", av.ptrDepthLast->image);
  cv::waitKey(0);
}

// A test function to check fusing of data
void testPtCldFuse(environment &av){
  // Setting up the point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer;

  // 4 kinect position to capture and fuse
  std::vector<std::vector<float>> kinectPoses = {{0.25,0,1.75,0,0.55,0},
                                                 {1.5,-1.25,1.75,0,0.55,1.57},
                                                 {2.75,0,1.75,0,0.55,3.14},
                                                 {1.5,1.25,1.75,0,0.55,-1.57}};

  for (int i = 0; i < 4; i++) {
    av.moveKinect(kinectPoses[i]);
    av.readKinect();
    av.fuseLastData();
    viewer = rgbVis(av.cPtrPtCldEnv,i+1);
    std::cout << "After fusing view No. " << i+1 << ". Close viewer to continue" << std::endl;
    while (!viewer->wasStopped ()){
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
}

// A test function to extract table and object data
void testDataExtract(environment &av){
  // Setting up the point cloud visualizer
  pcl::visualization::PCLVisualizer::Ptr viewer;

  std::vector<float> kinectPose = {0.25,0,1.75,0,0.55,0};
  av.moveKinect(kinectPose);
  av.readKinect();
  av.fuseLastData();
  av.dataExtract();

  viewer = rgbVis(av.cPtrPtCldTable,1);
  std::cout << "Showing the table extacted. Close viewer to continue" << std::endl;
  while (!viewer->wasStopped ()){
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds(100000));
  }

  viewer = rgbVis(av.cPtrPtCldObject,1);
  std::cout << "Showing the object extacted. Close viewer to continue" << std::endl;
  while (!viewer->wasStopped ()){
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds(100000));
  }
}

int main (int argc, char** argv){

  // Initialize ROS
  ros::init (argc, argv, "Testing_Model");
  ros::NodeHandle nh;

  environment activeVision(&nh);
  sleep(1); // Delay to ensure all publishers and subscribers are connected

  // testKinectMovement(activeVision);    // Use this to test Kinect movement
  // testKinectRead(activeVision);        // Use this to test data read from Kinect
  testPtCldFuse(activeVision);
  testDataExtract(activeVision);
}

/*
Notes:
-> POint cloud XYZRGB data type : std::vector< pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >
-> 640 elements in rach row of the matrix.
-> Transformation of KinectOpticalFrame wrt KinectGazeboFrame (RPY) - (-90 0 -90)
*/
