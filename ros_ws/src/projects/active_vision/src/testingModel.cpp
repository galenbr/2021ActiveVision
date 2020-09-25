#include <iostream>
#include <math.h>
#include <stdlib.h>
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

//Gazebo specific includes
#include <gazebo_msgs/SetModelState.h>

// Typedef for convinience
typedef pcl::PointCloud<pcl::PointXYZRGB> ptCldColor;

// Class to store data of environment and its processing
class environment{
private:
  ros::Rate r{10};
  ros::Publisher pubKinectPose;
  ros::Subscriber subKinectPtCld;
  ros::Subscriber subKinectRGB;
  ros::Subscriber subKinectDepth;
  int flag[3] = { };

public:
  ptCldColor ptCldLast;  // Point cloud to store the environment
  cv_bridge::CvImageConstPtr ptrRgbLast;    // RGB image from camera
  cv_bridge::CvImageConstPtr ptrDepthLast;  // Depth map from camera

  environment(ros::NodeHandle *nh){
    pubKinectPose = nh->advertise<gazebo_msgs::ModelState> ("/gazebo/set_model_state", 1);
    subKinectPtCld = nh->subscribe ("/camera/depth/points", 1, &environment::cbPtCld, this);
    subKinectRGB = nh->subscribe ("/camera/color/image_raw", 1, &environment::cbImgRgb, this);
    subKinectDepth = nh->subscribe ("/camera/depth/image_raw", 1, &environment::cbImgDepth, this);
  }

  // Callback function to point cloud subscriber
  void cbPtCld (const ptCldColor::ConstPtr& msg){
    if (flag[0]==1) {
      ptCldLast = *msg;
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

  // Function to move the kinect. Args: Array of X,Y,Z,Yaw,Pitch,Roll
  void moveKinect(float pose[6]){
    //Create Matrix3x3 from Euler Angles
    tf::Matrix3x3 rotMat;
    rotMat.setEulerYPR(pose[3], pose[4], pose[5]);

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
  }

  // Function to read the kinect data.
  void readKinect(){
    flag[0] = 1; flag[1] = 1; flag[2] = 1;
    while (flag[0]==1 || flag[1]==1 || flag[2]==1) {
      ros::spinOnce();
      r.sleep();
    }
  }
};

// A test function to check if the "moveKinect" function is working
void testKinectMovement(environment &av){
  int flag = 0;
  do {
    float pose[6] = { };
    std::cout << "Enter kinect pose data" << std::endl;
    std::cout << "X : ";      std::cin >> pose[0];
    std::cout << "Y : ";      std::cin >> pose[1];
    std::cout << "Z : ";      std::cin >> pose[2];
    std::cout << "Yaw : ";    std::cin >> pose[3];
    std::cout << "Pitch : ";  std::cin >> pose[4];
    std::cout << "Roll : ";   std::cin >> pose[5];

    av.moveKinect(pose);
    ros::spinOnce();
    std::cout << "Kinect moved" << std::endl;
    sleep(1);  // Wait for 1sec
    std::cout << "Do you want to continue (1/0) : "; std::cin >> flag;
  } while(flag == 1);
}

// A test function to check if the "readKinect" function is working
void testKinectRead(environment &av){
  int flag = 0;
  do{
    av.readKinect();
    std::cout << "Width of pointcloud is " << av.ptCldLast.width << std::endl;
    std::cout << "RGB image - Size :" << av.ptrRgbLast->image.size() << " Channels : " << av.ptrRgbLast->image.channels() << std::endl;
    std::cout << "Depth image - Size :" << av.ptrDepthLast->image.size() << " Channels : " << av.ptrDepthLast->image.channels() << std::endl;
    // std::cout << av.ptrRgbLast->image.at<cv::Vec3b>(320,240) << std::endl;
    cv::imshow("RGB Feed", av.ptrRgbLast->image);
    cv::imshow("Depth Feed", av.ptrDepthLast->image);
    std::cout << "Close windows to continue" << std::endl;
    cv::waitKey(0);
    std::cout << "Do you want to continue (1/0) : "; std::cin >> flag;
  } while(flag == 1);
}

int main (int argc, char** argv){

  // Initialize ROS
  ros::init (argc, argv, "Testing_Model");
  ros::NodeHandle nh;

  environment activeVision(&nh);

  testKinectMovement(activeVision);    // Use this to test Kinect movement
  // testKinectRead(activeVision);        // Use this to test data read from Kinect

}
