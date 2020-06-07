#include <ros/ros.h>

#include "./ros_pcl_manip.cpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_pcl_manip_node");
  ros::NodeHandle nh;

  ROS_PCL rpcl(nh);

  return 0;
}
