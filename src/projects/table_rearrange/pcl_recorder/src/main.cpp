#include <vector>

#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "pcl_recorder/GetPointCloud.h"

sensor_msgs::PointCloud2 curCloud;
std::vector<sensor_msgs::PointCloud2> cloudCluster;
// TODO: Implement collecting a group of point clouds over time

void recvPCLData(const sensor_msgs::PointCloud2ConstPtr& pclPtr) {
  curCloud = *pclPtr;
}

bool reqPointCloud(pcl_recorder::GetPointCloud::Request& req,
		   pcl_recorder::GetPointCloud::Response& res) {
  res.data = curCloud;

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_recorder_node");
  ros::NodeHandle nh{};

  // Setup connections
  ros::Subscriber pclSub =
    nh.subscribe<sensor_msgs::PointCloud2>("/panda_camera/depth/points", 1, recvPCLData);
  ros::ServiceServer pclServer =
    nh.advertiseService("get_point_cloud", reqPointCloud);

  // Spin
  ros::spin();

  return 0;
}
