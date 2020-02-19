#include "ros/ros.h"

// TEST IMPORTS, MOVE TO CLASS
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_processor_node");
  ros::NodeHandle nh{};

  return 0;
}
