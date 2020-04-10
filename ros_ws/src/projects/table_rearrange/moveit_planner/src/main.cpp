#include "ros/ros.h"

#include <iostream>
#include <string>

// Main class
#include "moveit_planner.hpp"

using moveit_planner::MoveitPlanner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "main_node");
  ros::NodeHandle nh{};
  std::string group_name;

  if(argc == 1) {
    ROS_INFO("No arm argument given, defaulting to \"arm\"");
    group_name = "arm";
  }
  else if(argc == 2) {
    ROS_INFO_STREAM("Starting moveit_planner with group name " << argv[1]);
    group_name = argv[1];
  }

  MoveitPlanner planner(nh, group_name);

  return 0;
}
