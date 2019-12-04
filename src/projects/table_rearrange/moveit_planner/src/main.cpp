#include "ros/ros.h"

#include <iostream>

// Main class
#include "moveit_planner.hpp"

using moveit_planner::MoveitPlanner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "main_node");
  ros::NodeHandle nh{};

  MoveitPlanner planner(nh, "arm");

  return 0;
}
