#include "ros/ros.h"

#include "obtain_pose_gazebo.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "obtain_pose_gazebo_node");
  ros::NodeHandle nh{};

  ObtainPoseGazebo p{nh};

  return 0;
}
