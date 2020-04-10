#include <ros/ros.h>

#include "franka_gripper.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_gazebo_gripper_node");
  ros::NodeHandle nh;

  FrankaGripper franka_gripper(nh);

  return 0;
}
