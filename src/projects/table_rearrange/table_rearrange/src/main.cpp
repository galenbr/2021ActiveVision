#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

// Moveit includes
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

// main file
int main(int argc, char** argv) {
  ros::init(argc, argv, "ref_benchmark_main");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("arm");
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 1;
  target_pose.position.x = 0.3;
  target_pose.position.y = 0.3;
  target_pose.position.z = 0.3;
  move_group.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  move_group.plan(plan);
  move_group.move();
}
