#include "franka_gripper.hpp"

#include <std_msgs/Float64.h>

FrankaGripper::FrankaGripper(ros::NodeHandle& nh) : _nh{nh} {
  _grip_server = nh.advertiseService("gazebo_franka_grip", &FrankaGripper::grip, this);
  _finger1_publisher = nh.advertise<std_msgs::Float64>("/panda/panda_finger1_controller/command", 1);
  _finger2_publisher = nh.advertise<std_msgs::Float64>("/panda/panda_finger2_controller/command", 1);

  ros::spin();
}
FrankaGripper::~FrankaGripper() {}

bool FrankaGripper::grip(franka_gripper_gazebo::GripMsg::Request& req,
			 franka_gripper_gazebo::GripMsg::Response& res) {
  std_msgs::Float64 joint_msg;
  joint_msg.data = req.force;

  // Now output to each joint
  _finger1_publisher.publish(joint_msg);
  _finger2_publisher.publish(joint_msg);

  ros::spinOnce();
  ros::spinOnce();

  ros::Duration(0.5).sleep();	// For now just wait for half a second to make sure they arrived

  return true;
}
