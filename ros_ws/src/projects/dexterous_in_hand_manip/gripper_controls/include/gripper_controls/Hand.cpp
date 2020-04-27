#include "gripper_controls/Hand.h"
#include <ros/ros.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <string>
#include "gripper_controls/SetControlValue.h"
#include <controller_manager_msgs/SwitchController.h>
#include "gripper_controls/SetControlType.h"
#include <gripper_controls/SetFriction.h>
#include <iostream>

float Hand::read_position(int motor_num){
  ros::NodeHandle n;
  ros::service::waitForService("gazebo/get_joint_properties");
  ros::ServiceClient client_read_pos = n.serviceClient<gazebo_msgs::GetJointProperties>("gazebo/get_joint_properties");
  gazebo_msgs::GetJointProperties srv;
  std::string joint = "J" + std::to_string(motor_num);
  srv.request.joint_name = joint;
  if (client_read_pos.call(srv)){
    ROS_INFO("Read_position_success");
    return srv.response.position[0];
  }
  else{
    ROS_INFO("Failure");
    return -1;
  }
}

bool Hand::command_position(int motor_num, float position){
  ros::NodeHandle n;
  ros::service::waitForService("value_change");
  ros::ServiceClient client_position = n.serviceClient<gripper_controls::SetControlValue>("value_change");
  gripper_controls::SetControlValue srv;
  srv.request.finger = motor_num;
  srv.request.value = position;
  if (client_position.call(srv)){
    ROS_INFO("Command_position[%d]_Success=[%f]", motor_num, position);
    return 1;
  }
  else{
    ROS_INFO("Failure");
    return -1;
  }
}

bool Hand::command_torque(int motor_num, float torque){
  ros::NodeHandle n;
  ros::service::waitForService("value_change");
  ros::ServiceClient client_torque = n.serviceClient<gripper_controls::SetControlValue>("value_change");
  gripper_controls::SetControlValue srv;
  srv.request.finger = motor_num;
  srv.request.value = torque;
  if (client_torque.call(srv)){
    ROS_INFO("Command_torque_Success");
    return 1;
  }
  else{
    ROS_INFO("Failure");
    return -1;
  }
}

bool Hand::set_actuator_modes(int size, int mode[]){
  ros::NodeHandle n;
  ros::service::waitForService("gripper/controller_manager/switch_controller");
  ros::ServiceClient client_operating_mode = n.serviceClient<controller_manager_msgs::SwitchController>("gripper/controller_manager/switch_controller");
  // 3 -> position_controller 0 -> effort_controller
  std::string left{"l"};
  std::string right{"r"};
  std::string position{"_finger_position"};
  std::string effort{"_finger_effort"};
  controller_manager_msgs::SwitchController srv_l;
  if (mode[0]==3){
    srv_l.request.start_controllers.push_back(left+position);
    srv_l.request.stop_controllers.push_back(left+effort);
  }
  else{
    srv_l.request.start_controllers.push_back(left+effort);
    srv_l.request.stop_controllers.push_back(left+position);
  }
  srv_l.request.strictness = 2;
  client_operating_mode.call(srv_l);
  controller_manager_msgs::SwitchController srv_r;
  if (mode[1]==3){
    srv_r.request.start_controllers.push_back(right+position);
    srv_r.request.stop_controllers.push_back(right+effort);
  }
  else{
    srv_r.request.start_controllers.push_back(right+effort);
    srv_r.request.stop_controllers.push_back(right+position);
  }
  srv_r.request.strictness = 2;
  client_operating_mode.call(srv_r);
  ROS_INFO("Set_operating_mode_success");
  ros::service::waitForService("type_change");
  ros::ServiceClient client_control_mode = n.serviceClient<gripper_controls::SetControlType>("type_change");
  gripper_controls::SetControlType srv;
  srv.request.left = mode[0];
  srv.request.right = mode[1];
  client_control_mode.call(srv);
  return 1;
}

bool Hand::set_friction_left(bool friction_surface){
  ros::NodeHandle n;
  ros::service::waitForService("set_friction");
  ros::ServiceClient client_friction = n.serviceClient<gripper_controls::SetFriction>("set_friction");
  gripper_controls::SetFriction srv;
  srv.request.finger = 0;
  srv.request.high_friction = friction_surface;
  if (client_friction.call(srv)){
    ROS_INFO("Friction_surface_set_Success");
    return 1;
  }
  else{
    ROS_INFO("Failure");
    return 0;
  }
}

bool Hand::set_friction_right(bool friction_surface){
  ros::NodeHandle n;
  ros::service::waitForService("set_friction");
  ros::ServiceClient client_friction = n.serviceClient<gripper_controls::SetFriction>("set_friction");
  gripper_controls::SetFriction srv;
  srv.request.finger = 1;
  srv.request.high_friction = friction_surface;
  if (client_friction.call(srv)){
    ROS_INFO("Friction_surface_set_Success");
    return 1;
  }
  else{
    ROS_INFO("Failure");
    return 0;
  }
}

Hand::Hand(){
  std::cout << "Hand object constructed" << std::endl;
  finger_state = 0;
}

bool Hand::slide_left_down(gripper_controls::PositionCommand::Request &req, gripper_controls::PositionCommand::Response &res){
  ros::NodeHandle n;
  float torque_val{0};
  n.getParam("/high_level/finger_torque", torque_val);
  bool set_modes, set_friction_l, set_friction_r, send_torque, send_pos;
  // Set modes - Left -> Position(3), Right -> Effort(0)
  int modes[] = {3,0};
  set_modes = set_actuator_modes(2,modes);
  if (finger_state != 1){
    // Set Friction Surfaces - Left -> Low, Right -> High
    set_friction_l = set_friction_left(false);
    set_friction_r = set_friction_right(true);
    finger_state = 1;
  }
  send_torque = command_torque(1, torque_val);
  if (send_torque){
    float initial_position;
    initial_position = read_position(1);
    send_pos = command_position(0, req.data);
    if (send_pos)
      return 1;
    else{
      ROS_ERROR("Sending Position Values Failed");
      return 0;
    }
  }
  else{
    ROS_ERROR("Sending Torque Values Failed");
    return 0;
  }
}

bool Hand::slide_left_up(gripper_controls::PositionCommand::Request &req, gripper_controls::PositionCommand::Response &res){
  ros::NodeHandle n;
  float torque_val{0};
  n.getParam("/high_level/finger_torque", torque_val);
  bool set_modes, set_friction_l, set_friction_r, send_torque, send_pos;
  // Set modes - Left -> Torque(0), Right -> Position(3)
  int modes[] = {0,3};
  set_modes = set_actuator_modes(2,modes);
  if (finger_state != 1){
    // Set Friction Surfaces - Left -> Low, Right -> High
    set_friction_l = set_friction_left(false);
    set_friction_r = set_friction_right(true);
    finger_state = 1;
  }
  send_torque = command_torque(0, torque_val);
  if (send_torque){
    float initial_position;
    initial_position = read_position(2);
    send_pos = command_position(1, req.data);
    if (send_pos)
      return 1;
    else{
      ROS_ERROR("Sending Position Values Failed");
      return 0;
    }
  }
  else{
    ROS_ERROR("Sending Torque Values Failed");
    return 0;
  }
}

bool Hand::slide_right_down(gripper_controls::PositionCommand::Request &req, gripper_controls::PositionCommand::Response &res){
  ros::NodeHandle n;
  float torque_val{0};
  n.getParam("/high_level/finger_torque", torque_val);
  bool set_modes, set_friction_l, set_friction_r, send_torque, send_pos;
  // Set modes - Left -> Effort(0), Right -> Position(3)
  int modes[] = {0,3};
  set_modes = set_actuator_modes(2,modes);
  if (finger_state != 2){
    // Set Friction Surfaces - Left -> High, Right -> Low
    set_friction_l = set_friction_left(true);
    set_friction_r = set_friction_right(false);
    finger_state = 2;
  }
  send_torque = command_torque(0, torque_val);
  if (send_torque){
    float initial_position;
    initial_position = read_position(2);
    send_pos = command_position(1, req.data);
    if (send_pos)
      return 1;
    else{
      ROS_ERROR("Sending Position Values Failed");
      return 0;
    }
  }
  else{
    ROS_ERROR("Sending Torque Values Failed");
    return 0;
  }
}

bool Hand::slide_right_up(gripper_controls::PositionCommand::Request &req, gripper_controls::PositionCommand::Response &res){
  ros::NodeHandle n;
  float torque_val{0};
  n.getParam("/high_level/finger_torque", torque_val);
  bool set_modes, set_friction_l, set_friction_r, send_torque, send_pos;
  // Set modes - Left -> Position(3), Right -> Effort(0)
  int modes[] = {3,0};
  set_modes = set_actuator_modes(2,modes);
  if (finger_state != 2){
    // Set Friction Surfaces - Left -> High, Right -> Low
    set_friction_l = set_friction_left(true);
    set_friction_r = set_friction_right(false);
    finger_state = 2;
  }
  send_torque = command_torque(1, torque_val);
  if (send_torque){
    float initial_position;
    initial_position = read_position(1);
    send_pos = command_position(0, req.data);
    if (send_pos)
      return 1;
    else{
      ROS_ERROR("Sending Position Values Failed");
      return 0;
    }
  }
  else{
    ROS_ERROR("Sending Torque Values Failed");
    return 0;
  }
  return 1;
}

bool Hand::rotate_anticlockwise(gripper_controls::PositionCommand::Request &req, gripper_controls::PositionCommand::Response &res){
  ros::NodeHandle n;
  float torque_val{0};
  n.getParam("/high_level/finger_torque", torque_val);
  bool set_modes, set_friction_l, set_friction_r, send_torque, send_pos;
  // Set modes - Left -> Effort(0), Right -> Position(3)
  int modes[] = {0,3};
  set_modes = set_actuator_modes(2,modes);
  if (finger_state != 3){
    // Set Friction Surfaces - Left -> High, Right -> High
    set_friction_l = set_friction_left(true);
    set_friction_r = set_friction_right(true);
    finger_state = 2;
  }
  send_torque = command_torque(0, torque_val);
  if (send_torque){
    float initial_position;
    initial_position = read_position(2);
    for (float i = initial_position; i >= req.data; i = i - 0.01){
      send_pos = command_position(1,i);
    }
    if (send_pos)
      return 1;
    else{
      ROS_ERROR("Sending Position Values Failed");
      return 0;
    }
  }
  else{
    ROS_ERROR("Sending Torque Values Failed");
    return 0;
  }
}

bool Hand::rotate_clockwise(gripper_controls::PositionCommand::Request &req, gripper_controls::PositionCommand::Response &res){
  ros::NodeHandle n;
  float torque_val{0};
  n.getParam("/high_level/finger_torque", torque_val);
  bool set_modes, set_friction_l, set_friction_r, send_torque, send_pos;
  // Set modes - Left -> Position(3), Right -> Effort(0)
  int modes[] = {3,0};
  set_modes = set_actuator_modes(2,modes);
  if (finger_state != 3){
    // Set Friction Surfaces - Left -> High, Right -> High
    set_friction_l = set_friction_left(true);
    set_friction_r = set_friction_right(true);
    finger_state = 2;
  }
  send_torque = command_torque(1, torque_val);
  if (send_torque){
    float initial_position;
    initial_position = read_position(1);
    for (float i = initial_position; i >= req.data; i = i - 0.01){
      send_pos = command_position(0,i);
    }
    if (send_pos)
      return 1;
    else{
      ROS_ERROR("Sending Position Values Failed");
      return 0;
    }
  }
  else{
    ROS_ERROR("Sending Torque Values Failed");
    return 0;
  }
}

bool Hand::hold_object(gripper_controls::Holdcommand::Request &req, gripper_controls::Holdcommand::Response &res){
  bool set_modes, set_friction_l, set_friction_r, send_pos1, send_pos2;
  // Set modes - Left -> Position(3), Right -> Position(3)
  int modes1[] = {3,3};
  set_modes = set_actuator_modes(2,modes1);
  send_pos1 = command_position(0, req.left);
  send_pos2 = command_position(1, req.right);
  if (finger_state != 3){
    set_friction_l = set_friction_left(true);
    set_friction_r = set_friction_right(true);
    finger_state = 3;
  }

  ros::Duration(1).sleep();

  if (send_pos1 && send_pos2)
    return true;
  else
    return false;
}
