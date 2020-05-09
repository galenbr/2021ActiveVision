#include <ros/ros.h>
#include <gripper_controls/SetControlType.h>
#include <gripper_controls/SetControlValue.h>
#include <gripper_controls/SetFriction.h>
#include "std_msgs/Float64.h"
#include <gripper_controls/Friction.h>
#include <sensor_msgs/JointState.h>

class LowLvlController{
private:
  int l_finger_type = 3;
  int r_finger_type = 3;
  float l_finger_pos = 0.5;
  float r_finger_pos = 0.5;
  float l_finger_effort = 0;
  float r_finger_effort = 0;
  int l_finger_friction = 1;
  int r_finger_friction = 1;

  float read_position(int finger){
    ros::NodeHandle n;
    sensor_msgs::JointStateConstPtr msg;
    msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/gripper/joint_states",n);
    if (finger==0){
      return msg->position[1];
    }
    else{
      return msg->position[0];
    }

  }

  float read_effort(int finger){
    ros::NodeHandle n;
    sensor_msgs::JointStateConstPtr msg;
    msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/gripper/joint_states",n);
    if (finger==0){
      return msg->effort[1];
    }
    else{
      return msg->effort[0];
    }

  }

public:
  LowLvlController(){
    std::cout << "Controller object constructed" << std::endl;
  }
  bool set_controller_type(gripper_controls::SetControlType::Request &req, gripper_controls::SetControlType::Response &res){
    l_finger_type = req.left;
    r_finger_type = req.right;
    l_finger_effort = read_effort(0);
    r_finger_effort = read_effort(1);
    l_finger_pos = read_position(0);
    r_finger_pos = read_position(1);
    res.success = 1;
    return 1;
  }
  
  bool set_effort(gripper_controls::SetControlValue::Request &req, gripper_controls::SetControlValue::Response &res){
    if(req.finger==0)
      l_finger_effort = req.value;
    else
      r_finger_effort = req.value;
    res.success = 1;
    return 1;
  }
  bool set_pos(gripper_controls::SetControlValue::Request &req, gripper_controls::SetControlValue::Response &res){
    if(req.finger==0){
      l_finger_pos = req.value;
      r_finger_pos = -l_finger_pos;
    }
    else{
      r_finger_pos = req.value;
      l_finger_pos = -r_finger_pos;
    }
    res.success = 1;
    return 1;
  }
  bool friction_setter(gripper_controls::SetFriction::Request &req, gripper_controls::SetFriction::Response &res){
    if (req.finger==0){
      if (req.high_friction)
        l_finger_friction = 1;
      else
        l_finger_friction = 0;
    }
    else{
      if (req.high_friction)
        r_finger_friction = 1;
      else
        r_finger_friction = 0;
    }
    return 1;
  }
  int get_finger_type(int finger){
    if(finger==0)
      return l_finger_type;
    else
      return r_finger_type;
  }
  float get_finger_pos(int finger){
    if(finger==0)
      return l_finger_pos;
    else
      return r_finger_pos;
  }
  float get_finger_effort(int finger){
    if(finger==0)
      return l_finger_effort;
    else
      return r_finger_effort;
  }
  int get_finger_friction(int finger){
    if(finger==0)
      return l_finger_friction;
    else
      return r_finger_friction;
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "low_level_action_server");
  ros::NodeHandle n;
  LowLvlController lowctrl;

  ros::ServiceServer service_type = n.advertiseService("type_change", &LowLvlController::set_controller_type, &lowctrl);
  ros::ServiceServer service_effort = n.advertiseService("effort_change", &LowLvlController::set_effort, &lowctrl);
  ros::ServiceServer service_pos = n.advertiseService("pos_change", &LowLvlController::set_pos, &lowctrl);
  ros::ServiceServer service_set_friction = n.advertiseService("set_friction", &LowLvlController::friction_setter, &lowctrl);
  ros::Publisher lp = n.advertise<std_msgs::Float64>("/gripper/l_finger_position/command", 1000);
  ros::Publisher rp = n.advertise<std_msgs::Float64>("/gripper/r_finger_position/command", 1000);
  ros::Publisher le = n.advertise<std_msgs::Float64>("/gripper/l_finger_effort/command", 1000);
  ros::Publisher re = n.advertise<std_msgs::Float64>("/gripper/r_finger_effort/command", 1000);
  ros::Publisher friction = n.advertise<gripper_controls::Friction>("/gripper/friction_setter", 1000);

  ros::Rate loop_rate(20);
  while(ros::ok()){
    // Controller
    // Left finger
    std_msgs::Float64 input_l;
    if(lowctrl.get_finger_type(0)==3){
      input_l.data = lowctrl.get_finger_pos(0);
      lp.publish(input_l);
    }
    else{
      input_l.data = lowctrl.get_finger_effort(0);
      le.publish(input_l);
    }

    // Right finger
    std_msgs::Float64 input_r;
    if(lowctrl.get_finger_type(1)==3){
      input_r.data = lowctrl.get_finger_pos(1);
      rp.publish(input_r);
    }

    else{
      input_r.data = lowctrl.get_finger_effort(1);
      re.publish(input_r);
    }


    //Friction
    gripper_controls::Friction input_friction;
    input_friction.left_friction = lowctrl.get_finger_friction(0);
    input_friction.right_friction = lowctrl.get_finger_friction(1);
    friction.publish(input_friction);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
