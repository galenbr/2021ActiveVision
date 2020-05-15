#include <ros/ros.h>
#include <gripper_controls/SetControlType.h>
#include <gripper_controls/SetControlValue.h>
#include <gripper_controls/SetFriction.h>
#include "std_msgs/Float64.h"
#include <gripper_controls/Friction.h>
#include <sensor_msgs/JointState.h>

class LowLvlController{
private:
  // finger type: position --> 3, effort --> 0
  int l_finger_type = 3;
  int r_finger_type = 3;
  // finger position reference
  float l_finger_pos = 0.5;
  float r_finger_pos = 0.5;
  // finger effort reference
  float l_finger_effort = 0;
  float r_finger_effort = 0;
  // finger friction reference
  float l_finger_friction = 1e5;
  float r_finger_friction = 1e5;
  // current effort
  float left_effort = 0;
  float right_effort = 0;
  // current position
  float left_position = 0;
  float right_position = 0;

public:
  LowLvlController(){
    std::cout << "Controller object constructed" << std::endl;
  }
  bool set_controller_type(gripper_controls::SetControlType::Request &req, gripper_controls::SetControlType::Response &res){
    l_finger_type = req.left;
    r_finger_type = req.right;
    // set references to current values
    l_finger_effort = left_effort;
    r_finger_effort = right_effort;
    l_finger_pos = left_position;
    r_finger_pos = right_position;
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
    }
    else{
      r_finger_pos = req.value;
    }
    res.success = 1;
    return 1;
  }
  bool friction_setter(gripper_controls::SetFriction::Request &req, gripper_controls::SetFriction::Response &res){
    ros::NodeHandle n;
    float high_friction{0};
    float low_friction{0};
    n.getParam("/low_level/high_friction", high_friction);
    n.getParam("/low_level/low_friction", low_friction);
    // set predefined friction value depending on request --> high or low
    if (req.finger==0){
      if (req.high_friction)
        l_finger_friction = high_friction;
      else
        l_finger_friction = low_friction;
    }
    else{
      if (req.high_friction)
        r_finger_friction = high_friction;
      else
        r_finger_friction = low_friction;
    }
    ROS_INFO("Left %f",l_finger_friction);
    ROS_INFO("Right %f",r_finger_friction);
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
  void jsCallback(const sensor_msgs::JointState& msg){
    // get current effort and position
    left_effort = msg.effort[1];
    right_effort = msg.effort[0];
    left_position = msg.position[1];
    right_position = msg.position[0];
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "low_level_action_server");
  ros::NodeHandle n;
  LowLvlController lowctrl;

  // to be able to run both arm+gripper and gripper
  std::string topic_js;
  if(argc == 1) {
    ROS_INFO("No arm argument given, defaulting to gripper only");
    topic_js = "joint_states";
  }
  else if(argc == 2) {
    ROS_INFO_STREAM("Starting controller with namespace " << argv[1]);
    std::string ns;
    ns = argv[1];
    topic_js = ns + "/joint_states";
  }

  ros::ServiceServer service_type = n.advertiseService("type_change", &LowLvlController::set_controller_type, &lowctrl);
  ros::ServiceServer service_effort = n.advertiseService("effort_change", &LowLvlController::set_effort, &lowctrl);
  ros::ServiceServer service_pos = n.advertiseService("pos_change", &LowLvlController::set_pos, &lowctrl);
  ros::ServiceServer service_set_friction = n.advertiseService("set_friction", &LowLvlController::friction_setter, &lowctrl);
  ros::Subscriber js_sub = n.subscribe(topic_js,1000,&LowLvlController::jsCallback, &lowctrl);
  ros::Publisher lp = n.advertise<std_msgs::Float64>("/vf_hand/l_finger_position/command", 1000);
  ros::Publisher rp = n.advertise<std_msgs::Float64>("/vf_hand/r_finger_position/command", 1000);
  ros::Publisher le = n.advertise<std_msgs::Float64>("/vf_hand/l_finger_effort/command", 1000);
  ros::Publisher re = n.advertise<std_msgs::Float64>("/vf_hand/r_finger_effort/command", 1000);
  ros::Publisher friction = n.advertise<gripper_controls::Friction>("/franka_vf/friction_setter", 1000);

  ros::Rate loop_rate(10);
  // keep publishing to controller topics within this loop
  // update references depending on the service calls made by the high level controller
  while(ros::ok()){
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
