#include <ros/ros.h>
#include <moveit_planner.hpp>
#include <geometry_msgs/Pose.h>
#include <arm_controls/PoseChange.h>

// ArmController class implementation

class ArmController{
private:
  ros::NodeHandle n_;
  // current pose of the arm
  geometry_msgs::Pose pose;

public:
  // initialize arm to a predefined pose
  bool initialize_pose(arm_controls::PoseChange::Request &req, arm_controls::PoseChange::Response &res){
    ros::service::waitForService("move_to_pose");
    ros::ServiceClient client_pose = n_.serviceClient<moveit_planner::MovePose>("move_to_pose");
    moveit_planner::MovePose srv;
    geometry_msgs::Pose goal;
    n_.getParam("arm_control_params/init_pose/x_pos", goal.position.x);
    n_.getParam("arm_control_params/init_pose/y_pos", goal.position.y);
    n_.getParam("arm_control_params/init_pose/z_pos", goal.position.z);
    n_.getParam("arm_control_params/init_pose/x_orn", goal.orientation.x);
    n_.getParam("arm_control_params/init_pose/y_orn", goal.orientation.y);
    n_.getParam("arm_control_params/init_pose/z_orn", goal.orientation.z);
    n_.getParam("arm_control_params/init_pose/w_orn", goal.orientation.w);
    srv.request.val = goal;
    srv.request.execute = req.execute;
    if (client_pose.call(srv)){
      pose = goal;
      ROS_INFO("Arm initialized");
      return true;
    }
    else{
      ROS_ERROR("Initialization failed");
      return false;
    }
  }

  // go to a grasping pose
  bool grasp_pose(arm_controls::PoseChange::Request &req, arm_controls::PoseChange::Response &res){
    ros::service::waitForService("move_to_pose");
    ros::ServiceClient client_pose = n_.serviceClient<moveit_planner::MovePose>("move_to_pose");
    moveit_planner::MovePose srv;
    geometry_msgs::Pose goal;
    n_.getParam("arm_control_params/grasp_pose/x_pos", goal.position.x);
    n_.getParam("arm_control_params/grasp_pose/y_pos", goal.position.y);
    n_.getParam("arm_control_params/grasp_pose/z_pos", goal.position.z);
    n_.getParam("arm_control_params/grasp_pose/x_orn", goal.orientation.x);
    n_.getParam("arm_control_params/grasp_pose/y_orn", goal.orientation.y);
    n_.getParam("arm_control_params/grasp_pose/z_orn", goal.orientation.z);
    n_.getParam("arm_control_params/grasp_pose/w_orn", goal.orientation.w);
    srv.request.val = goal;
    srv.request.execute = req.execute;
    if (client_pose.call(srv)){
      pose = goal;
      ROS_INFO("Grasp pose reached");
      return true;
    }
    else{
      ROS_ERROR("Grasp pose failed");
      return false;
    }
  }

  // go to a resting pose
  bool rest_pose(arm_controls::PoseChange::Request &req, arm_controls::PoseChange::Response &res){
    ros::service::waitForService("move_to_pose");
    ros::ServiceClient client_pose = n_.serviceClient<moveit_planner::MovePose>("move_to_pose");
    moveit_planner::MovePose srv;
    geometry_msgs::Pose goal;
    n_.getParam("arm_control_params/rest_pose/x_pos", goal.position.x);
    n_.getParam("arm_control_params/rest_pose/y_pos", goal.position.y);
    n_.getParam("arm_control_params/rest_pose/z_pos", goal.position.z);
    n_.getParam("arm_control_params/rest_pose/x_orn", goal.orientation.x);
    n_.getParam("arm_control_params/rest_pose/y_orn", goal.orientation.y);
    n_.getParam("arm_control_params/rest_pose/z_orn", goal.orientation.z);
    n_.getParam("arm_control_params/rest_pose/w_orn", goal.orientation.w);
    srv.request.val = goal;
    srv.request.execute = req.execute;
    if (client_pose.call(srv)){
      pose = goal;
      ROS_INFO("Rest pose reached");
      return true;
    }
    else{
      ROS_ERROR("Rest pose failed");
      return false;
    }
  }

  // move up by a predefined distance
  bool move_up(arm_controls::PoseChange::Request &req, arm_controls::PoseChange::Response &res){
    ros::service::waitForService("move_to_pose");
    ros::ServiceClient client_pose = n_.serviceClient<moveit_planner::MovePose>("move_to_pose");
    moveit_planner::MovePose srv;
    geometry_msgs::Pose goal;
    goal = pose;
    goal.position.z += 0.02;
    srv.request.val = goal;
    srv.request.execute = req.execute;
    if (client_pose.call(srv)){
      pose = goal;
      ROS_INFO("Moved up");
      return true;
    }
    else{
      ROS_ERROR("Moving up failed");
      return false;
    }
  }

  // move down by a predefined distance
  bool move_down(arm_controls::PoseChange::Request &req, arm_controls::PoseChange::Response &res){
    ros::service::waitForService("move_to_pose");
    ros::ServiceClient client_pose = n_.serviceClient<moveit_planner::MovePose>("move_to_pose");
    moveit_planner::MovePose srv;
    geometry_msgs::Pose goal;
    goal = pose;
    goal.position.z -= 0.02;
    srv.request.val = goal;
    srv.request.execute = req.execute;
    if (client_pose.call(srv)){
      pose = goal;
      ROS_INFO("Moved down");
      return true;
    }
    else{
      ROS_ERROR("Moving down failed");
      return false;
    }
  }
};


int main(int argc, char **argv){
  ros::init(argc, argv, "arm_control_server");
  ros::NodeHandle n;
  ArmController armctrl;
  ROS_INFO("ArmController object constructed");

  // advertise services
  ros::ServiceServer pose_init = n.advertiseService("initialize_arm_pose", &ArmController::initialize_pose, &armctrl);
  ros::ServiceServer pose_grasp = n.advertiseService("grasp_pose", &ArmController::grasp_pose, &armctrl);
  ros::ServiceServer pose_rest = n.advertiseService("rest_pose", &ArmController::rest_pose, &armctrl);
  ros::ServiceServer move_up = n.advertiseService("move_up", &ArmController::move_up, &armctrl);
  ros::ServiceServer move_down = n.advertiseService("move_down", &ArmController::move_down, &armctrl);

  ros::spin();
  return 0;
}
