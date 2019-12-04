#include "moveit_planner.hpp"

namespace moveit_planner {  
  // Constructor/Destructor
  MoveitPlanner::MoveitPlanner(ros::NodeHandle& nh, const std::string& arm)
    : n{nh}, spinner{2}, armGroup{arm}, moveGroup{armGroup} {
      spinner.start();

      setupServices();
      setupMoveit();

      ros::waitForShutdown();
    }
  MoveitPlanner::~MoveitPlanner() {
    delete jointModelGroup;
  }

  bool MoveitPlanner::moveToPose(const geometry_msgs::Pose& p, const bool& exe) {
    moveGroup.setPoseTarget(p);
    bool success = (moveGroup.plan(curPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("CHECKING");
    if(success)
      ROS_INFO("PLAN SUCCESS");
    if(!success)
      ROS_INFO("PLAN FAILED");
    if(success && exe) {
      ROS_INFO("EXECUTING PATH");
      moveGroup.execute(curPlan);
    }

    return success;
  }

  bool MoveitPlanner::moveToJointSpace(const std::vector<double>& jointPositions, bool exe) {
    // First, check if the correct number of joints was given
    if(jointPositions.size() != curJointPositions.size())
      return false;
    
    moveGroup.setJointValueTarget(jointPositions);
    bool success = (moveGroup.plan(curPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success && exe)
      moveGroup.execute(curPlan);

    return success;
  }

  bool MoveitPlanner::poseClientCallback(moveit_planner::MovePose::Request& req,
			  moveit_planner::MovePose::Response& res) {
    return moveToPose(req.val, req.execute);
  }

  bool MoveitPlanner::jsClientCallback(moveit_planner::MoveJoint::Request& req,
			moveit_planner::MoveJoint::Response& res) {
    return moveToJointSpace(req.val, req.execute);
  }

  // Private
  void MoveitPlanner::setupServices() {
    // TOOD: Setup remaining services
    poseClient = n.advertiseService("move_to_pose",
				    &MoveitPlanner::poseClientCallback,
				    this);
    jsClient = n.advertiseService("move_to_joint_space",
				  &MoveitPlanner::jsClientCallback,
				  this);
  }

  void MoveitPlanner::setupMoveit() {
    ros::spinOnce();
    // TODO: Add setup
  }
}
