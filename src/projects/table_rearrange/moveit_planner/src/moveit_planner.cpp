#include "moveit_planner.hpp"

#include "moveit_msgs/RobotTrajectory.h"

namespace moveit_planner {  
  // Constructor/Destructor
  MoveitPlanner::MoveitPlanner(ros::NodeHandle& nh, const std::string& arm)
    : n{nh}, spinner{2}, armGroup{arm}, moveGroup{armGroup},
      eef_step{0.01}, jump_threshold{0} {
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
    if(success && exe)
      moveGroup.execute(curPlan);

    return success;
  }
  bool MoveitPlanner::moveToJointSpace(const std::vector<double>& jointPositions, bool exe) {
    ROS_INFO("RECEIVED JOINT SPACE REQUEST");
    // First, check if the correct number of joints was given
    if(jointPositions.size() != 7) {
      ROS_INFO_STREAM("INCORRECT PARAMETERS SENT: " << jointPositions.size() << " VS " << 7);
      return false;
    }
    
    moveGroup.setJointValueTarget(jointPositions);
    bool success = (moveGroup.plan(curPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(success && exe)
      moveGroup.execute(curPlan);

    return success;
  }
  bool MoveitPlanner::cartesianMove(const std::vector<geometry_msgs::Pose>& p, const bool& exe) {
    if(p.size() == 0)
      return true;		// Empty waypoints, nothing to execute

    moveit_msgs::RobotTrajectory trajectory;
    moveit::planning_interface::MoveGroupInterface::Plan curPlan;

    moveGroup.computeCartesianPath(p, eef_step, jump_threshold, trajectory);
    curPlan.trajectory_ = trajectory;

    if(exe)
      moveGroup.execute(curPlan);

    // TODO: Check how to get success
    return true;
  }

  bool MoveitPlanner::poseClientCallback(moveit_planner::MovePose::Request& req,
					 moveit_planner::MovePose::Response& res) {
    return moveToPose(req.val, req.execute);
  }

  bool MoveitPlanner::jsClientCallback(moveit_planner::MoveJoint::Request& req,
				       moveit_planner::MoveJoint::Response& res) {
    return moveToJointSpace(req.val, req.execute);
  }

  bool MoveitPlanner::cartesianMoveCallback(moveit_planner::MoveCart::Request& req,
					    moveit_planner::MoveCart::Response& res) {
    return cartesianMove(req.val, req.execute);
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
    cartesianClient = n.advertiseService("cartesian_move",
					 &MoveitPlanner::cartesianMoveCallback,
					 this);
  }

  void MoveitPlanner::setupMoveit() {
    ros::spinOnce();
    // TODO: Add setup
  }
}
