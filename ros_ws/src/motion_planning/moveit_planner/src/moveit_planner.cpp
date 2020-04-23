#include "moveit_planner.hpp"

#include "moveit_msgs/RobotTrajectory.h"

#include "geometry_msgs/PoseStamped.h"

namespace moveit_planner {  
  // Constructor/Destructor
  MoveitPlanner::MoveitPlanner(ros::NodeHandle& nh, const std::string& arm)
    : n{nh}, spinner{3}, armGroup{arm}, moveGroup{armGroup}, curScaling{1.0},
      eef_step{0.005}, jump_threshold{0} {
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
    auto errCode = moveGroup.plan(curPlan);
    bool success = (errCode == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success)
      ROS_WARN_STREAM("ERROR MOVING TO POSE " << errCode);
    if(success && exe)
      moveGroup.execute(curPlan);

    return success;
  }
  bool MoveitPlanner::moveToJointSpace(const std::vector<double>& jointPositions, bool exe) {
    moveGroup.setJointValueTarget(jointPositions);
    auto errCode = moveGroup.plan(curPlan);
    bool success = (errCode == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success)
      ROS_WARN_STREAM("ERROR MOVING TO JOINT " << errCode);
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

    // Change trajectory speed to match set velocity scaling
    if(curScaling < 1) {
      for(int i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
	for(int j = 0; j < trajectory.joint_trajectory.points[i].velocities.size(); ++j)
	  trajectory.joint_trajectory.points[i].velocities[j] *= curScaling;
	for(int j = 0; j < trajectory.joint_trajectory.points[i].accelerations.size(); ++j)
	  trajectory.joint_trajectory.points[i].accelerations[j] *= (curScaling*curScaling);
	trajectory.joint_trajectory.points[i].time_from_start *= 2 - curScaling;
      }
    }

    curPlan.trajectory_ = trajectory;

    if(exe)
      moveGroup.execute(curPlan);

    // TODO: Check how to get success
    return true;
  }
  bool MoveitPlanner::addCollisionObjects(const std::vector<moveit_msgs::CollisionObject>& collObjs) {
    planningSceneInterface.applyCollisionObjects(collObjs);

    return true;
  }

  bool MoveitPlanner::poseClientCallback(moveit_planner::MovePose::Request& req,
					 moveit_planner::MovePose::Response& res) {
    return moveToPose(req.val, req.execute);
  }

  bool MoveitPlanner::rotClientCallback(moveit_planner::MoveQuat::Request& req,
					moveit_planner::MoveQuat::Response& res) {
    geometry_msgs::PoseStamped curPose = moveGroup.getCurrentPose();
    ROS_INFO_STREAM("Current pose: " << curPose);
    geometry_msgs::Pose targetPose = curPose.pose;
    targetPose.orientation = req.val;

    return moveToPose(targetPose, req.execute);
  }

  bool MoveitPlanner::jsClientCallback(moveit_planner::MoveJoint::Request& req,
				       moveit_planner::MoveJoint::Response& res) {
    return moveToJointSpace(req.val, req.execute);
  }

  bool MoveitPlanner::cartesianMoveCallback(moveit_planner::MoveCart::Request& req,
					    moveit_planner::MoveCart::Response& res) {
    return cartesianMove(req.val, req.execute);
  }
  bool MoveitPlanner::distanceAwayCallback(moveit_planner::MoveAway::Request& req,
					   moveit_planner::MoveAway::Response& res) {
    geometry_msgs::Quaternion desiredQuat = req.pose.orientation;

    // Calculate z-axis direction
    geometry_msgs::Point desiredZ;
    desiredZ.x = 2*desiredQuat.x*desiredQuat.z + 2*desiredQuat.w*desiredQuat.y;
    desiredZ.y = 2*desiredQuat.y*desiredQuat.z - 2*desiredQuat.w*desiredQuat.x;
    desiredZ.z = 1 - 2*desiredQuat.x*desiredQuat.x - 2*desiredQuat.y*desiredQuat.y;

    // Calculate new pose
    geometry_msgs::Pose targetPose = req.pose;
    targetPose.position.x = targetPose.position.x - desiredZ.x * req.distance;
    targetPose.position.y = targetPose.position.y - desiredZ.y * req.distance;
    targetPose.position.z = targetPose.position.z - desiredZ.z * req.distance;

    // Setup response
    res.awayPose = targetPose;

    // Send to moveit
    return moveToPose(targetPose, req.execute);
  }
  bool MoveitPlanner::setVelocityCallback(moveit_planner::SetVelocity::Request& req,
					  moveit_planner::SetVelocity::Response& res) {
    if(req.velScaling <= 0 || req.velScaling > 1) // Invalid scaling
      return false;
    
    moveGroup.setMaxVelocityScalingFactor(req.velScaling);
    moveGroup.setMaxAccelerationScalingFactor(req.velScaling);
    curScaling = req.velScaling;
    return true;
  }
  bool MoveitPlanner::addCollisionCallback(moveit_planner::AddCollision::Request& req,
					   moveit_planner::AddCollision::Response& res) {
    // Place in a vector to use general private function
    std::vector<moveit_msgs::CollisionObject> collObjects;
    collObjects.push_back(req.collObject);

    // Return whether it was successfull or not
    return addCollisionObjects(collObjects);
  }

  // Private
  void MoveitPlanner::setupServices() {
    // TOOD: Setup remaining services
    poseClient = n.advertiseService("move_to_pose",
				    &MoveitPlanner::poseClientCallback,
				    this);
    // Not currently working, need to debug joint_states issue
    // rotClient = n.advertiseService("move_to_orientation",
    // 				   &MoveitPlanner::rotClientCallback,
    // 				   this);
    jsClient = n.advertiseService("move_to_joint_space",
				  &MoveitPlanner::jsClientCallback,
				  this);
    cartesianClient = n.advertiseService("cartesian_move",
					 &MoveitPlanner::cartesianMoveCallback,
					 this);
    distanceAwayClient = n.advertiseService("move_away_point",
					    &MoveitPlanner::distanceAwayCallback,
					    this);
    velocityClient = n.advertiseService("set_velocity_scaling",
					&MoveitPlanner::setVelocityCallback,
					this);
    addCollClient = n.advertiseService("add_collision_object",
				       &MoveitPlanner::addCollisionCallback,
				       this);
  }

  void MoveitPlanner::setupMoveit() {
    ros::spinOnce();
    // TODO: Add setup
  }
}
