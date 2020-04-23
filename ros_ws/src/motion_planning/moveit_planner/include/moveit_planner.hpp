// Header file
#ifndef MOVEIT_PLANNER_H
#define MOVEIT_PLANNER_H

// std includes
#include <string>
#include <vector>

// ros includes
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

// custom srv includes
#include "moveit_planner/MoveQuat.h"
#include "moveit_planner/MovePose.h"
#include "moveit_planner/MoveCart.h"
#include "moveit_planner/MoveAway.h"
#include "moveit_planner/MoveJoint.h"
#include "moveit_planner/MovePoint.h"
#include "moveit_planner/SetVelocity.h"
#include "moveit_planner/AddCollision.h"

// moveit includes

#include "moveit/robot_state/robot_state.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "moveit/robot_trajectory/robot_trajectory.h"

namespace moveit_planner {

  class MoveitPlanner {
  public:
    MoveitPlanner(ros::NodeHandle&, const std::string& arm);
    ~MoveitPlanner();

    // Movement commands
    // bool moveToOrientation(const geometry_msgs::Quaternion& r, bool exe);
    // bool moveToPosition(const geometry_msgs::Point& p, bool exe);
    bool moveToPose(const geometry_msgs::Pose& p, const bool& exe);
    bool moveToJointSpace(const std::vector<double>& jointPositions, bool exe);
    bool cartesianMove(const std::vector<geometry_msgs::Pose>& p, const bool& exe);
    bool addCollisionObjects(const std::vector<moveit_msgs::CollisionObject>& collObjs);

    // Getters/Setters
    std::string getGroup() {return armGroup;};
  private:
    ros::NodeHandle n;
    ros::AsyncSpinner spinner;
    std::string armGroup;

    // Setup
    void setupServices();
    void setupMoveit();

    // moveit stuff
    moveit::planning_interface::MoveGroupInterface moveGroup;
    moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
    moveit::planning_interface::MoveGroupInterface::Plan curPlan;
    moveit::core::RobotStatePtr curState;
    const robot_state::JointModelGroup* jointModelGroup;
    std::vector<double> curJointPositions;
    double curScaling;

    // Services/Topics
    ros::ServiceServer poseClient;
    ros::ServiceServer rotClient;
    ros::ServiceServer jsClient;
    ros::ServiceServer cartesianClient;
    ros::ServiceServer distanceAwayClient;
    ros::ServiceServer velocityClient;
    ros::ServiceServer addCollClient;

    // Callbacks
    bool poseClientCallback(moveit_planner::MovePose::Request& req,
			    moveit_planner::MovePose::Response& res);
    bool rotClientCallback(moveit_planner::MoveQuat::Request& req,
			   moveit_planner::MoveQuat::Response& res);
    bool jsClientCallback(moveit_planner::MoveJoint::Request& req,
			  moveit_planner::MoveJoint::Response& res);
    bool cartesianMoveCallback(moveit_planner::MoveCart::Request& req,
			       moveit_planner::MoveCart::Response& res);
    bool distanceAwayCallback(moveit_planner::MoveAway::Request& req,
			      moveit_planner::MoveAway::Response& res);
    bool setVelocityCallback(moveit_planner::SetVelocity::Request& req,
			     moveit_planner::SetVelocity::Response& res);
    bool addCollisionCallback(moveit_planner::AddCollision::Request& req,
			      moveit_planner::AddCollision::Response& res);

    // Misc
    bool checkSuccess();
    double eef_step;
    double jump_threshold;
  };
}

#endif
