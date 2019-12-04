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
#include "moveit_planner/MoveJoint.h"
#include "moveit_planner/MovePoint.h"

// moveit includes
#include "moveit/robot_state/robot_state.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

namespace moveit_planner {

  class MoveitPlanner {
  public:
    MoveitPlanner(ros::NodeHandle&, const std::string& arm);
    ~MoveitPlanner();

    // Movement commands
    bool moveToOrientation(const geometry_msgs::Quaternion& r, bool exe);
    bool moveToPosition(const geometry_msgs::Point& p, bool exe);
    bool moveToPose(const geometry_msgs::Pose& p, const bool& exe);
    bool moveToJointSpace(const std::vector<double>& jointPositions, bool exe);

    // Getters/Setters
    std::string getGroup() {return armGroup;};
  private:
    ros::NodeHandle n;
    std::string armGroup;

    // Setup
    void setupServices();
    void setupMoveit();

    // moveit stuff
    moveit::planning_interface::MoveGroupInterface moveGroup;
    moveit::planning_interface::MoveGroupInterface::Plan curPlan;
    moveit::core::RobotStatePtr curState;
    const robot_state::JointModelGroup* jointModelGroup;
    std::vector<double> curJointPositions;

    // Services/Topics
    ros::ServiceServer orientationClient;
    ros::ServiceServer positionClient;
    ros::ServiceServer poseClient;
    ros::ServiceServer jsClient;

    // Callbacks
    bool orientationClientCallback(moveit_planner::MoveQuat::Request& req,
				   moveit_planner::MoveQuat::Response& res);
    bool positionClientCallback(moveit_planner::MovePoint::Request& req,
				moveit_planner::MovePoint::Response& res);
    bool poseClientCallback(moveit_planner::MovePose::Request& req,
			    moveit_planner::MovePose::Response& res);
    bool jsClientCallback(moveit_planner::MoveJoint::Request& req,
			  moveit_planner::MoveJoint::Response& res);

    // Misc
    bool checkSuccess();
  };
}

#endif
