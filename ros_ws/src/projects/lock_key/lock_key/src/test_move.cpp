#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "moveit_planner/GetPose.h"
#include "moveit_planner/MoveCart.h"
#include "moveit_planner/MovePose.h"
#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

ros::NodeHandle* n_ptr;
int32_t timeout = 1000;

void moveRel(double x, double y, double z, double qw, double qx, double qy, double qz){
	//Waiting for services to be available
	ros::service::waitForService("move_to_pose",timeout);
	ros::ServiceClient getPoseClient = n_ptr->serviceClient<moveit_planner::GetPose>("get_pose");
	ros::ServiceClient moveCartClient = n_ptr->serviceClient<moveit_planner::MoveCart>("cartesian_move");
	moveit_planner::GetPose curPose;
	moveit_planner::MoveCart cart;
	//Get current robot pose
	getPoseClient.call(curPose);
	geometry_msgs::Pose p;
    p=curPose.response.pose;
    //Update position with relative changes (SWITCH Y and Z AXES)
    ROS_INFO("Currently at: %.3f, %.3f, %.3f.",p.position.x,p.position.y,p.position.z);
	//p.position.x += x;
	//p.position.y = p.position.y+y;
    p.position.z = p.position.z+z+0.333; // +0.333 offset from CF1 to CF0
    // p.position.x += x;
	// p.position.y += y;
	// p.position.z += z;
    ROS_INFO("Going to: %.3f, %.3f, %.3f.",p.position.x,p.position.y,p.position.z);
    // p.orientation.x = 1.0;
    // p.orientation.y = -0.5;
    // p.orientation.z = 0.0;
    // p.orientation.w = 0.0;
    cart.request.val.push_back(p);
    cart.request.execute = true;
    moveCartClient.call(cart);
    
    
    //ROS_INFO("Moving to Relative Position");
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "test_move");
    ros::NodeHandle n;
    n_ptr=&n;

    double delta_cmd{0.05};

    moveRel(0.0,0.0,-delta_cmd,0.0,0.0,0.0,0.0);

    //ros::spin();
    return 0;

}