#include <ros/ros.h>
#include <lock_key/SpiralInsertAction.h> // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/WrenchStamped.h>
#include "moveit_planner/GetPose.h"
#include "moveit_planner/MoveCart.h"
#include "moveit_planner/MovePose.h"
#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

ros::NodeHandle* n_ptr;
vector<double> forces;
vector<double> torques;
int32_t timeout = 1000;

typedef actionlib::SimpleActionServer<lock_key::SpiralInsertAction> Server;

bool maxDownForce(double force){
	return forces[2]<force;
}

bool maxSpiralForces(double Fd){
	return ((sqrt(pow(forces[0],2))>0.9) ||
            (sqrt(pow(torques[1],2))>0.9) ||
            (forces[2]>-Fd));
}

void moveAbs(double x, double y, double z, double qw, double qx, double qy, double qz){
	//Waiting for services to be available
	ros::service::waitForService("move_to_pose",timeout);
    ros::ServiceClient moveCartClient = n_ptr->serviceClient<moveit_planner::MoveCart>("cartesian_move");
    moveit_planner::MoveCart cart;
    geometry_msgs::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.w = qw;
    p.orientation.x = qx;
    p.orientation.y = qy; 
    p.orientation.z = qz;
	cart.request.val.push_back(p);
    cart.request.execute = true;
    moveCartClient.call(cart);
    ROS_INFO("Moving to Absolute Position");
}

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
    //Update position with relative changes
	p.position.x += x;
	p.position.y += y;
	p.position.z += z;
	p.orientation.w += qw;
	p.orientation.x += qx;
	p.orientation.y += qy;
	p.orientation.z += qz;
	cart.request.val.push_back(p);
    cart.request.execute = true;
    moveCartClient.call(cart);
    ROS_INFO("Moving to Relative Position");
}

void moveSpiral(){
	// Implement Algorithm 1 from Paper here.
}

void execute(const lock_key::SpiralInsertGoalConstPtr& goal, Server* as){  // Note: "Action" is not appended  here
    ROS_INFO("Beginning spiral trajectory");

    // vector<double> x=req.x;
    // double delta_z=req.delta_z;
    // double Ft=req.Ft;
    // double Fd=req.Fd;
    // double Fi=req.Fi;
    // double delta_max=req.delta_max;

    moveSpiral();

    as->setSucceeded();
}

void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    forces[0]=msg->wrench.force.x;
    forces[1]=msg->wrench.force.y;
    forces[2]=msg->wrench.force.z;
    torques[0]=msg->wrench.torque.x;
    torques[1]=msg->wrench.torque.y;
    torques[2]=msg->wrench.torque.z;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "spiralInsert");
    ros::NodeHandle n=*n_ptr;
    
    ros::Subscriber wrench_sub = n.subscribe("panda_joint7_wrench", 1000, wrenchCallback);
    Server server(n, "spiral_insert_key", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::spin();
    return 0;
}