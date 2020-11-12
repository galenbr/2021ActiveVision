#include <ros/ros.h>
#include <lock_key/SpiralInsertAction.h> // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include "moveit_planner/GetPose.h"
#include "moveit_planner/MoveCart.h"
#include "moveit_planner/MovePose.h"
#include "lock_key/getWrench.h"
#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

ros::NodeHandle* n_ptr;
int32_t timeout = 1000;

typedef actionlib::SimpleActionServer<lock_key::SpiralInsertAction> Server;

bool maxDownForce(double force){
    ros::service::waitForService("getWrench",timeout);
    ros::ServiceClient getWrenchclient = n_ptr->serviceClient<lock_key::getWrench>("getWrench");
    lock_key::getWrench currentWrench;
    getWrenchclient.call(currentWrench);

    vector<double> forces={currentWrench.response.fx,
                           currentWrench.response.fy,
                           currentWrench.response.fz};

	return forces[2]<force;
}

bool maxSpiralForces(double Fd){
    ros::service::waitForService("getWrench",timeout);
    ros::ServiceClient getWrenchclient = n_ptr->serviceClient<lock_key::getWrench>("getWrench");
    lock_key::getWrench currentWrench;
    getWrenchclient.call(currentWrench);

    vector<double> forces={currentWrench.response.fx,
                           currentWrench.response.fy,
                           currentWrench.response.fz};
    vector<double> torques={currentWrench.response.tx,
                            currentWrench.response.ty,
                            currentWrench.response.tz};

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
    //ROS_INFO("Moving to Relative Position");
}

void InsertPartSpiral(double Ft, double Fd, double Fi, double delta_max){
	// Implement Algorithm 1 from Paper here.

    // 11. MoveAbs - Move to insertion plane. Should already be
    //               accomplished by controller.cpp

    // 12. MoveRel - Move towards goal (xi,yi) until traveled 
    //               delta_max or MaxDownForce==false
    ROS_INFO("Moving to insertion plane");
    double delta{0.0};
    //Break delta up into smaller steps
    double delta_cmd{delta_max/20.0};
    while (delta<=delta_max && maxDownForce(-Ft)==0){
        moveRel(0.0,0.0,-delta_cmd,0.0,0.0,0.0,0.0);
        // Update delta. TODO: Use actual feedback rather than cmd
        delta-=delta_cmd;
    }

    // 13. MoveSpiral - Perform spiral motion while 
    //                  MaxSpiralForces is true
    double x{0}, y{0};           // Current EE position
    double prev_x{0}, prev_y{0}; // Previous EE position
    double r{0}, phi{0}; //Spiral radius and angle
    double r_step{0.00001}, phi_step{0.05}; //Step size for radius and angle
    int ii{1}; // Iteration Counter
    int ii_max{1000}; //Max number of iterations

    ROS_INFO("Starting Spiral Insertion Motion");

    while (ii<ii_max && maxSpiralForces(Fd)==1){
        //Calculate new xy
        x+=r*cos(phi); y+=r*sin(phi);
        ROS_INFO("Spiral X: %.4f, Y: %.4f", x, y);
        //Send relative movement command
        moveRel(x-prev_x,y-prev_y,0,0.0,0.0,0.0,0.0);
        //Update parameters
        ii+=1; r+=r_step, phi+=phi_step;
        prev_x=x; prev_y=y;

    }

    ROS_INFO("Performing final insertion");

    // 14. MoveRel - Finish insertion
    while (delta<=delta_max && maxDownForce(-Fi)==0){
        moveRel(0.0,0.0,-delta_cmd,0.0,0.0,0.0,0.0);
        // Update delta. TODO: Use actual feedback rather than cmd
        delta-=delta_cmd;
    }

}

void execute(const lock_key::SpiralInsertGoalConstPtr& goal, Server* as){  // Note: "Action" is not appended here
    ROS_INFO("Beginning insertion procedure");

    // Ft: Max force when moving to insertion plane
    // Fd: Max wrist Force/Torque during spiral
    // Fi: Max Insertion Force
    // delta_max: acceptable travel in z-axis

    InsertPartSpiral(goal->Ft, goal->Fd, goal->Fi, goal->delta_max);

    as->setSucceeded();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "spiralInsert");
    ros::NodeHandle n;
    n_ptr=&n;
    
    Server server(n, "spiral_insert_key", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::spin();
}