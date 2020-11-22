#include <ros/ros.h>
#include "moveit_planner/MoveCart.h"

int main(int argc, char ** argv){
    ros::init(argc, argv, "actual_control");
    ros::NodeHandle n;

    ros::ServiceClient cartMoveClient = n.serviceClient<moveit_planner::MoveCart>("cartesian_move");
    ros::service::waitForService("cartesian_move",-1);

    // Move to Key
    moveit_planner::MoveCart move;
    geometry_msgs::Pose p;
    p.orientation.x = 0.83;
    p.orientation.y = -0.55;
    p.orientation.z = 0.01;
    p.orientation.w = -0.01;
    p.position.x = 0.5;
    p.position.y = 0.2;
    p.position.z = 0.1;
    move.request.val.push_back(p);
    move.request.execute = true;
    cartMoveClient.call(move);
    ROS_INFO("Reached key!");

    // Move to Lock
    moveit_planner::MoveCart move2;
    geometry_msgs::Pose p2;
    p2.orientation.x = 0.96;
    p2.orientation.y = -0.26;
    p2.orientation.z = 0.01;
    p2.orientation.w = -0.01;
    p2.position.x = 0.5;
    p2.position.y = -0.2;
    p2.position.z = 0.1;
    move2.request.val.push_back(p2);
    move2.request.execute = true;
    cartMoveClient.call(move2);
    ROS_INFO("Reached lock!");

    // Move to Home (roughly)
    moveit_planner::MoveCart move3;
    geometry_msgs::Pose p3;
    p3.orientation.x = 0.83;
    p3.orientation.y = -0.55;
    p3.orientation.z = 0.01;
    p3.orientation.w = -0.01;
    p3.position.x = 0.3;
    p3.position.y = 0.0;
    p3.position.z = 0.6;
    move3.request.val.push_back(p3);
    move3.request.execute = true;
    cartMoveClient.call(move3);
    ROS_INFO("Reached Home!");

    return 0;

}