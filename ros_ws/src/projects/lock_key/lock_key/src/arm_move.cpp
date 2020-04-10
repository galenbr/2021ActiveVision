// The arm_move node handles movement for the robot. It pulls services from the moveit_planner node.
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include "moveit_planner/MoveCart.h"
#include "moveit_planner/MovePose.h"

void wait_for_service(int32_t timeout){
    ros::service::waitForService("move_to_pose",timeout);
}

void setup_service_client(ros::NodeHandle n){
    ros::ServiceClient MovePoseClient = n.serviceClient<moveit_planner::MovePose>("move_to_pose");
}

int main(int argc, char **argv){

    ros::init(argc, argv, "movement");
    ros::NodeHandle n;

    //Waiting for services to be available
    int32_t timeout = 1000;
    wait_for_service(timeout);

    //Setting up Service Clients
    //setup_service_client(n);
    
    ros::ServiceClient MovePoseClient = n.serviceClient<moveit_planner::MovePose>("move_to_pose");

    moveit_planner::MovePose pose;

    pose.request.val.position.x = 0.43;
    pose.request.val.position.y = 0.0;
    pose.request.val.position.z = 0.305;
    pose.request.val.orientation.w = 0.00179342;
    pose.request.val.orientation.x = 0.92404;
    pose.request.val.orientation.y = 0.382283;
    pose.request.val.orientation.z = -0.00269858;
    pose.request.execute = true;

    MovePoseClient.call(pose);
    ROS_INFO("Move Pose Client Call");





    return 0;
}
