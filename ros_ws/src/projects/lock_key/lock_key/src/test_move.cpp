#include <ros/ros.h>
#include "moveit_planner/MoveCart.h"

int main(int argc, char ** argv){
    ros::init(argc, argv, "test_move");
    ros::NodeHandle n;

    ros::ServiceClient cartMoveClient = n.serviceClient<moveit_planner::MoveCart>("cartesian_move");

    moveit_planner::MoveCart move;
    geometry_msgs::Pose p;
    p.orientation.w = 0.7071068;
    p.orientation.x = 0;
    p.orientation.y = 0.7071068;
    p.orientation.z = 0;
    p.position.x = 0.457262 - 0.06;
    p.position.y = 0.104397
    p.position.z = 0.264967 - 0.006;
    
    // p.position.x = 0.339306-0.06;
    // p.position.y = 0.105677;
    // p.position.z= 0.758311-0.006;

    // Move away
    move.request.val.push_back(p);
    p.position.x -= 0.04;
    move.request.val.push_back(p);
    p.position.z += 0.05;
    move.request.val.push_back(p);
    move.request.execute = true;
    cartMoveClient.call(move);
    ros::Duration(1).sleep();

    //ros::spin();
    return 0;

}