#include <ros/ros.h>

#include "geometry_msgs/Point.h"
#include "owi_robot/generatecmd.h"
#include "owi_robot/writecmd.h"

geometry_msgs::Point curPose_;

geometry_msgs::Point get_error(geometry_msgs::Point goalPose){

    geometry_msgs::Point err;
    err.x = goalPose.x - curPose_.x;
    err.y = goalPose.y - curPose_.y;
    err.z = goalPose.z - curPose_.z;

    return err;
}

geometry_msgs::Point get_goal_pose(){

    geometry_msgs::Point goalPose;
    goalPose.x = 0.0;  //Read from YAML
    goalPose.y = 0.0;
    goalPose.z = 0.0;

    return goalPose;
}

void get_cur_pose(const geometry_msgs::Point &msg){

    curPose_.x = msg.x;
    curPose_.y = msg.y;
    curPose_.z = msg.z;

}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "owi_control");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("object_position",1,get_cur_pose);
    
    geometry_msgs::Point goal = get_goal_pose();
    geometry_msgs::Point err = get_error(goal);

    if(err.x>1e2){
        // move robot
    }


    
    return 0;
}