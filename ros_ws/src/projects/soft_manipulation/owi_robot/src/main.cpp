#include <ros/ros.h>

#include "geometry_msgs/Point.h"
#include "owi_robot/generatecmd.h"
#include "owi_robot/writecmd.h"

geometry_msgs::Point curPose_;
geometry_msgs::Point goalPose_;
geometry_msgs::Point err_;

void get_error(){

    err_.x = goalPose_.x - curPose_.x;
    err_.y = goalPose_.y - curPose_.y;
    err_.z = goalPose_.z - curPose_.z;
}

void get_goal_pose(){
    goalPose_.x = 0.0;  //Read from YAML
    goalPose_.y = 0.0;
    goalPose_.z = 0.0;
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
    get_goal_pose();

    geometry_msgs::Point err;

    get_error();

    if(err_.x>1e2){
        // move robot
    }


    
    return 0;
}