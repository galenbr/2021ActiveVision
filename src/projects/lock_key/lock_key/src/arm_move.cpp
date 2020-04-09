#include <ros/ros.h>
//#include "movement.cpp"
#include <pcl/point_cloud.h>
#include "moveit_planner/MoveCart.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "movement");
    ros::NodeHandle n;

    ROS_INFO("Creating Movement Object");
    //Movement m(n);

    return 0;
}
