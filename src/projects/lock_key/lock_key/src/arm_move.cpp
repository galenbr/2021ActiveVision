#include <ros/ros.h>
#include "movement.cpp"

int main(int argc, char **argv){

    ros::init(argc, argv, "movement");
    ros::NodeHandle n;

    ROS_INFO("Creating Movement Object");
    Movement m(n);

    return 0;
}
