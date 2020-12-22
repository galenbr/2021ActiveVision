#include <ros/ros.h>
#include <math.h>

#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#define PI 3.14159265/180

int main(int argc, char **argv){

    ros::init(argc, argv, "sin_vel");
    ros::NodeHandle n;

    ros::Publisher sin_pub = n.advertise<std_msgs::Float64MultiArray>("pwm",1);
    float count = 0;
    float vel;
    std_msgs::Float64MultiArray vel_msg;
    vel_msg.data.resize(2);

    ros::Rate r{30};

    while(ros::ok()){
        if(count <=360){
            vel = sin(count*PI);
            
            vel_msg.data[0] = vel/2;
            vel_msg.data[1] = 0;
            
            sin_pub.publish(vel_msg);
            
            count += 1.5;
        }
        else{
            ROS_INFO("DONE!");
        }
            ros::spinOnce();
            r.sleep();
    }
        ros::spin();
    return 0;
}