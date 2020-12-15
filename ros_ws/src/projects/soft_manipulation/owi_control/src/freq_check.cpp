#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include <geometry_msgs/Point.h>

#define rad 0.0174533

float prevTime = 0;
float old_theta1 = 0;
float angular_vel = 0;

void diff(const geometry_msgs::Point &msg){
    
    float theta1 = msg.x * rad;    
    float curTime = ros::Time::now().toSec();
    angular_vel = (theta1 - old_theta1)/0.033;
    // ROS_INFO("time %f :", curTime);
    old_theta1 = theta1;
    prevTime = curTime;

}

int main(int argc, char **argv){

    ros::init(argc, argv, "freq_response");
    ros::NodeHandle n;

    //subscribe to angles
    ros::Subscriber sub = n.subscribe("theta_vals", 1, diff);
    // callback: (cur angle - old angle)/(timestamp cur - timestamp old)
    
    // publish this amgular velocity
    ros::Publisher pub = n.advertise<std_msgs::Float32>("angular_vel",1);
    std_msgs::Float32 angular_msg;

    ros::Rate r{30};
    while(ros::ok()){
        angular_msg.data = angular_vel;
        pub.publish(angular_msg);
    
        ros::spinOnce();
        r.sleep();
    }
    //record angular velocity
    //plot angular velocity




    ros::spin();
    return 0;
}