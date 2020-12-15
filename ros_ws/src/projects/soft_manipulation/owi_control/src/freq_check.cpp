#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include <geometry_msgs/Point.h>

#define rad 0.0174533

float prevTime = 0;
float old_theta1 = 0;
float angular_vel = 0;
ros::Publisher pub;
float dt = 0.033;
void diff(const geometry_msgs::Point &msg){
    
    float theta1 = msg.x * rad;    
    float curTime = ros::Time::now().toSec();
    if(theta1 != old_theta1){
        angular_vel = (theta1 - old_theta1)/dt;
        dt = 0.033;
    }
    else{
        dt += 0.033;
    }
    // ROS_INFO("time %f :", curTime);
    
    old_theta1 = theta1;
    prevTime = curTime;

    std_msgs::Float32 angular_msg;
    angular_msg.data = angular_vel;
    pub.publish(angular_msg);

}

int main(int argc, char **argv){

    ros::init(argc, argv, "freq_response");
    ros::NodeHandle n;

    //subscribe to angles
    ros::Subscriber sub = n.subscribe("theta_vals", 1, diff);
    // callback: (cur angle - old angle)/(timestamp cur - timestamp old)
    
    // publish this amgular velocity
    pub = n.advertise<std_msgs::Float32>("angular_vel",1);
    
    // ros::Rate r{30};
    // while(ros::ok()){
        
    
    //     ros::spinOnce();
    //     r.sleep();
    // }
    //record angular velocity
    //plot angular velocity




    ros::spin();
    return 0;
}