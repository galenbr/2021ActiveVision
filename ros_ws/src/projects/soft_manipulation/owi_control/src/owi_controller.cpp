#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <math.h>

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"

#define PI 3.14159265/180

geometry_msgs::Point curPose_;

geometry_msgs::Point get_goal_pose(){

    geometry_msgs::Point goalPose;
    goalPose.x = 0.23;  //Read from YAML
    goalPose.y = -0.14;
    goalPose.z = -0.17;

    return goalPose;
}

void get_cur_pose(const geometry_msgs::Point &msg){

    curPose_.x = msg.x;
    curPose_.y = msg.y;
    curPose_.z = msg.z;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "owi_controller");
    ros::NodeHandle n;

    
    ros::Subscriber sub = n.subscribe("object_position",1,get_cur_pose);
    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("pwm",1);
    ros::Publisher err_pub = n.advertise<std_msgs::Float64MultiArray>("error",1);

    geometry_msgs::Point goal = get_goal_pose();
    
    Eigen::Vector3d f(curPose_.x, curPose_.y, curPose_.z);
    Eigen::Vector3d f_d(goal.x, goal.y, goal.z);
    Eigen::Vector3d err;
    Eigen::Vector3d t_dot;
    Eigen::Vector3d j_dot;

    err = f_d - f;
    float lam = 0.035; // in m
    float z = 0.1;
    int kp = 1;
    float l1 = 0.09;    // in m
    float l2 = 0.065;   // in m
    float th1 = 0;
    float th2 = 0;

    float jr1 = - l1*sin(th1*PI) - l2*sin((th1+th2)*PI);
    float jr2 = - l2*sin((th1 + th2)*PI);
    float jr3 = l1*cos(th1*PI) + l2*cos((th1 + th2)*PI);
    float jr4 = l2*cos((th1 + th2)*PI);

    Eigen::MatrixXd Jv(2,2); // Image Jacobian
    Jv << lam/z,0,
         0,lam/z;

    Eigen::MatrixXd Kp(2,2); //Gains matrix
    Kp << kp, 0,
          0, kp;

    Eigen::MatrixXd Jv_inv(2,2);
    if(Jv.determinant() != 0)
        Jv_inv = Jv.inverse();

    Eigen::MatrixXd Jr(2,2); // Robot Jacobian 
    Jr << jr1, jr2,
          jr3, jr4;

    Eigen::MatrixXd Jr_inv(2,2);
    if(Jr.determinant() != 0)
        Jr_inv = Jr.inverse();

    t_dot = (Kp * Jv_inv) * err;
    
    j_dot = Jr_inv * t_dot;
    j_dot /= 10;        //normalization
    
    // Piecewise function
    if(j_dot(1) > 1.0){
        j_dot(1) = 1.0;
    }
    else if(j_dot(1) < -1.0){
        j_dot(1) = -1.0;
    }
    else if(j_dot(1) > -0.45 && j_dot(1) < 0.45){
        j_dot(1) = 0;
    }

    if(j_dot(2) > 1.0){
        j_dot(2) = 1.0;
    }
    else if(j_dot(2) < -1.0){
        j_dot(2) = -1.0;
    }
    else if(j_dot(2) > -0.45 && j_dot(2) < 0.45){
        j_dot(2) = 0;
    }

    std_msgs::Float64MultiArray vel_msg;    //Velocity Message for Pub
    vel_msg.data.resize(2);
    
    vel_msg.data[0] = j_dot(1);
    vel_msg.data[1] = j_dot(2);

    pub.publish(vel_msg);

    std_msgs::Float64MultiArray err_msg;
    err_msg.data.resize(3);
    
    err_msg.data[0] = err(1);
    err_msg.data[1] = err(2);
    err_msg.data[2] = err(3);

    err_pub.publish(err_msg);




    ros::spin();
    return 0;
}