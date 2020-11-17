#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <math.h>

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"

#define PI 3.14159265/180

ros::NodeHandle* nh;
ros::Publisher pub;
ros::Publisher err_pub;
Eigen::MatrixXd Jr(2,2);

geometry_msgs::Point curPose_;

geometry_msgs::Point get_goal_pose(){

    geometry_msgs::Point goalPose;
    
    goalPose.x = -0.096145;  //Read from YAML
    goalPose.y = -0.049568;
    goalPose.z = -0.007454;

    return goalPose;
}

void get_cur_pose(const geometry_msgs::Point &msg){

    curPose_.x = msg.x;
    curPose_.y = msg.y;
    curPose_.z = msg.z;   

    geometry_msgs::Point goal = get_goal_pose();
    
    Eigen::Vector3d f(curPose_.x, curPose_.y, curPose_.z);
    Eigen::Vector3d f_d(goal.x, goal.y, goal.z);
    Eigen::Vector2d err;
    Eigen::Vector2d t_dot;      // velocity in task space
    Eigen::Vector2d j_dot;      // velocity in joint space

    err(0) = f_d(0) - f(0);
    err(1) = f_d(1) - f(1);

    float lam = 1.95; // in m
    float z = 0.4;
    int k = 320;
    // float l1 = 0.09;    // in m
    // float l2 = 0.065;   // in m
    // float th1 = 45;
    // float th2 = -45;

    // float jr1 = - l1*sin(th1*PI) - l2*sin((th1+th2)*PI);
    // float jr2 = - l2*sin((th1 + th2)*PI);
    // float jr3 = l1*cos(th1*PI) + l2*cos((th1 + th2)*PI);
    // float jr4 = l2*cos((th1 + th2)*PI);
    
    if(abs(err(1))>=0.0001){
    // Eigen::MatrixXd Jv(2,2); // Image Jacobian
    // Jv << 1,0,
    //       0,1;

    Eigen::MatrixXd Kp(2,2); //Gains matrix
    Kp << k, 0,
          0, k;

    // Eigen::MatrixXd Jv_inv(2,2);
    // if(Jv.determinant() != 0)
    //     Jv_inv = Jv.inverse();

    // Eigen::MatrixXd Jr(2,2); // Robot Jacobian 
    // Jr << jr1, jr2,
    //       jr3, jr4;
    // std::cout << "Jv: \n" << Jr_inv <<std::endl;

    Eigen::MatrixXd Jr_inv(2,2);
        Jr_inv << -15,  0,
                   0, 15; //31. 15;

    t_dot = -Kp * err;

    j_dot = Jr_inv * t_dot;
    j_dot /= 25;        //normalization
    printf("%f\n",j_dot(1));

    // Piecewise function
    // if(j_dot(0) >= 0.4){
    //     j_dot(0) = 0.4;
    // }
    // else if(j_dot(0) <= -0.4){
    //     j_dot(0) = -0.4;
    // }
    // else if(j_dot(0) < 0){
    //     if(j_dot(0) > -0.2)
    //         j_dot(0) = 0;
    // }
    // else if(j_dot(0) > 0){
    //     if(j_dot(0) < 0.2)
    //         j_dot(0) = 0;
    // }

    // if(j_dot(1) >= 0.4){
    //     j_dot(1) = 0.4;
    // }
    // else if(j_dot(1) <= -0.4){
    //     j_dot(1) = -0.4;
    // }
    // else if(j_dot(1) < 0){
    //     if(j_dot(1) > -0.2)
    //         j_dot(1) = 0;
    // }
    // else if(j_dot(1) > 0){
    //     if(j_dot(1) < 0.2)
    //         j_dot(1) = 0;
    // }

    if(j_dot(1)>0.3){
        j_dot(1) = 0.3;
    }
    if(j_dot(1)<-0.3){
        j_dot(1) = -0.3;
    }

    std_msgs::Float64MultiArray vel_msg;    //Velocity Message for Pub
    vel_msg.data.resize(2);
    
    vel_msg.data[0] = 0;//j_dot(0);
    vel_msg.data[1] = j_dot(1);

    pub.publish(vel_msg);

    std_msgs::Float64MultiArray err_msg;
    err_msg.data.resize(3);
    
    err_msg.data[0] = err(0);
    err_msg.data[1] = err(1);
    err_msg.data[2] = 0;

    err_pub.publish(err_msg);
    }
    else{
            std_msgs::Float64MultiArray vel_msg;    //Velocity Message for Pub
            vel_msg.data.resize(2);
    
            vel_msg.data[0] = 0;//j_dot(0);
            vel_msg.data[1] = 0;

            pub.publish(vel_msg);
            std_msgs::Float64MultiArray err_msg;
            err_msg.data.resize(3);
    
            err_msg.data[0] = err(0);
            err_msg.data[1] = err(1);
            err_msg.data[2] = 0;

    err_pub.publish(err_msg);
    }
}

void update_Jr(const geometry_msgs::Point &msg){

    float th1 = msg.x;
    float th2 = msg.y;

    float l1 = 0.09;    // in m
    float l2 = 0.065;   // in m

    float jr1 = - l1*sin(th1*PI) - l2*sin((th1+th2)*PI);
    float jr2 = - l2*sin((th1 + th2)*PI);
    float jr3 = l1*cos(th1*PI) + l2*cos((th1 + th2)*PI);
    float jr4 = l2*cos((th1 + th2)*PI);
    
    Jr << jr1, jr2,
          jr3, jr4;


}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "owi_controller");
    // ros::NodeHandle n;
    nh = new ros::NodeHandle;

    
    ros::Subscriber sub = nh->subscribe("object_position",1,get_cur_pose);
    ros::Subscriber theta_sub = nh->subscribe("theta_vals",1,update_Jr);
    pub = nh->advertise<std_msgs::Float64MultiArray>("pwm",10);
    err_pub = nh->advertise<std_msgs::Float64MultiArray>("error",10);

    
    ros::spin();
    return 0;
}