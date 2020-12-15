#include <ros/ros.h>
#include <rosbag/bag.h>
#include <time.h>

#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/Image.h>

std_msgs::Float32 w1;
std_msgs::Float64 pwm1;
sensor_msgs::Image img;

void rec_img(const sensor_msgs::Image image){

    img = image;

}

void rec_vel(const std_msgs::Float32 &msg){
    w1.data = msg.data;
}

void rec_pwm(const std_msgs::Float64MultiArray &msg){
    pwm1.data = msg.data.at(0);
}

int main(int argc, char** argv){

    rosbag::Bag bag;
    bag.open("/home/abhinav/freq_test.bag", rosbag::bagmode::Write);

    ros::init(argc, argv, "record_freq");
    ros::NodeHandle n;

    ros::Subscriber sub1 = n.subscribe("angular_vel",1,rec_vel);
    ros::Subscriber sub2 = n.subscribe("pwm",1,rec_pwm);
    ros::Subscriber sub3 = n.subscribe("usb_cam/image_raw",1,rec_img);

    ros::Rate r{30};
    while(ros::ok()){
        ros::Time t = ros::Time::now();

        bag.write("angular_vel", t, w1);
        bag.write("pwm", t, pwm1);
        // bag.write("err_Z", t, err_Z);
        bag.write("image", t, img);

        ros::spinOnce();
        r.sleep();
    }

    bag.close();

    ros::spin();


    return 0;
}