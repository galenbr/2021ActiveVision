//This node records images and sensor readings in a rosbag
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Image.h>
#include <memory>

geometry_msgs::WrenchStamped msg;
sensor_msgs::Image img;
rosbag::Bag* bag;
std::string ft_sensor; std::string camera;

void msg_callback(const geometry_msgs::WrenchStamped message){
    msg = message;
    bag->write(ft_sensor, ros::Time::now(), message);
}

void img_callback(const sensor_msgs::Image image){
    img = image;
    bag->write(camera, ros::Time::now(), img);
}

int main(int argc, char** argv){
    ros::init(argc, argv,"record");
    ros::NodeHandle n;
    
    bag = new rosbag::Bag();
    bag->open("/home/agandhi2/test.bag", rosbag::bagmode::Write);    
    ROS_INFO("Opening bag");
    
    n.getParam("topics/topic1", ft_sensor);
    n.getParam("topics/topic2", camera);
    
    ros::Subscriber sub = n.subscribe(ft_sensor, 1, msg_callback);
    ros::Subscriber sub2 = n.subscribe(camera, 1, img_callback);    

    //Recording the topics at a rate of 30Hz.
    //Increase the rate in multiples of 30 since the camera publishes at 30fps

    ros::Rate r{30};
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    bag->close();
}