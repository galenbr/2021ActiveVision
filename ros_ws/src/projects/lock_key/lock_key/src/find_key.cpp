#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <lock_key/findKey.h>

bool key_service_callback(lock_key::findKey::Request &req, lock_key::findKey::Response &res){

    //Do pointcloud stuff here to find key



    return true;

}

int main(int argc, char **argv){

    ros::init(argc, argv, "findKey");
    ros::NodeHandle n;

    ros::ServiceServer key = n.advertiseService("findKeyServer", key_service_callback);

    ros::spin();
}