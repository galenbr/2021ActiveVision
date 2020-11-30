#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <image_transport/image_transport.h>

#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/image_encodings.h"

cv::Mat inImage;
std::vector<float> traj_points_X;
std::vector<float> traj_points_Y;

void trajPointsCallback(const geometry_msgs::PointStamped &msg){
    traj_points_X.push_back(msg.point.x);
    traj_points_Y.push_back(msg.point.y);
}

void trajPlotCallback(const sensor_msgs::ImageConstPtr& img){

}

int main(int argc, char **argv){
    ros::init(argc, argv, "traj_plotter");
    ros::NodeHandle n;

    //subscribe to ee pose
    ros::Subscriber traj_sub = n.subscribe("/aruco_simple/pixel2",1,trajPointsCallback);
    //callback: everytime new pose is received, pushback into a vector
    //subscribe to image
    ros::Subscriber img_sub = n.subscribe("/a",1,trajPlotCallback);
    //callback: draw line between successive pairwise points in the vector
    // publish the new image
    ros::spin();
    return 0;
}