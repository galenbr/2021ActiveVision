// Project to 2D and return an image that can be used by python numpy

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "rrbot_pcl/projectCloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "sensor_msgs/Image.h"

bool projectCloud(rrbot_pcl::projectCloud::Request &req, rrbot_pcl::projectCloud::Response &res){
    
    // Convert sensor msgs pc2 to pcl pointcloud2
    pcl::PCLPointCloud2::Ptr pcl_pc2 (new pcl::PCLPointCloud2 ());
    std::string frameID = "/camera_optical_link";
    pcl_conversions::toPCL(req.pc2, *pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*pcl_pc2, *cloud);

    // Projecting all points to a plane
    for(int i = 0; i < cloud->points.size(); i++)
        cloud->points[i].z = 3.1;
    ROS_INFO("Cloud Projected");
    // Convert to cv::Mat
    // Then Convert to sensor_msgs/Image
    
    // Convert to sensor_msgs/Image
    sensor_msgs::Image img;
    try{
        pcl::toROSMsg(*cloud, img);
        img.encoding = "rgb8";    
    }
    catch (std::runtime_error e){
        ROS_ERROR_STREAM("UNABLE TO CONVERT:" << e.what());
    }

    res.img = img;

    return true;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "project_cloud_node");
    ros::NodeHandle n;

    ros::ServiceServer projectCloudServ = n.advertiseService("project_cloud", projectCloud);

    ros::spin();
    return 0;
}