#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <lock_key/findKey.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>

bool key_service_callback(lock_key::findKey::Request &req, lock_key::findKey::Response &res){
    ROS_INFO("FINDING KEY");
    // Convert sensor_msgs to pcl pointcloud2
    // TO DO: Put in a function
    pcl::PCLPointCloud2::Ptr pcl_pc2 (new pcl::PCLPointCloud2 ());
    
    pcl_conversions::toPCL(req.pc2, *pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pcl_pc2, *cloud);

    //Downsampling the pointcloud
    // TO DO: Put in a function
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    // std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
    //    << " data points (" << pcl::getFieldsList (*cloud) << ").";

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (pcl_pc2);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);

    // std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
    //    << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

    // Lets Try Segmentation
    // TO DO: Put in a function

    



    return true;

}

int main(int argc, char **argv){

    ros::init(argc, argv, "findKey");
    ros::NodeHandle n;
    
    ros::ServiceServer key = n.advertiseService("findKeyServer", key_service_callback);

    ros::spin();
}