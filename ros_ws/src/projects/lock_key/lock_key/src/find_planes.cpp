#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "lock_key_msgs/FindPlanes.h"
#include <iostream>
// PCL Includes for conversion
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
// Downsampling
#include <pcl/filters/voxel_grid.h>
// Segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// Centroid
#include <pcl/common/centroid.h>

bool plane_service_callback(lock_key_msgs::FindPlanes::Request &req, lock_key_msgs::FindPlanes::Response &res){
    ROS_INFO("Finding primary planes");
    // Convert sensor_msgs to pcl pointcloud2
    // TO DO: Put in a function
    pcl::PCLPointCloud2::Ptr pcl_pc2 (new pcl::PCLPointCloud2 ());
    std::string frameID = "/camera_depth_optical_frame"; //camera_color_optical_frame
    pcl_conversions::toPCL(req.pc2, *pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*pcl_pc2, *cloud);

    //Downsampling the pointcloud
    // TO DO: Put in a function
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);  
    // std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
    //    << " data points (" << pcl::getFieldsList (*cloud) << ").";

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize(0.002f, 0.002f, 0.002f);
    sor.filter(*cloud_filtered);

    // std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
    //    << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

    // Lets Try Segmentation
    // TO DO: Put in a function

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices); 
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01); //0.001

    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.3 * nr_points){
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0){
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.05); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    //Separate out the different cloud segments
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_key (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_lock (new pcl::PointCloud<pcl::PointXYZRGB>);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        if (j==0){     
            //Key   
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster_key->points.push_back (cloud_filtered->points[*pit]); //*
            cloud_cluster_key->width = cloud_cluster_key->points.size ();
            cloud_cluster_key->height = 1;
            cloud_cluster_key->is_dense = true;
        } else if (j==1){
            //Lock
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster_lock->points.push_back (cloud_filtered->points[*pit]); //*
            cloud_cluster_lock->width = cloud_cluster_lock->points.size ();
            cloud_cluster_lock->height = 1;
            cloud_cluster_lock->is_dense = true;
        }
        j++;
    }

    //UPDATE THE .SRV FILE and UPDATE THE SECTION BELOW TO SEND 
    //BOTH POINT CLOUDS.

    //DO PLANE DETECTION AGAIN???

    // Convert Cloud Cluster to PCL
    // TO DO: put in function
    sensor_msgs::PointCloud2 ret_key, ret_lock;
    pcl::PCLPointCloud2 temppc2_key, temppc2_lock;
    pcl::toPCLPointCloud2(*cloud_cluster_key, temppc2_key); //*cloud_cluster, *cloud_filtered, *cloud_plane
    pcl::toPCLPointCloud2(*cloud_cluster_lock, temppc2_lock); //*cloud_cluster, *cloud_filtered, *cloud_plane
    pcl_conversions::fromPCL(temppc2_key, ret_key);
    pcl_conversions::fromPCL(temppc2_lock, ret_lock);

    // Fill up the return object
    ret_key.header.stamp = ros::Time();
    ret_key.header.frame_id = frameID;
    ret_lock.header.stamp = ros::Time();
    ret_lock.header.frame_id = frameID;
    res.key_cloud = ret_key;
    res.lock_cloud = ret_lock;

    return true;

}

int main(int argc, char **argv){

    ros::init(argc, argv, "FindPlanes");
    ros::NodeHandle n;
    
    ros::ServiceServer key = n.advertiseService("FindPlanesServer", plane_service_callback);

    ros::spin();
}