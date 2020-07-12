#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
///////////////////////////////////////////////////////////////
//Ransac Filtering
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include "testing_image_transport/image_stitching.h"
#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/box_clipper3D.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <Eigen/Dense>
#include <unistd.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/visualization/cloud_viewer.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/features/integral_image_normal.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <pcl/features/integral_image_normal.h>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_state/conversions.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <boost/scoped_ptr.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>

ros::NodeHandle *nhptr;
ros::Publisher image_arr_pub;

using namespace std;

void testCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  transform_1(0,0) = 0;
  transform_1(0,1) = 0;
  transform_1(0,2) = 1;
  transform_1(0,3) = 0;
  transform_1(1,0) = -1;
  transform_1(1,1) = 0;
  transform_1(1,2) = 0;
  transform_1(2,0) = 0;
  transform_1(2,1) = -1;
  transform_1(2,2) = 0;
  transform_1(3,0) = 0;
  transform_1(3,1) = 0;
  transform_1(3,2) = 0;
  transform_1(3,3) = 1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg (*cloud_msg, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*cloud, *transformed_cloud, transform_1);

  image_arr_pub.publish(*transformed_cloud);
}

int main(int argc, char** argv)
{

ros::init(argc, argv, "rotate_cloud_publisher");

  // ros::init(argc, argv, "sub_pcl");

  ros::NodeHandle nh;

  nhptr = &nh;

	image_arr_pub = nhptr->advertise<sensor_msgs::PointCloud2>("image_arr_topic", 1000);

    ros::Subscriber subPoints = nh.subscribe("camera/depth/points225", 1, testCallback);

    ros::spin();
}