#include <vector>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL includes
#include <pcl/point_types.h>

// SRV includes
#include <ros_pcl_manip/Downsample.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;
typedef CloudType::Ptr CloudPtr;
typedef CloudType::ConstPtr CloudConstPtr;

class ROS_PCL {
public:

  ROS_PCL(ros::NodeHandle& nh);
  ~ROS_PCL();

  // Non-service methods
  // std::vector<pcl::PointCloud<pcl::PointXYZ>> seg_cloud(const pcl::PointCloud<pcl::PointXYZ>& in_cloud);
  CloudPtr downsample(CloudPtr in_cloud, float leaf_size);

  // Conversions
  CloudPtr from_pc2(const sensor_msgs::PointCloud2& pc2);
  sensor_msgs::PointCloud2 to_pc2(CloudPtr in_cloud);

  // Service methods
  bool downsample_service(ros_pcl_manip::Downsample::Request& req,
			  ros_pcl_manip::Downsample::Response& res);
private:
  ros::NodeHandle& _nh;
  ros::ServiceServer _down_server;

  void _setup_services();
};
