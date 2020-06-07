#include <pcl/filters/voxel_grid.h>

#include "ros_pcl_manip/ros_pcl_manip.h"
#include "ros_pcl_manip/conversions.h"

ROS_PCL::ROS_PCL(ros::NodeHandle& nh) : _nh(nh) {_setup_services(); ros::spin();}
ROS_PCL::~ROS_PCL() {}

// pcl::PointCloud<pcl::PointXYZ> ROS_PCL::seg_cloud(const pcl::PointCloud<pcl::PointXYZ>& in_cloud) {
//   std::vector<pcl::PointXYZ> ret;

//   ret.push_back(in_cloud);

//   return ret;
// }

// Use cloudPointers
CloudPtr ROS_PCL::downsample(CloudPtr in_cloud, float leaf_size) {
  CloudPtr ret;
  pcl::VoxelGrid<PointType> vg;

  vg.setInputCloud(in_cloud);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(*ret);

  return ret;
}

bool ROS_PCL::downsample_service(ros_pcl_manip::Downsample::Request& req,
				 ros_pcl_manip::Downsample::Response& res) {
  CloudPtr tempPtr = from_pc2(req.cloud);
  tempPtr = downsample(tempPtr, req.size);

  res.cloud = to_pc2(tempPtr);

  return true;
}

CloudPtr ROS_PCL::from_pc2(const sensor_msgs::PointCloud2& pc2) {
  CloudPtr ret_cloud;
  pcl::PCLPointCloud2 temp_cloud;

  // Copy metadata
  temp_cloud.header.stamp = pc2.header.stamp.toNSec() / 1000ull;
  temp_cloud.header.seq = pc2.header.seq;
  temp_cloud.header.frame_id = pc2.header.frame_id;
  temp_cloud.height = pc2.height;
  temp_cloud.width = pc2.width;
  temp_cloud.fields.resize(pc2.fields.size());
  std::vector<sensor_msgs::PointField>::const_iterator it = pc2.fields.begin();
  int i = 0;
  for(; it != pc2.fields.end(); ++i, ++it) {
    temp_cloud.fields[i].name = (*it).name;
    temp_cloud.fields[i].offset = (*it).offset;
    temp_cloud.fields[i].datatype = (*it).datatype;
    temp_cloud.fields[i].count = (*it).count;
  }
  temp_cloud.is_bigendian = pc2.is_bigendian;
  temp_cloud.point_step = pc2.point_step;
  temp_cloud.row_step = pc2.row_step;
  temp_cloud.is_dense = pc2.is_dense;

  // Copy data
  temp_cloud.data = pc2.data;

  // Place into cloud
  pcl::fromPCLPointCloud2(temp_cloud, (*ret_cloud));

  return ret_cloud;
}
sensor_msgs::PointCloud2 ROS_PCL::to_pc2(CloudPtr in_cloud) {
  sensor_msgs::PointCloud2 ret;
  pcl::PCLPointCloud2 temp_cloud;

  pcl::toPCLPointCloud2((*in_cloud), temp_cloud);

  // TODO: Finish this
  ret.header.stamp.fromNSec(temp_cloud.header.stamp  *1000ull);
  ret.header.seq = temp_cloud.header.seq;
  ret.header.frame_id = temp_cloud.header.frame_id;
  ret.height = temp_cloud.height;
  ret.width = temp_cloud.width;
  ret.fields.resize(temp_cloud.fields.size());
  std::vector<pcl::PCLPointField>::const_iterator it = temp_cloud.fields.begin();
  int i = 0;
  for(; it != temp_cloud.fields.end(); ++i, ++it) {
    ret.fields[i].name = (*it).name;
    ret.fields[i].offset = (*it).offset;
    ret.fields[i].datatype = (*it).datatype;
    ret.fields[i].count = (*it).count;
  }
  ret.is_bigendian = temp_cloud.is_bigendian;
  ret.point_step = temp_cloud.point_step;
  ret.row_step = temp_cloud.row_step;
  ret.is_dense = temp_cloud.is_dense;
  ret.data.swap(temp_cloud.data);

  return ret;
}

void ROS_PCL::_setup_services() {
  _down_server = _nh.advertiseService("downsample",
				      &ROS_PCL::downsample_service,
				      this);
}
