#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "ros_pcl_manip/ros_pcl_manip.h"

ROS_PCL::ROS_PCL(ros::NodeHandle& nh) : _nh(nh) {_setup_services();}
ROS_PCL::~ROS_PCL() {}

CloudPtr ROS_PCL::downsample(CloudPtr in_cloud, float leaf_size) {
  if(leaf_size == 0.0f)		// No change in size
    return in_cloud;
  CloudPtr ret(new CloudType());
  pcl::VoxelGrid<PointType> vg;

  vg.setInputCloud(in_cloud);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(*ret);

  return ret;
}
CloudPtr ROS_PCL::seg_plane(CloudPtr in_cloud, bool remove, float leaf_size) {
  // Variables
  CloudPtr downsampledCloud = downsample(in_cloud, leaf_size);
  CloudPtr extractedPlane;
  pcl::PointIndices::Ptr planeInliers(new pcl::PointIndices);
  pcl::ExtractIndices<PointType> extract;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<PointType> planeSeg;

  // Setup segmentation
  planeSeg.setOptimizeCoefficients(true);
  planeSeg.setModelType(pcl::SACMODEL_PLANE);
  planeSeg.setMethodType(pcl::SAC_RANSAC);
  planeSeg.setDistanceThreshold(0.01);
  planeSeg.setInputCloud(downsampledCloud);

  // Segment
  planeSeg.segment(*planeInliers, *coefficients);

  // Extract indices
  extract.setInputCloud(downsampledCloud);
  extract.setIndices(planeInliers);
  extract.setNegative(false);
  extract.filter(*extractedPlane);
  if(remove) {			// Replace the current cloud with no plane
    extract.setNegative(true);
    extract.filter(*in_cloud);
  }

  return extractedPlane;
}

bool ROS_PCL::downsample_service(ros_pcl_manip::Downsample::Request& req,
				 ros_pcl_manip::Downsample::Response& res) {
  CloudPtr tempPtr = from_pc2(req.cloud);
  ROS_INFO("Converted cloud into PCL");
  ROS_INFO_STREAM((*tempPtr));
  tempPtr = downsample(tempPtr, req.size);
  ROS_INFO("Downsampled cloud");

  res.cloud = to_pc2(tempPtr);

  return true;
}
bool ROS_PCL::segment_plane_service(ros_pcl_manip::SegmentPlane::Request& req,
				    ros_pcl_manip::SegmentPlane::Response& res) {
  CloudPtr negativePtr = from_pc2(req.cloud);
  CloudPtr planePtr;

  planePtr = seg_plane(negativePtr, true, req.leaf_size);

  res.plane = to_pc2(planePtr);
  res.negativeCloud = to_pc2(negativePtr);

  return true;
}

CloudPtr ROS_PCL::from_pc2(const sensor_msgs::PointCloud2& pc2) {
  ROS_INFO("Converting PC2");
  CloudPtr ret_cloud(new CloudType());
  pcl::PCLPointCloud2 temp_cloud;

  // Copy metadata
  ROS_INFO("Copying metadata");
  temp_cloud.header.stamp = pc2.header.stamp.toNSec() / 1000ull;
  temp_cloud.header.seq = pc2.header.seq;
  temp_cloud.header.frame_id = pc2.header.frame_id;
  temp_cloud.height = pc2.height;
  temp_cloud.width = pc2.width;
  temp_cloud.fields.resize(pc2.fields.size());
  ROS_INFO_STREAM("Copying fields " << pc2.fields.size());
  std::vector<sensor_msgs::PointField>::const_iterator it = pc2.fields.begin();
  int i = 0;
  for(; it != pc2.fields.end(); ++i, ++it) {
    temp_cloud.fields[i].name = (*it).name;
    temp_cloud.fields[i].offset = (*it).offset;
    temp_cloud.fields[i].datatype = (*it).datatype;
    temp_cloud.fields[i].count = (*it).count;
    ROS_INFO_STREAM("Copied field " << (*it));
  }
  temp_cloud.is_bigendian = pc2.is_bigendian;
  temp_cloud.point_step = pc2.point_step;
  temp_cloud.row_step = pc2.row_step;
  temp_cloud.is_dense = pc2.is_dense;

  // Copy data
  ROS_INFO("Copying data");
  temp_cloud.data = pc2.data;

  // Place into cloud
  ROS_INFO("Placing data into PCL Cloud");
  pcl::fromPCLPointCloud2(temp_cloud, (*ret_cloud));

  ROS_INFO("Returning cloud");
  return ret_cloud;
}
sensor_msgs::PointCloud2 ROS_PCL::to_pc2(CloudPtr in_cloud) {
  sensor_msgs::PointCloud2 ret;
  pcl::PCLPointCloud2 temp_cloud;

  pcl::toPCLPointCloud2((*in_cloud), temp_cloud);

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
  _plane_server = _nh.advertiseService("segment_plane",
				       &ROS_PCL::segment_plane_service,
				       this);
}
