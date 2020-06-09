#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "ros_pcl_manip/Downsample.h"
#include "ros_pcl_manip/SegmentPlane.h"

sensor_msgs::PointCloud2 img;

void img_callback(const sensor_msgs::PointCloud2& msg) {
  img = msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_vision_node");
  ros::NodeHandle nh;

  ros::Subscriber imgSub = nh.subscribe("/fixed_camera/depth/points", 1, img_callback);
  while(img.data.size() == 0)
    ros::spinOnce();		// Spin once to get an image

  // Send downsample request
  ros::ServiceClient downClient = nh.serviceClient<ros_pcl_manip::Downsample>("downsample");
  ros_pcl_manip::Downsample dsReq;
  dsReq.request.cloud = img;
  dsReq.request.size = 0.01;
  downClient.call(dsReq);
  ros::Publisher down_pub = nh.advertise<sensor_msgs::PointCloud2>("downsample_result", 1);

  // Send plane seg request
  img = sensor_msgs::PointCloud2();
  while(img.data.size() == 0)
    ros::spinOnce();		// Spin once to get an image
  ros::ServiceClient planeClient = nh.serviceClient<ros_pcl_manip::SegmentPlane>("segment_plane");
  ros_pcl_manip::SegmentPlane planeReq;
  planeReq.request.cloud = img;
  planeClient.call(planeReq);
  ros::Publisher plane_pub = nh.advertise<sensor_msgs::PointCloud2>("segment_plane_result", 1);
  ros::Publisher neg_plane_pub = nh.advertise<sensor_msgs::PointCloud2>("segment_plane_negative_result", 1);

  while(ros::ok()) {
    down_pub.publish(dsReq.response.cloud);
    plane_pub.publish(planeReq.response.plane);
    neg_plane_pub.publish(planeReq.response.negativeCloud);
    ros::spinOnce();
  }

  return 0;
}
