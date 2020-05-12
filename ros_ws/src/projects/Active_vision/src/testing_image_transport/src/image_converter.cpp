

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>

//Ransac Filtering
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  if(!viewer->updatePointCloud(cloud, "sample cloud"))
  {
	viewer->addPointCloud(cloud, "sample cloud");
  }
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->initCameraParameters ();
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  return (viewer);
}
//filters out the table and just shows the objects on the table
void testCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
//Grab the pcl cloud from ros message
  pcl::fromROSMsg (*cloud_msg, *cloud);

    // Get the plane model, if present.
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	segmentation.setInputCloud(cloud);
	segmentation.setModelType(pcl::SACMODEL_PLANE);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(0.01);
	segmentation.setOptimizeCoefficients(true);
	pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
	segmentation.segment(*planeIndices, *coefficients);

	if (planeIndices->indices.size() == 0)
		std::cout << "Could not find a plane in the scene." << std::endl;
	else
	{
		// Copy the points of the plane to a new cloud.
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(planeIndices);
		extract.filter(*plane);

		// Retrieve the convex hull.
		pcl::ConvexHull<pcl::PointXYZ> hull;
		hull.setInputCloud(plane);
		// Make sure that the resulting hull is bidimensional.
		hull.setDimension(2);
		hull.reconstruct(*convexHull);

		// Redundant check.
		if (hull.getDimension() == 2)
		{
			// Prism object.
			pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
			prism.setInputCloud(cloud);
			prism.setInputPlanarHull(convexHull);
			// First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
			// Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect.
			prism.setHeightLimits(0.01f, 1.5f);
			pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

			prism.segment(*objectIndices);

			// Get and show all points retrieved by the hull.
			extract.setIndices(objectIndices);
			extract.filter(*objects);
			//pcl::visualization::CloudViewer viewerObjects("Objects on table");
			simpleVis(objects);
			//viewerObjects.showCloud(objects);
			//while (!viewerObjects.wasStopped())
			//{
				// Do nothing but wait.
			//}
		}
		else std::cout << "The chosen hull is not planar." << std::endl;
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");

  ros::init(argc, argv, "sub_pcl");

  ros::NodeHandle nh;
  
  ros::Subscriber subPoints = nh.subscribe("panda_camera/depth/points", 1, testCallback);

  ros::spin();	
}
