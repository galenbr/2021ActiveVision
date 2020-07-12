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

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
ros::NodeHandle *nhptr;

using namespace std;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
bool isservice=false;
Eigen::Quaterniond q2;
ros::Publisher image_arr_pub;
// std_msgs::Int64 index_ros;
long int index_1;
long int index_2;


pcl::PointCloud<PointT>::Ptr updatedPCL(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointT>::Ptr storedPCL(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointT>::Ptr storedPCL_new(new pcl::PointCloud<pcl::PointXYZ>);
Eigen::Matrix4f transform_mat;
pcl::PointXYZ minPt, maxPt,minpt_unexplored, maxpt_unexplored;
std::vector<float> camera_origin_coords, robot_feature_vector;

/////////////////////////////

// This is a tutorial so we can afford having global variables  
	//our visualizer
pcl::visualization::PCLVisualizer *p;
//viewports
int vp_2;
//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem(1.0,1.0,0,1.1);
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


bool srv_callback(testing_image_transport::image_stitching::Request  &req,testing_image_transport::image_stitching::Response &res)
{
  index_1 = req.a;
 	index_2 = req.c;

  // int index = req.a;
  // index_ros.data = req.a;	
  // index = index_ros.data;
  isservice = true;
  
  res.b = "Success!";

  return true;
}	

void calculate_rotation(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  q2.x() = msg->transform.rotation.x;
  q2.y() = msg->transform.rotation.y;
  q2.z() = msg->transform.rotation.z;
  q2.w() = msg->transform.rotation.w;
  float x = msg->transform.translation.x;
  float y = msg->transform.translation.y;
  float z = msg->transform.translation.z;

  Eigen::Matrix3d R = q2.normalized().toRotationMatrix();
  transform_mat = Eigen::Matrix4f::Identity();
  
  transform_mat (0,0) = R (0,0);
  transform_mat (0,1) = R (0,1);
  transform_mat (0,2) = R (0,2);
  transform_mat (1,0) = R (1,0);
  transform_mat (1,1) = R (1,1);
  transform_mat (1,2) = R (1,2);
  transform_mat (2,0) = R (2,0);
  transform_mat (2,1) = R (2,1);
  transform_mat (2,2) = R (2,2);
  transform_mat (0,3) = x; //- 1;
  transform_mat (1,3) = y;
  transform_mat (2,3) = z; //- 0.1;

}

void calculate_camera_coords(const geometry_msgs::Vector3::ConstPtr& msg)
{
  cout<<"Camera coords subscriber called"<<endl;
  
  float x = msg->x;
  float y = msg->y;
  float z = msg->z;

  camera_origin_coords.push_back(x);
  camera_origin_coords.push_back(y);
  camera_origin_coords.push_back(z);
}

void calculate_robot_feature_vector(const geometry_msgs::Vector3::ConstPtr& msg)
{
  cout<<"Robot_Feature_Vector subscriber called"<<endl;
  
  float radius = msg->x;
  float theta = msg->y;
  float phi = msg->z;

  robot_feature_vector.push_back(radius);
  robot_feature_vector.push_back(theta);
  robot_feature_vector.push_back(phi);
}


float * highest_point (const PointCloud::Ptr temp, pcl::PointXYZ minimum, pcl::PointXYZ maximum )
{
  temp->points.resize (temp->width*temp->height);
  float k = (maximum.x - minimum.x)/5;
  float l = (maximum.y - minimum.y)/5;
  float X[6] = {minimum.x, minimum.x + k , minimum.x + 2*k, minimum.x + 3*k, minimum.x + 4*k, maximum.x };
  float Y[6] = {minimum.y, minimum.y + l , minimum.y + 2*l, minimum.y + 3*l, minimum.y + 4*l, maximum.y };
  float x0 = 0.0;
  float x1 = 0.0;
  float y0 = 0.0;
  float y1 = 0.0;
  float highest_z = -100.0;
  float explored_vector [25] = {};
  int j = 0;

  for (int x = 0; x < 5; x++)
  {
    for (int y= 0; y<5; y++)
    {
      x0 = X[x];
      x1 = X[x+1];
      y0 = Y[y];
      y1 = Y[y+1];

      for (int i = 0; i< temp->points.size(); ++i)
      {
        if(temp->points[i].x >= x0 && temp->points[i].x <= x1 && temp->points[i].y >= y0 && temp->points[i].y <= y1)
        {
          if(temp->points[i].z > highest_z)
            {highest_z = temp->points[i].z;}
          explored_vector[j] = highest_z;
        }
      }
      j++;
    }
  }

  std::cout<<"The region 25x1 vector is"<<std::endl;
  for(int i = 0 ; i<25 ; i++)
  {std::cout<<explored_vector[i]<<std::endl;}
  
  return explored_vector;

}

class unexplored_object_model
{
public:
  float c[3];
  float length, breadth, height;
  float resolution; // resolution of point cloud
  PointCloud unexplored_pcd;
  PointCloud::Ptr unexplored_pcd_ptr = unexplored_pcd.makeShared();
  //PointCloud::Ptr unexplored_pcd_ptr (new PointCloud);

  PointCloud cloud_in;
  PointCloud cloud_occluded;
  PointCloud cloud_visible;
  PointCloud::Ptr cloud_in_ptr = cloud_in.makeShared();
  PointCloud::Ptr cloud_occluded_ptr = cloud_occluded.makeShared();
  PointCloud::Ptr cloud_visible_ptr = cloud_visible.makeShared();
  PointCloud::Ptr cloud_clip_ptr = cloud_visible.makeShared();

  unexplored_object_model(const pcl::PointXYZ &minpt, const pcl::PointXYZ &maxpt, 
                          const PointCloud::Ptr &object_cloud)
  {
    //get point cloud
    resolution = 0.008; // 0.008 is approx resolution of the object
    //get_object_point_cloud();
    length = maxpt.x - minpt.x;
    breadth = maxpt.y - minpt.y;
    height = maxpt.z - minpt.z;

    cloud_in_ptr = object_cloud;

    generate_uniform_cloud(minpt, maxpt);
    //generate_spherical_cloud();
  }

  void generate_uniform_cloud(pcl::PointXYZ minPt, pcl::PointXYZ maxPt) // x, y, z
  {
    cout<<"minpt is:::"<<minPt.x<<endl;
    float x = 0, y = 0, z = 0; //1 3D point
    //float i = 0, j = 0, k = 0; // iterators
    int count = 0;

    unexplored_pcd_ptr->width = int(float(length*breadth*height)/float(resolution*resolution*resolution));
    unexplored_pcd_ptr->height = 1;
    unexplored_pcd_ptr->points.resize (unexplored_pcd_ptr->width * unexplored_pcd_ptr->height);

    cout << unexplored_pcd_ptr->size() <<"---------------\n";

    size_t i = 0;

    float scale = 1.0;
    float shift = (scale - 1)*(maxPt.y-minPt.y)/2;
    cout<<"x minpt is "<<minPt.x<<"  max pt is "<<maxPt.x + 2*shift<<endl;
    cout<<"y minpt is "<<minPt.y - shift<<"  max pt is "<<maxPt.y + shift<<endl;
    cout<<"z minpt is "<<minPt.z<<"  max pt is "<<maxPt.z + 2*shift<<endl;

    for(x = minPt.x; x<=maxPt.x + 2*shift; x=x+resolution)
    {
      for(y = minPt.y - shift; y<=maxPt.y + shift; y=y+resolution)
      {
        for(z = minPt.z; z<=maxPt.z + 2*shift; z=z+resolution)
        {
            // Shift the point cloud to center of the object, and append
            if(i < unexplored_pcd_ptr->size()) // or <= ?
            {
              

              unexplored_pcd_ptr->points[i].x = x; 
              unexplored_pcd_ptr->points[i].y = y;
              unexplored_pcd_ptr->points[i].z = z;
            }
            // cout<<"x is"<<x;
            // cout<<"y is"<<y;

            // cout<<"z is"<<z;
            i++;
        }
    }
  }

    cout<<"size: " << unexplored_pcd_ptr->size() << endl;
    //pcl::io::savePCDFileASCII ("unexplored_pcd.pcd", unexplored_pcd);

  }

  void ray_tracing(std::vector<float> &camera_origin)
  {
     Eigen::Vector3f translation;
     translation << 0, 0, 0; 
     Eigen::Affine3f transformation = Eigen::Affine3f::Identity();


    //  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("bowl.pcd", *unexplored_pcd_ptr) == -1) // load the file
    // {
    //   PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    //   return;
    // }

     pcl::CropBox<PointT> cropBoxFilter (false);
     cropBoxFilter.setInputCloud(cloud_in_ptr);
     Eigen::Vector4f min_pt (-5.0f, -0.1f, -0.1f, 1.0f);
     Eigen::Vector4f max_pt (5.0f, 0.1f, 0.1f, 1.0f);

     Eigen::Vector3f v_camera, v_point, axis;
     v_camera << 1, 0, 0;
     //axis << 0, 0, 0;
     std::vector<int> in, out;
     float angle = 0;
    
     for(size_t i = 0; i < unexplored_pcd_ptr->size(); i++)  
     {
        PointT pt=unexplored_pcd_ptr->points[i]; // the cloud here should be unexplored region
        v_point << pt.x - camera_origin[0], pt.y - camera_origin[1], pt.z - camera_origin[2];

        axis << v_camera.cross(v_point);  // get axis with cross product between two vectors
        //cout << "axis" << axis << endl;

        if(v_point.norm() == 0)
        {
          continue;
        }
        else
        {
          angle = acos(v_point.dot(v_camera) / (v_point.norm() * v_camera.norm())); // "division by zero tackled"
        }
        Eigen::Quaterniond q1(cos(angle/2), axis.x()*sin(angle/2), axis.y()*sin(angle/2), axis.z()*sin(angle/2)); //axis angle to quaternion d
        //Eigen::Matrix3f R = Eigen::Matrix3f(q1.normalized().toRotationMatrix());
        //cout << "angle: " << angle << endl;
        q1.normalize();
        //cout << "q1: " << q1.w() << " " <<  q1.x() << " " << q1.y() << " " <<q1.z() << endl;

        Eigen::Vector3f euler_rot;
        
        //translation << v_point.x()/2, v_point.y()/2, v_point.z()/2; // as box center is the origin
        translation << v_point.norm()/2 , 0, 0;

        // Convert quaternion to euler angles 
        Eigen::Vector3d euler = q1.toRotationMatrix().eulerAngles(0, 1, 2);
        //cout << "euler_rot\n"<< euler << endl;
        
        //cout << "translation\n"<< translation << endl;
        
        min_pt << -v_point.norm()/2.0, -resolution/2.0, -resolution/2.0, 1; 
        max_pt << v_point.norm()/2.0, resolution/2.0, resolution/2.0, 1;
        cropBoxFilter.setMin(min_pt);
        cropBoxFilter.setMax(max_pt);
        cropBoxFilter.setTranslation(translation);
        cropBoxFilter.setRotation(euler_rot);


        //cropBoxFilter.setTransform(transformation);

        cropBoxFilter.filter(*cloud_clip_ptr);

        //cout << cloud_clip_ptr->size() << "---- clipped\n";
        if (cloud_clip_ptr->size() > 0)
        {
          cloud_occluded_ptr->push_back(pt);
        }
        else
          cloud_visible_ptr->push_back(pt);

     }

     cout << "cloud_visible: " << cloud_visible_ptr->size() << endl;
     cout << "cloud_occluded: " << cloud_occluded_ptr->size() << endl;
     
  }
  
}; // end of class unexplored_object_model



void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");


  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");

  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");


  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

  p->spinOnce();
}

/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (1);  
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);

  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (5);
  for (int i = 0; i < 50; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      {
        reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
      }

    prev = reg.getLastIncrementalTransformation ();

    // visualize current state
    showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }

	//
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  p->removePointCloud ("source");
  p->removePointCloud ("target");

  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

	//PCL_INFO ("Press q to continue the registration.\n");
  //p->spin ();

  p->removePointCloud ("source"); 
  p->removePointCloud ("target");

  //add the source to the transformed target
  *output += *cloud_src;
  
  final_transform = targetToSource;
 }


void registration(pcl::PointCloud<pcl::PointXYZ>::Ptr newSegPC)
{
  PointCloud::Ptr result (new PointCloud), source, target;
  PointCloud::Ptr exploredpcd_worldframe (new PointCloud);

  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
//updatedPCL = newSegPC;//testing
  if(index_1==2)
  {
  	source = storedPCL;///stored point cloud
  	std::cout<<"Used the stored point cloud" <<std::endl;
  }

    if(index_1==4)
  {
  	source = storedPCL_new;///stored point cloud
  	std::cout<<"Used the stored point cloud" <<std::endl;
  }

  	if(index_1 ==5)
  {
  	PointCloud::Ptr new_source (new PointCloud);
  	source = new_source;
  	std::cout<<"Empty point cloud size is"<<source->size()<<std::endl;
  }

  else
  {
  source = updatedPCL;//needs to be what has been registered already
  }
  
  target = newSegPC;

  PointCloud::Ptr temp (new PointCloud);
  PCL_INFO ("Aligning point clouds.\n");
  pairAlign (source, target, temp, pairTransform, true);

  //transform current pair into the global transform
  pcl::transformPointCloud (*temp, *result, GlobalTransform);

  //update the global transform
  GlobalTransform = GlobalTransform * pairTransform;

  pcl::transformPointCloud (*result,*exploredpcd_worldframe, transform_mat);

  pcl::getMinMax3D (*exploredpcd_worldframe, minPt, maxPt);



  float *explored_haf_pcd;
  explored_haf_pcd = highest_point(exploredpcd_worldframe,minPt,maxPt);

    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;


  unexplored_object_model hidden_model(minPt, maxPt, exploredpcd_worldframe);

  std::vector<float> camera_origin;
 camera_origin.push_back(0.359500); camera_origin.push_back(0.0); camera_origin.push_back(0.643499);

  hidden_model.ray_tracing(camera_origin);

  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(hidden_model.cloud_occluded_ptr);
   while (!viewer.wasStopped ())
   {
   }

  pcl::getMinMax3D (*hidden_model.unexplored_pcd_ptr, minpt_unexplored, maxpt_unexplored);

  float *unexplored_haf_pcd;
  unexplored_haf_pcd = highest_point(hidden_model.unexplored_pcd_ptr,minPt,maxPt);    


  updatedPCL = result;

  if (index_1==1)
  { 
  	storedPCL = updatedPCL;
  	std::cout<<"Stored a point cloud" <<std::endl;
  }

  if (index_1==3)
  {
  	storedPCL_new = updatedPCL;
  	std::cout<<"Stored a point cloud new" <<std::endl;
  }

  if(index_2 == 1)
  {	
  //ros::Publisher image_arr_pub = nhptr->advertise<sensor_msgs::PointCloud2>("image_arr_topic", 1000);
	std::cout<<"Publishing point clout image arr"<<std::endl;
	// sensor_msgs::PointCloud2 temp2;
	// pcl::toROSMsg(*updatedPCL, temp2);
	image_arr_pub.publish(*updatedPCL);
  }
    
    // pcl::transformPointCloud (*result,*result2, tranform_mat);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//filters out the table and just shows the objects on the table
void testCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

  if(isservice == false)
  {
    return;
  }

	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
  //Grab the pcl cloud from ros message
  std::cout<<"---------------- 20 -----------" <<endl;

  pcl::fromROSMsg (*cloud_msg, *cloud);
  std::cout<<"---------------- 21 -----------" <<endl;

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
			//simpleVis(objects);
      std::cout<<"---------------- 22 -----------" <<endl;

      registration(objects);

      std::cout<<"---------------- 23 -----------" <<endl;

		}
		
    else 
			std::cout << "The chosen hull is not planar." << std::endl;

	}

isservice = false;

}


int main (int argc, char** argv)
{
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

  ros::init(argc, argv, "image_listener");

  ros::init(argc, argv, "sub_pcl");

  ros::NodeHandle nh;

  nhptr = &nh;

  ros::Subscriber transformation_sub = nh.subscribe("Transformation",1, calculate_rotation);
  ros::Subscriber camera_origin_coords_sub = nh.subscribe("Camera_Coords",1,calculate_camera_coords);
  // ros::Subscriber robot_feature_vector_sub = nh.subscribe("Robot_Feature_Vector",1,calculate_robot_feature_vector);
  ros::Subscriber subPoints = nh.subscribe("panda_camera/depth/points", 1, testCallback);
  image_arr_pub = nhptr->advertise<sensor_msgs::PointCloud2>("image_arr_topic", 1000);

  ros::ServiceServer service = nh.advertiseService("image_stitching", srv_callback);



//ros::Subscriber movement = nh.subscribe("panda/joint_states",1,testMovement);
  
  ros::spin();
}
 /*]--- */
