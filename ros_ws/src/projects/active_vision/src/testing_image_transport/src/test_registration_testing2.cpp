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
#include <std_msgs/Float32MultiArray.h>
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
// measure time
#include <chrono>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
ros::NodeHandle *nhptr;

using namespace std;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
bool isservice=false, isGrasping = false;
Eigen::Quaterniond q2;
ros::Publisher image_arr_pub;
ros::Publisher unexplored_pub;

// ros::Publisher exploredpcd_worldframe_pub;
ros::Publisher grasp_quality_pub;
// std_msgs::Int64 index_ros;
long int index_1;
long int index_2;
std::vector<int> rayTracingNo;

pcl::PointCloud<PointT>::Ptr updatedPCL(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointT>::Ptr storedPCL(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointT>::Ptr storedPCL_new_direction(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointT>::Ptr storedPCL_updated_direction(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointT>::Ptr storedPCL_random_iterations(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<PointT>::Ptr unexplored_initial(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointT>::Ptr unexplored_direction(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointT>::Ptr unexplored_direction_updated(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointT>::Ptr unexplored_random_iterations(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointT>::Ptr unexplored_main(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointT>::Ptr unexplored_40(new pcl::PointCloud<pcl::PointXYZ>);

Eigen::Matrix4f transform_mat;
pcl::PointXYZ minPt, maxPt,minpt_unexplored, maxpt_unexplored;
std::vector<float> camera_origin_coords, robot_feature_vector;

std::vector<double> graspPoints;
std::vector<double> maxGraspPoints;
std::vector<double> recorded_maxGraspQuality;
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

pcl::visualization::PCLVisualizer::Ptr normalsVis1 (
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZ> (cloud,"sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr normalsVis (
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr unexplored_cloud,double a, double b, double c, double d, double e, double f)
{

  pcl::PointXYZ point1;
  pcl::PointXYZ point2;
  point1.x = a;
  point1.y = b;
  point1.z = c;
  point2.x = d;
  point2.y = e;
  point2.z = f;


  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(cloud);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(unexplored_cloud);

  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->addPointCloud<pcl::PointXYZRGB> (unexplored_cloud, rgb2, "sample cloud2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 3, 0.002, "normals");
  viewer->addSphere(point1, 0.0015, 0, 0, 255, "sphere1");
  viewer->addSphere(point2, 0.0015, 0, 0, 255, "sphere2");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


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

void calculate_rotation(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  transform_mat (0,0) = msg->data[0];
  transform_mat (0,1) = msg->data[1];
  transform_mat (0,2) = msg->data[2];
  transform_mat (1,0) = msg->data[3];
  transform_mat (1,1) = msg->data[4];
  transform_mat (1,2) = msg->data[5];
  transform_mat (2,0) = msg->data[6];
  transform_mat (2,1) = msg->data[7];
  transform_mat (2,2) = msg->data[8];
  transform_mat (0,3) = msg->data[9]; //- 1;
  transform_mat (1,3) = msg->data[10];
  transform_mat (2,3) = msg->data[11]; //- 0.1;

}

void calculate_camera_coords(const geometry_msgs::Vector3::ConstPtr& msg)
{
  // cout<<"Camera coords subscriber called"<<endl;

  float x = msg->x;
  float y = msg->y;
  float z = msg->z;

  camera_origin_coords.push_back(x);
  camera_origin_coords.push_back(y);
  camera_origin_coords.push_back(z);
}

void calculate_robot_feature_vector(const geometry_msgs::Vector3::ConstPtr& msg)
{
  // cout<<"Robot_Feature_Vector subscriber called"<<endl;

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
  float* explored_vector;
	explored_vector = (float*)malloc(25 * sizeof(float));
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

  // std::cout<<"The region 25x1 vector is"<<std::endl;
  // for(int i = 0 ; i<25 ; i++)
  // {std::cout<<explored_vector[i]<<std::endl;}

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
  PointCloud unexplored_cloud_old;
  PointCloud::Ptr unexplored_cloud_old_ptr = unexplored_cloud_old.makeShared();
  PointCloud::Ptr cloud_in_ptr = cloud_in.makeShared();
  PointCloud::Ptr cloud_occluded_ptr = cloud_occluded.makeShared();
  PointCloud::Ptr cloud_visible_ptr = cloud_visible.makeShared();
  PointCloud::Ptr cloud_clip_ptr = cloud_visible.makeShared();

  unexplored_object_model(const pcl::PointXYZ &minpt, const pcl::PointXYZ &maxpt,
                          const PointCloud::Ptr &object_cloud)
  {
    //get point cloud
    resolution = 0.008; // 0.008 is approx resolution of the object
     cloud_in_ptr = object_cloud;
    generate_uniform_cloud(minpt, maxpt);

    // std::cout<<"Cloud in ptr size is"<<cloud_in_ptr->size()<<std::endl;
  }

  void generate_uniform_cloud(pcl::PointXYZ minPt, pcl::PointXYZ maxPt) // x, y, z
  {
    // cout<<"minpt is:::"<<minPt.x<<endl;
    float x = 0, y = 0, z = 0; //1 3D point
    int count = 0;

    size_t i = 0;

    float scale = 1.5;
    float shift_x = (scale - 1)*(maxPt.x-minPt.x)/2;
    float shift_y = (scale - 1)*(maxPt.y-minPt.y)/2;
    float shift_z = (scale - 1)*(maxPt.z-minPt.z)/2;

    length = maxPt.x + 2*shift_x - minPt.x;
    breadth = maxPt.y + 2*shift_y - minPt.y;
    height = maxPt.z + 2*shift_z - minPt.z;

    unexplored_pcd_ptr->width = int(float(length*breadth*height)/float(resolution*resolution*resolution));
    unexplored_pcd_ptr->height = 1;
    unexplored_pcd_ptr->points.resize (unexplored_pcd_ptr->width * unexplored_pcd_ptr->height);

    // cout << unexplored_pcd_ptr->size() <<"---------------\n";



    // cout<<"x minpt is "<<minPt.x<<"  max pt is "<<maxPt.x + 2*shift_x<<endl;
    // cout<<"y minpt is "<<minPt.y - shift_y<<"  max pt is "<<maxPt.y + shift_y<<endl;
    // cout<<"z minpt is "<<minPt.z<<"  max pt is "<<maxPt.z + 2*shift_z<<endl;


    for(x = minPt.x; x<=maxPt.x + 2*shift_x; x=x+resolution)
    {
      for(y = minPt.y - shift_y; y<=maxPt.y + shift_y; y=y+resolution)
      {
        for(z = minPt.z; z<=maxPt.z + 2*shift_z; z=z+resolution)
        {
            // Shift the point cloud to center of the object, and append
            if(i < unexplored_pcd_ptr->size()) // or <= ?
            {


              unexplored_pcd_ptr->points[i].x = x;
              unexplored_pcd_ptr->points[i].y = y;
              unexplored_pcd_ptr->points[i].z = z;
            }

            i++;
        }
    }
  }

    // cout<<"size: " << unexplored_pcd_ptr->size() << endl;

  }


  void ray_tracing_new(std::vector<float> &camera_origin)
  {

     Eigen::Affine3f transformation = Eigen::Affine3f::Identity();

     pcl::CropBox<PointT> cropBoxFilter;
     cropBoxFilter.setInputCloud(cloud_in_ptr);

     Eigen::Vector3f v_camera, v_point, axis;
     Eigen::Vector3f v_point_norm, v_camera_norm;
     v_camera << 1, 0, 0;
     //axis << 0, 0, 0;
     std::vector<int> in, out;
     float angle = 0;

     for(size_t i = 0; i < unexplored_pcd_ptr->size(); i++)
     {
     	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clip_ptr2(new pcl::PointCloud<pcl::PointXYZ>);
        PointT pt=unexplored_pcd_ptr->points[i]; // the cloud here should be unexplored region
        v_point << pt.x - camera_origin[0], pt.y - camera_origin[1], pt.z - camera_origin[2];
	    v_point_norm << v_point[0]/v_point.norm(),v_point[1]/v_point.norm(),v_point[2]/v_point.norm();
	    v_camera_norm << v_camera[0]/v_camera.norm(),v_camera[1]/v_camera.norm(),v_camera[2]/v_camera.norm();


        axis = v_camera_norm.cross(v_point_norm);  // get axis with cross product between two vectors
	    axis << axis[0]/axis.norm(),axis[1]/axis.norm(),axis[2]/axis.norm();


        if(v_point.norm() == 0)
        {
          continue;
        }
        else
        {
          angle = acos(v_point.dot(v_camera) / (v_point.norm() * v_camera.norm())); // "division by zero tackled"
        }

        Eigen::Quaternionf q1(cos(angle/2), axis.x()*sin(angle/2), axis.y()*sin(angle/2), axis.z()*sin(angle/2)); //axis angle to quaternion d
        q1.normalize();

        Eigen::Vector3f translation(camera_origin[0], camera_origin[1],camera_origin[2]);

        // Convert quaternion to euler angles
        Eigen::Vector3f euler = q1.toRotationMatrix().eulerAngles(0, 1, 2);

       Eigen::Vector4f min_pt (-resolution/2.0, -resolution/2.0, -resolution/2.0, 1);
       Eigen::Vector4f max_pt (v_point.norm()+resolution/2.0, resolution/2.0, resolution/2.0, 1);
        cropBoxFilter.setMin(min_pt);
        cropBoxFilter.setMax(max_pt);
        cropBoxFilter.setTranslation(translation);
        cropBoxFilter.setRotation(euler);


        cropBoxFilter.filter(*cloud_clip_ptr2);

        if (cloud_clip_ptr2->size() > 0)
        {
          cloud_occluded_ptr->push_back(pt);
        }
        else
          cloud_visible_ptr->push_back(pt);

     }

     // cout << "cloud_visible: " << cloud_visible_ptr->size() << endl;
     // cout << "cloud_occluded: " << cloud_occluded_ptr->size() << endl;

  }

   void ray_tracing_old(const PointCloud::Ptr &object_cloud, std::vector<float> &camera_origin)
  {
     unexplored_cloud_old_ptr = object_cloud;
     Eigen::Affine3f transformation = Eigen::Affine3f::Identity();

     pcl::CropBox<PointT> cropBoxFilter;
     cropBoxFilter.setInputCloud(unexplored_cloud_old_ptr);

     Eigen::Vector3f v_camera, v_point, axis;
     Eigen::Vector3f v_point_norm, v_camera_norm;
     v_camera << 1, 0, 0;
     //axis << 0, 0, 0;
     std::vector<int> in, out;
     float angle = 0;

     for(size_t i = 0; i < unexplored_pcd_ptr->size(); i++)
     {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clip_ptr2(new pcl::PointCloud<pcl::PointXYZ>);
        PointT pt=unexplored_pcd_ptr->points[i]; // the cloud here should be unexplored region
        v_point << pt.x - camera_origin[0], pt.y - camera_origin[1], pt.z - camera_origin[2];
      v_point_norm << v_point[0]/v_point.norm(),v_point[1]/v_point.norm(),v_point[2]/v_point.norm();
      v_camera_norm << v_camera[0]/v_camera.norm(),v_camera[1]/v_camera.norm(),v_camera[2]/v_camera.norm();


        axis = v_camera_norm.cross(v_point_norm);  // get axis with cross product between two vectors
      axis << axis[0]/axis.norm(),axis[1]/axis.norm(),axis[2]/axis.norm();


        if(v_point.norm() == 0)
        {
          continue;
        }
        else
        {
          angle = acos(v_point.dot(v_camera) / (v_point.norm() * v_camera.norm())); // "division by zero tackled"
        }

        Eigen::Quaternionf q1(cos(angle/2), axis.x()*sin(angle/2), axis.y()*sin(angle/2), axis.z()*sin(angle/2)); //axis angle to quaternion d
        q1.normalize();

        Eigen::Vector3f translation(camera_origin[0], camera_origin[1],camera_origin[2]);

        // Convert quaternion to euler angles
        Eigen::Vector3f euler = q1.toRotationMatrix().eulerAngles(0, 1, 2);

       Eigen::Vector4f min_pt (-resolution/2.0, -resolution/2.0, -resolution/2.0, 1);
       Eigen::Vector4f max_pt (v_point.norm()+resolution/2.0, resolution/2.0, resolution/2.0, 1);
        cropBoxFilter.setMin(min_pt);
        cropBoxFilter.setMax(max_pt);
        cropBoxFilter.setTranslation(translation);
        cropBoxFilter.setRotation(euler);


        cropBoxFilter.filter(*cloud_clip_ptr2);

        if (cloud_clip_ptr2->size() > 0)
        {
          cloud_occluded_ptr->push_back(pt);
        }
        else
          cloud_visible_ptr->push_back(pt);

     }

     // cout << "cloud_visible: " << cloud_visible_ptr->size() << endl;
     // cout << "cloud_occluded: " << cloud_occluded_ptr->size() << endl;

  }

};


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
  reg.setTransformationEpsilon (1e-12);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.0001);
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);

  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (3000);
  for (int i = 0; i < 3000; ++i)
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
        reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.00001);
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

double DistanceCalculation3D(double x1, double x2, double y1, double y2, double z1, double z2)
{
  double x = x2 - x1;
  double y = y2 - y1;
  double z = z2 - z1;

  double sum = x*x + y*y + z*z;
  return std::sqrt(sum);
}

bool CollisionCheck(double point1x, double point1x_normal, double point1y, double point1y_normal, double point1z, double point1z_normal, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_occluded_ptr)
{
 // std::cout<<"Entered collision check hehe"<<std::endl;
 float newPt1x, newPt1y, newPt1z, newPt2x, newPt2y, newPt2z;
 newPt1x = float(point1x + 0.015*point1x_normal);
 newPt1y = float(point1y + 0.015*point1y_normal);
 newPt1z = float(point1z + 0.015*point1z_normal);

  pcl::CropBox<PointT> cropBoxFilter;
  cropBoxFilter.setInputCloud(cloud_occluded_ptr);

  Eigen::Vector3f v_camera, v_point, axis;
  Eigen::Vector3f v_point_norm, v_camera_norm;
  v_camera << 1, 0, 0;
  //axis << 0, 0, 0;
  std::vector<int> in, out;
  float angle = 0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clip_ptr2(new pcl::PointCloud<pcl::PointXYZ>);
  v_point << point1x_normal, point1y_normal, point1z_normal;
  v_point_norm << v_point[0]/v_point.norm(),v_point[1]/v_point.norm(),v_point[2]/v_point.norm();
  v_camera_norm << v_camera[0]/v_camera.norm(),v_camera[1]/v_camera.norm(),v_camera[2]/v_camera.norm();

  axis = v_camera_norm.cross(v_point_norm);  // get axis with cross product between two vectors
  axis << axis[0]/axis.norm(),axis[1]/axis.norm(),axis[2]/axis.norm();

    if(v_point.norm() == 0)
    {
      return false;    }
    else
    {
      angle = acos(v_point.dot(v_camera) / (v_point.norm() * v_camera.norm())); // "division by zero tackled"
    }

    Eigen::Quaternionf q1(cos(angle/2), axis.x()*sin(angle/2), axis.y()*sin(angle/2), axis.z()*sin(angle/2)); //axis angle to quaternion d
    q1.normalize();

    Eigen::Vector3f translation(newPt1x, newPt1y, newPt1z);

    // Convert quaternion to euler angles
    Eigen::Vector3f euler = q1.toRotationMatrix().eulerAngles(0, 1, 2);

   Eigen::Vector4f min_pt (-0.015, -0.015, -0.015, 1);
   Eigen::Vector4f max_pt (0.015, 0.015, 0.015, 1);
    cropBoxFilter.setMin(min_pt);
    cropBoxFilter.setMax(max_pt);
    cropBoxFilter.setTranslation(translation);
    cropBoxFilter.setRotation(euler);


    cropBoxFilter.filter(*cloud_clip_ptr2);

    if (cloud_clip_ptr2->size() > 0)
    {
      return false;
    }
    else
      return true;


}


//needs work... but just trying to get the planning scene to work outside the main... in test collision function
void graspSynthesis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_occluded_ptr)
{

  PointCloud::Ptr src (new PointCloud);
  pcl::VoxelGrid<PointT> grid;

  grid.setLeafSize (0.003, 0.003, 0.003);
  grid.setInputCloud (cloud);
  grid.filter (*src);
  //pcl::io::savePCDFileASCII ("explored_pcd_downsampled.pcd", *src);

  //std::cout<<"Post voxel grid size is: "<<src->size()<<std::endl;

  if (src->size() > 1200 && src->size() < 1800)
  {
  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize (0.01, 0.01, 0.01);
  grid.setInputCloud (cloud);
  PointCloud::Ptr src1 (new PointCloud);
  grid.filter (*src1);
  src = src1;
  std::cout<<"Entered ---1---"<<std::endl;
  }

  if (src->size() > 1800)
  {
  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize (0.01, 0.01, 0.01);
  grid.setInputCloud (cloud);
  PointCloud::Ptr src2 (new PointCloud);
  grid.filter (*src2);
  src = src2;
  std::cout<<"Entered ---2---"<<std::endl;
  }

  std::cout<<"Final size for grasp synthesis is:: "<< src->size()<<std::endl;

  // src = cloud;
  PointCloud::Ptr unexplored_cloud (new PointCloud);
  unexplored_cloud = cloud_occluded_ptr;
  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);

  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  pcl::PointCloud<pcl::PointNormal> dst; // To be created

  // Initialization part
  dst.width = src->width;
  dst.height = src->height;
  dst.is_dense = true;
  dst.points.resize(dst.width * dst.height);
  // std::cout<<"in grasp pcd size is: "<<dst.size()<<std::endl;


    // Assignment part
  for (int i = 0; i < points_with_normals_src->points.size(); i++)
  {
    dst.points[i].x = src->points[i].x;
    dst.points[i].y = src->points[i].y;
    dst.points[i].z = src->points[i].z;

    dst.points[i].curvature = points_with_normals_src->points[i].curvature;

    dst.points[i].normal_x = points_with_normals_src->points[i].normal_x;
    dst.points[i].normal_y = points_with_normals_src->points[i].normal_y;
    dst.points[i].normal_z = points_with_normals_src->points[i].normal_z;
  }



  //---------------------- Grasp Synthesis ------------------------------

  double ax, bx;
  double ay, by;
  double az, bz;
  double ptax, ptay, ptbx, ptby, ptaz, ptbz;
  double aDotab, naDotab, bDotba, nbDotba, magA, magB;
  double angle1_pt1, angle1_pt2, angle2_pt1, angle2_pt2;
  double theta, base, angle;
  double angle1, angle2;
  Eigen::Vector3f a, b;
  string temp;
  //we are going to make a vector of potential grasps.. if it meets our angle threshold we will add it to the vector
  std::vector<moveit_msgs::Grasp> grasps;
  std::vector<double> graspVector;
  moveit_msgs::Grasp testGrasp;

//The maximum gripper width is 80 mm. I went a little under, for padding
  double gripperWidth = 0.2;
  double lowerGripperWidth = 0.02;
  // std::cout << "Checking for grasps"<< std::endl;
  // PointT minPtDst, maxPtDst;
  // pcl::getMinMax3D (src, minPtDst, maxPtDst);

    //int j = 1139;
	// std::cout<<"dst point size is: "<<dst.points.size()<<std::endl;
  std::vector<double> graspQuality;
  // std::vector<double> graspPoints;
  double graspQualityInstance;

  for(int j = 0; j<dst.points.size(); j+=2)
  {
   for (int i = j+1; i < dst.points.size(); i+=2)
   {
           //need to find the angle bewteen the two points
		ax = dst.points[i].normal_x;
		ay = dst.points[i].normal_y;
		az = dst.points[i].normal_z; //- dst.points[i].curvature;

		bx = dst.points[j].normal_x;
		by = dst.points[j].normal_y;
		bz = dst.points[j].normal_z;// - dst.points[j].curvature;

		ptax = dst.points[i].x;
		ptay = dst.points[i].y;
		ptaz = dst.points[i].z;

		ptbx = dst.points[j].x;
		ptby = dst.points[j].y;
		ptbz = dst.points[j].z;

		////// New angle calculation

		double abx, aby, abz; // Vectors from point a to point b
		double bax, bay, baz; // Vectors from point b to point a

		abx = ptbx - ptax;
		aby = ptby - ptay;
		abz = ptbz - ptaz;

		bax = ptax - ptbx;
		bay = ptay - ptby;
		baz = ptaz - ptbz;

		aDotab = ax*abx + ay*aby + az*abz;
		double norm_pta = sqrt(ax*ax+ay*ay+az*az);
		double norm_ab = sqrt(abx*abx + aby*aby + abz*abz);

		naDotab = -ax*abx - ay*aby - az*abz;

		angle1_pt1 = acos(aDotab/(norm_pta * norm_ab));
		angle1_pt1 = angle1_pt1*(180/M_PI);
		angle2_pt1 = acos(naDotab/(norm_pta * norm_ab));
		angle2_pt1 = angle2_pt1*(180/M_PI);

		bDotba = bx*bax + by*bay + bz*baz;
		nbDotba = -bx*bax - by*bay - bz*baz;

		double norm_ptb = sqrt(bx*bx + by*by + bz*bz);
		double norm_ba = sqrt(bax*bax + bay*bay + baz*baz);

		angle1_pt2 = acos(bDotba/(norm_ptb * norm_ba));
		angle1_pt2 = angle1_pt2*(180/M_PI);
		angle2_pt2 = acos(nbDotba/(norm_ptb * norm_ba));
		angle2_pt2 = angle2_pt2*(180/M_PI);

		base = DistanceCalculation3D(ptax, ptbx, ptay, ptby, ptaz, ptbz); // Nicholas calculations

          //the angle threshold that we determine
      if( ((angle1_pt1 < 40) && (angle1_pt2 < 40) && (base < gripperWidth) && (base > lowerGripperWidth)) || ((angle2_pt1 < 40) && (angle1_pt2 < 40) && (base < gripperWidth)&& (base > lowerGripperWidth)) || ((angle1_pt1 < 40) && (angle2_pt2 < 40) && (base < gripperWidth)&& (base > lowerGripperWidth)) || ((angle2_pt1 < 40) && (angle2_pt2 < 40) && (base < gripperWidth)&& (base > lowerGripperWidth)))  // || ((angle > 140) && (angle < 400)))
      {
        if((CollisionCheck(ptax, ax, ptay, ay, ptaz, az, unexplored_cloud) == true) && (CollisionCheck(ptbx, bx, ptby, by, ptbz, bz, unexplored_cloud) == true))
        {
          // std::cout<<"collision check returned true for case 1"<<std::endl;
          if ((angle1_pt1 < 40) && (angle1_pt2 < 40))
          {
            graspQualityInstance = 300 - angle1_pt1 - angle1_pt2;
            graspQuality.push_back(graspQualityInstance);
            graspPoints.push_back(ptax);
            graspPoints.push_back(ptay);
            graspPoints.push_back(ptaz);
            graspPoints.push_back(ptbx);
            graspPoints.push_back(ptby);
            graspPoints.push_back(ptbz);
          }
          else if ((angle2_pt1 < 40) && (angle1_pt2 < 40))
          {
            graspQualityInstance = 300 - angle2_pt1 - angle1_pt2;
            graspQuality.push_back(graspQualityInstance);
            graspPoints.push_back(ptax);
            graspPoints.push_back(ptay);
            graspPoints.push_back(ptaz);
            graspPoints.push_back(ptbx);
            graspPoints.push_back(ptby);
            graspPoints.push_back(ptbz);
          }
          else if ((angle1_pt1 < 40) && (angle2_pt2 < 40))
          {
            graspQualityInstance = 300 - angle1_pt1 - angle2_pt2;
            graspQuality.push_back(graspQualityInstance);
            graspPoints.push_back(ptax);
            graspPoints.push_back(ptay);
            graspPoints.push_back(ptaz);
            graspPoints.push_back(ptbx);
            graspPoints.push_back(ptby);
            graspPoints.push_back(ptbz);
          }
          else if ((angle2_pt1 < 40) && (angle2_pt2 < 40))
          {
            graspQualityInstance = 300 - angle2_pt1 - angle2_pt2;
            graspQuality.push_back(graspQualityInstance);
            graspPoints.push_back(ptax);
            graspPoints.push_back(ptay);
            graspPoints.push_back(ptaz);
            graspPoints.push_back(ptbx);
            graspPoints.push_back(ptby);
            graspPoints.push_back(ptbz);
          }
        }

        else if((CollisionCheck(ptax, -ax, ptay, -ay, ptaz, -az, unexplored_cloud) == true) && (CollisionCheck(ptbx, bx, ptby, by, ptbz, bz, unexplored_cloud) == true)) // || (angle2 < 40) && (base < gripperWidth))// || ((angle > 130) && (base < gripperWidth) && (angle < 400)))
        {
          // std::cout<<"collision check returned true for case 2"<<std::endl;
          if ((angle1_pt1 < 40) && (angle1_pt2 < 40))
          {
            graspQualityInstance = 300 - angle1_pt1 - angle1_pt2;
            graspQuality.push_back(graspQualityInstance);
            graspPoints.push_back(ptax);
            graspPoints.push_back(ptay);
            graspPoints.push_back(ptaz);
            graspPoints.push_back(ptbx);
            graspPoints.push_back(ptby);
            graspPoints.push_back(ptbz);
          }
          else if ((angle2_pt1 < 40) && (angle1_pt2 < 40))
          {
            graspQualityInstance = 300 - angle2_pt1 - angle1_pt2;
            graspQuality.push_back(graspQualityInstance);
            graspPoints.push_back(ptax);
            graspPoints.push_back(ptay);
            graspPoints.push_back(ptaz);
            graspPoints.push_back(ptbx);
            graspPoints.push_back(ptby);
            graspPoints.push_back(ptbz);
          }
          else if ((angle1_pt1 < 40) && (angle2_pt2 < 40))
          {
            graspQualityInstance = 300 - angle1_pt1 - angle2_pt2;
            graspQuality.push_back(graspQualityInstance);
            graspPoints.push_back(ptax);
            graspPoints.push_back(ptay);
            graspPoints.push_back(ptaz);
            graspPoints.push_back(ptbx);
            graspPoints.push_back(ptby);
            graspPoints.push_back(ptbz);
          }
          else if ((angle2_pt1 < 40) && (angle2_pt2 < 40))
          {
            graspQualityInstance = 300 - angle2_pt1 - angle2_pt2;
            graspQuality.push_back(graspQualityInstance);
            graspPoints.push_back(ptax);
            graspPoints.push_back(ptay);
            graspPoints.push_back(ptaz);
            graspPoints.push_back(ptbx);
            graspPoints.push_back(ptby);
            graspPoints.push_back(ptbz);
          }
        }

        else if((CollisionCheck(ptax, ax, ptay, ay, ptaz, az, unexplored_cloud) == true) && (CollisionCheck(ptbx, -bx, ptby, -by, ptbz, -bz, unexplored_cloud) == true)) // || (angle2 < 40) && (base < gripperWidth))// || ((angle > 130) && (base < gripperWidth) && (angle < 400)))
        {
          // std::cout<<"collision check returned true for case 3"<<std::endl;
          if ((angle1_pt1 < 40) && (angle1_pt2 < 40))
          {
            graspQualityInstance = 300 - angle1_pt1 - angle1_pt2;
            graspQuality.push_back(graspQualityInstance);
            graspPoints.push_back(ptax);
            graspPoints.push_back(ptay);
            graspPoints.push_back(ptaz);
            graspPoints.push_back(ptbx);
            graspPoints.push_back(ptby);
            graspPoints.push_back(ptbz);
          }
          else if ((angle2_pt1 < 40) && (angle1_pt2 < 40))
          {
            graspQualityInstance = 300 - angle2_pt1 - angle1_pt2;
            graspQuality.push_back(graspQualityInstance);
            graspPoints.push_back(ptax);
            graspPoints.push_back(ptay);
            graspPoints.push_back(ptaz);
            graspPoints.push_back(ptbx);
            graspPoints.push_back(ptby);
            graspPoints.push_back(ptbz);
          }
          else if ((angle1_pt1 < 40) && (angle2_pt2 < 40))
          {
            graspQualityInstance = 300 - angle1_pt1 - angle2_pt2;
            graspQuality.push_back(graspQualityInstance);
            graspPoints.push_back(ptax);
            graspPoints.push_back(ptay);
            graspPoints.push_back(ptaz);
            graspPoints.push_back(ptbx);
            graspPoints.push_back(ptby);
            graspPoints.push_back(ptbz);
          }
          else if ((angle2_pt1 < 40) && (angle2_pt2 < 40))
          {
            graspQualityInstance = 300 - angle2_pt1 - angle2_pt2;
            graspQuality.push_back(graspQualityInstance);
            graspPoints.push_back(ptax);
            graspPoints.push_back(ptay);
            graspPoints.push_back(ptaz);
            graspPoints.push_back(ptbx);
            graspPoints.push_back(ptby);
            graspPoints.push_back(ptbz);
          }
        }

         else if((CollisionCheck(ptax, -ax, ptay, -ay, ptaz, -az, unexplored_cloud) == true) && (CollisionCheck(ptbx, -bx, ptby, -by, ptbz, -bz, unexplored_cloud) == true)) // || (angle2 < 40) && (base < gripperWidth))// || ((angle > 130) && (base < gripperWidth) && (angle < 400)))
        {
          // std::cout<<"collision check returned true for case 4"<<std::endl;
          if ((angle1_pt1 < 40) && (angle1_pt2 < 40))
          {
            graspQualityInstance = 300 - angle1_pt1 - angle1_pt2;
            graspQuality.push_back(graspQualityInstance);
            graspPoints.push_back(ptax);
            graspPoints.push_back(ptay);
            graspPoints.push_back(ptaz);
            graspPoints.push_back(ptbx);
            graspPoints.push_back(ptby);
            graspPoints.push_back(ptbz);
          }
          else if ((angle2_pt1 < 40) && (angle1_pt2 < 40))
          {
            graspQualityInstance = 300 - angle2_pt1 - angle1_pt2;
            graspQuality.push_back(graspQualityInstance);
            graspPoints.push_back(ptax);
            graspPoints.push_back(ptay);
            graspPoints.push_back(ptaz);
            graspPoints.push_back(ptbx);
            graspPoints.push_back(ptby);
            graspPoints.push_back(ptbz);
          }
          else if ((angle1_pt1 < 40) && (angle2_pt2 < 40))
          {
            graspQualityInstance = 300 - angle1_pt1 - angle2_pt2;
            graspQuality.push_back(graspQualityInstance);
            graspPoints.push_back(ptax);
            graspPoints.push_back(ptay);
            graspPoints.push_back(ptaz);
            graspPoints.push_back(ptbx);
            graspPoints.push_back(ptby);
            graspPoints.push_back(ptbz);
          }
          else if ((angle2_pt1 < 40) && (angle2_pt2 < 40))
          {
            graspQualityInstance = 300 - angle2_pt1 - angle2_pt2;
            graspQuality.push_back(graspQualityInstance);
            graspPoints.push_back(ptax);
            graspPoints.push_back(ptay);
            graspPoints.push_back(ptaz);
            graspPoints.push_back(ptbx);
            graspPoints.push_back(ptby);
            graspPoints.push_back(ptbz);
          }

        }
          else
            {continue;}

      } // ending if statement
    } // ending smaller for loop
  }  // ending bigger for loop

  if (graspQuality.size() > 0)
  {
    double maxGraspQuality = *max_element(graspQuality.begin(), graspQuality.end());
    int maxDistance = std::distance(graspQuality.begin(), std::max_element(graspQuality.begin(), graspQuality.end()));
    double Point1x = graspPoints[maxDistance*6];
    double Point1y = graspPoints[maxDistance*6 + 1];
    double Point1z = graspPoints[maxDistance*6 + 2];
    double Point2x = graspPoints[maxDistance*6 + 3];
    double Point2y = graspPoints[maxDistance*6 + 4];
    double Point2z = graspPoints[maxDistance*6 + 5];
    recorded_maxGraspQuality.push_back(maxGraspQuality);
    maxGraspPoints.push_back(Point1x);
    maxGraspPoints.push_back(Point1y);
    maxGraspPoints.push_back(Point1z);
    maxGraspPoints.push_back(Point2x);
    maxGraspPoints.push_back(Point2y);
    maxGraspPoints.push_back(Point2z);
    std_msgs::Float64 graspQualityMsg;
    graspQualityMsg.data = maxGraspQuality;
    grasp_quality_pub.publish(graspQualityMsg);

  }
  if (graspQuality.size() == 0)
  {
    std_msgs::Float64 graspQualityMsg;
    graspQualityMsg.data = 0;
    grasp_quality_pub.publish(graspQualityMsg);

  }

  // std::cout<<"Published grasp quality data"<<std::endl;

} // ending grasp synthesis function

void registration(pcl::PointCloud<pcl::PointXYZ>::Ptr newSegPC)
{
  PointCloud::Ptr result (new PointCloud), source, target;
  PointCloud::Ptr exploredpcd_worldframe (new PointCloud);
  PointCloud::Ptr output_cloud (new PointCloud);

  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
  auto t1 = std::chrono::high_resolution_clock::now();
  std::cout<<"index"<<std::endl;
  std::cout << index_1<<std::endl;

  if(index_1==2)
  {
  	source = storedPCL;///stored point cloud
  	// std::cout<<"Used the stored point cloud" <<std::endl;
  }
  else if(index_1==41)
  {
  	source = storedPCL_new_direction;///stored point cloud
  	// std::cout<<"Used the stored point cloud new direction" <<std::endl;
  }
  else if(index_1==42)
  {
    source = storedPCL_updated_direction;///stored point cloud
    // std::cout<<"Used the stored point cloud updated direction" <<std::endl;
  }
  else if(index_1 ==5)
  {
  	PointCloud::Ptr new_source (new PointCloud);
  	updatedPCL = new_source;
  	// std::cout<<"Empty point cloud size is"<<source->size()<<std::endl;
  	return;
  }
  else if(index_1 == 40)
  {
    storedPCL_random_iterations = updatedPCL;
    unexplored_40 = unexplored_main;
    return;
  }
  else if(index_1 == 21)
  {
    source = storedPCL_random_iterations;
  }
  else
  {
  source = updatedPCL;//needs to be what has been registered already
  // cout<<"Chose updatedPCL when it should not have unless case 1"<<std::endl;
  }




  target = newSegPC;

  PointCloud::Ptr temp (new PointCloud);
  PCL_INFO ("Aligning point clouds.\n");



  if (source->size() > 1500 && source->size() < 3000)
    {PointCloud::Ptr src (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize (0.002, 0.002, 0.002);
  grid.setInputCloud (source);
  grid.filter (*src);
  source = src;}
  if (source->size() > 3000 && source->size() < 15000)
    {PointCloud::Ptr src (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize (0.0025, 0.0025, 0.0025);
  grid.setInputCloud (source);
  grid.filter (*src);
  source = src;}
  if (source->size() > 15000 && source->size()<25000)
    {PointCloud::Ptr src (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize (0.003, 0.003, 0.003);
  grid.setInputCloud (source);
  grid.filter (*src);
  source = src;}
  if (source->size() > 25000)
    {PointCloud::Ptr src (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize (0.004, 0.004, 0.004);
  grid.setInputCloud (source);
  grid.filter (*src);
  source = src;}

  pcl::transformPointCloud (*newSegPC,*target, transform_mat);
  exploredpcd_worldframe->width = source->width + target->width;
  exploredpcd_worldframe->height = source->height + target->height;
  exploredpcd_worldframe->resize(exploredpcd_worldframe->width*exploredpcd_worldframe->height);
  *exploredpcd_worldframe = *source + *target;
  pcl::CropBox<pcl::PointXYZ> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(0.2, -10.0, 0.0, 1.0));
  boxFilter.setMax(Eigen::Vector4f(0.6, 3, 1, 1.0));
  boxFilter.setInputCloud(exploredpcd_worldframe);
  boxFilter.filter(*output_cloud);
  updatedPCL = output_cloud;
  exploredpcd_worldframe = updatedPCL;
  //if (index_1 == 1)
  //{pcl::io::savePCDFileASCII ("camera_explored_pcdstiched_ind1_source.pcd", *exploredpcd_worldframe);}

  pcl::getMinMax3D (*exploredpcd_worldframe, minPt, maxPt);


  //float *explored_haf_pcd;
  //explored_haf_pcd = highest_point(exploredpcd_worldframe,minPt,maxPt);
  //int ctr_int = 0;
  //while(ctr_int < 25){
  //  std::cout<<"regis vector check:"<<*(explored_haf_pcd + ctr_int)<<std::endl;
  //  ctr_int += 1;
  //}

  // std::cout<<"explored pcd world frame size is:  "<< exploredpcd_worldframe->size()<<std::endl;
  auto t2 = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  std::cout<<"set grid"<<std::endl;
  std::cout << duration<<std::endl;

  if(index_1 == 3)
  {
    storedPCL_new_direction = updatedPCL;
    // std::cout<<"Stored a point cloud new" <<std::endl;
    return;
  }

  if (index_1 == 1)
  {
  unexplored_object_model hidden_model(minPt, maxPt, exploredpcd_worldframe);
  hidden_model.ray_tracing_new(camera_origin_coords);
  unexplored_initial = hidden_model.cloud_occluded_ptr;
  unexplored_main = unexplored_initial;
  }

  if(index_1 == 2)
  {
  unexplored_object_model hidden_model(minPt, maxPt, exploredpcd_worldframe);
  hidden_model.ray_tracing_old(unexplored_initial, camera_origin_coords);
  unexplored_direction = hidden_model.cloud_occluded_ptr;
  unexplored_main = unexplored_direction;
  }

  if (index_1 == 21)
  {
   unexplored_object_model hidden_model(minPt, maxPt, exploredpcd_worldframe);
  hidden_model.ray_tracing_old(unexplored_40, camera_origin_coords);
  unexplored_direction = hidden_model.cloud_occluded_ptr;
  unexplored_main = unexplored_direction;
  }

  if(index_1 == 41)
  {
  unexplored_object_model hidden_model(minPt, maxPt, exploredpcd_worldframe);
  hidden_model.ray_tracing_old(unexplored_direction, camera_origin_coords);
  unexplored_direction_updated = hidden_model.cloud_occluded_ptr;
  unexplored_main = unexplored_direction_updated;
  }


  if(index_1 == 42)
  {
  unexplored_object_model hidden_model(minPt, maxPt, exploredpcd_worldframe);
  hidden_model.ray_tracing_old(unexplored_direction_updated, camera_origin_coords);
  unexplored_direction_updated = hidden_model.cloud_occluded_ptr;
  }

    if(index_1 == 6)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr unexplored_point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    unexplored_point_cloud_ptr->width = unexplored_direction_updated->width;
    unexplored_point_cloud_ptr->height = unexplored_direction_updated->height;

    unexplored_point_cloud_ptr->points.resize(unexplored_direction_updated->width*unexplored_direction_updated->height);

    for(int i =0; i<unexplored_direction_updated->size();i++)
    {
    unexplored_point_cloud_ptr->points[i].x = unexplored_direction_updated->points[i].x;
    unexplored_point_cloud_ptr->points[i].y = unexplored_direction_updated->points[i].y;
    unexplored_point_cloud_ptr->points[i].z = unexplored_direction_updated->points[i].z;
    unexplored_point_cloud_ptr->points[i].r = 255;
    unexplored_point_cloud_ptr->points[i].g = 0;
    unexplored_point_cloud_ptr->points[i].b = 0;

    }

    int finalMaxDistance = std::distance(recorded_maxGraspQuality.begin(), std::max_element(recorded_maxGraspQuality.begin(), recorded_maxGraspQuality.end()));
    double maxPoint1x = maxGraspPoints[finalMaxDistance*6];
    double maxPoint1y = maxGraspPoints[finalMaxDistance*6 + 1];
    double maxPoint1z = maxGraspPoints[finalMaxDistance*6 + 2];
    double maxPoint2x = maxGraspPoints[finalMaxDistance*6 + 3];
    double maxPoint2y = maxGraspPoints[finalMaxDistance*6 + 4];
    double maxPoint2z = maxGraspPoints[finalMaxDistance*6 + 5];
    std::cout<<"point 1x is: "<<maxPoint1x<<std::endl;
    std::cout<<"point 1y is: "<<maxPoint1y<<std::endl;
    std::cout<<"point 1z is: "<<maxPoint1z<<std::endl;
    std::cout<<"point 2x is: "<<maxPoint2x<<std::endl;
    std::cout<<"point 2y is: "<<maxPoint2y<<std::endl;
    std::cout<<"point 2z is: "<<maxPoint2z<<std::endl;

    //pcl::io::savePCDFileASCII ("index6pcdFINAL.pcd", *exploredpcd_worldframe);
    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = normalsVis(exploredpcd_worldframe,unexplored_point_cloud_ptr, maxPoint1x, maxPoint1y, maxPoint1z, maxPoint2x, maxPoint2y, maxPoint2z);
    //viewer->setBackgroundColor(1,1,1);
    while (!viewer->wasStopped ())
   {
    // viewer
    viewer->spinOnce (10);
    // std::this_thread::sleep_for(1);
    }
  }

  auto t3 = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>( t3 - t2 ).count();

  std::cout<<"transfer and update PCL"<<std::endl;
  std::cout << duration<<std::endl;
  if (index_1==1)
  {
  	storedPCL = updatedPCL;
  	// std::cout<<"Stored a point cloud" <<std::endl;
  }
    if(index_1 == 41 || index_1 == 42)
  {
    storedPCL_updated_direction = updatedPCL;
    // std::cout<<"Stored a point cloud updated direction" <<std::endl;
  }
  if(index_2 == 1)
  {
  //ros::Publisher image_arr_pub = nhptr->advertise<sensor_msgs::PointCloud2>("image_arr_topic", 1000);
	// std::cout<<"Publishing point clout image arr of size: "<<unexplored_initial->size()<<std::endl;
	// sensor_msgs::PointCloud2 temp2;
	// pcl::toROSMsg(*updatedPCL, temp2);
	image_arr_pub.publish(*updatedPCL);
  unexplored_pub.publish(*unexplored_initial);
  }

  if (index_1 == 40)
  {
    storedPCL_random_iterations = updatedPCL;
    // exploredpcd_worldframe_pub.publish(*exploredpcd_worldframe);
  }
  auto t4 = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>( t4 - t3 ).count();
  std::cout<<"store PCL"<<std::endl;
  std::cout << duration<<std::endl;
  if(index_1 == 1)
    {graspSynthesis(exploredpcd_worldframe, unexplored_initial);}
  if(index_1 == 2)
    {graspSynthesis(exploredpcd_worldframe, unexplored_direction);}
   if(index_1 == 41 || index_1 == 42)
    {graspSynthesis(exploredpcd_worldframe, unexplored_direction_updated);}
    // pcl::transformPointCloud (*result,*result2, tranform_mat);
    auto t5 = std::chrono::high_resolution_clock::now();
  	duration = std::chrono::duration_cast<std::chrono::microseconds>( t5 - t4 ).count();

  	std::cout<<"grasp time"<<std::endl;
  	std::cout << duration<<std::endl;

    //pcl::getMinMax3D (*unexplored_main, minPt, maxPt);
    //float * unexplored_haf_pcd;
    //unexplored_haf_pcd = highest_point(unexplored_main,minPt,maxPt);


}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//filters out the table and just shows the objects on the table
void testCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::fromROSMsg (*cloud_msg, *cloud);
//   return;
// }
  // std::cout<<"---------------- 21 -----
  if(isservice == false)
  {
    return;
  }

	// Objects for storing the point clouds.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr noise_removed_objects(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  //Grab the pcl cloud from ros message
  // std::cout<<"---------------- 40 -----------" <<endl;

  pcl::fromROSMsg (*cloud_msg, *transformed_cloud);

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
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*transformed_cloud, *cloud, transform_1);


  // std::cout<<"---------------- 21 -----------" <<endl;

  // Get the plane model, if present.
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	segmentation.setInputCloud(cloud);
	segmentation.setModelType(pcl::SACMODEL_PLANE);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(0.005);
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
    extract.setNegative(true);
		extract.filter(*plane);

    // std::cout<<"post segmentation size is: "<<plane->size()<<std::endl;

      registration(plane);


	}

isservice = false;

}


int main (int argc, char** argv)
{
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

  ros::init(argc, argv, "image_listener");

  // ros::init(argc, argv, "sub_pcl");

  ros::NodeHandle nh;

  nhptr = &nh;

	image_arr_pub = nhptr->advertise<sensor_msgs::PointCloud2>("image_arr_topic", 1000);
  unexplored_pub = nhptr->advertise<sensor_msgs::PointCloud2>("unexplored_topic",1000);

	// exploredpcd_worldframe_pub = nhptr->advertise<sensor_msgs::PointCloud2>("exploredpcd_worldframe_topic", 1000);
	grasp_quality_pub = nhptr->advertise<std_msgs::Float64>("graspQuality_topic",1000);

  ros::ServiceServer service = nh.advertiseService("image_stitching", srv_callback);

  ros::Subscriber camera_origin_coords_sub = nh.subscribe("Camera_Coords",1000,calculate_camera_coords);

  ros::Subscriber transformation_sub = nh.subscribe("Transformation",1, calculate_rotation);

  // ros::Subscriber robot_feature_vector_sub = nh.subscribe("Robot_Feature_Vector",1,calculate_robot_feature_vector);
  ros::Subscriber subPoints = nh.subscribe("panda_camera/depth/points", 1, testCallback);

  ros::spin();
}
 /*]--- */
