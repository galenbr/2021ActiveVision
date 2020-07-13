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

/////////////////////////////////////////////////////////////////////
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

using namespace std;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
bool isservice=false, isGrasping = false;

ros::NodeHandle*  nh_ptr;
////////////////////////////////////////////////////////////////////////////////
//pcl::PointCloud<pcl::PointXYZ>::Ptr newSegPCL(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr updatedPCL(new pcl::PointCloud<pcl::PointXYZ>);

/////////////////////////////

// This is a tutorial so we can afford having global variables 
  //our visualizer
  pcl::visualization::PCLVisualizer *p;
  //viewports
  int vp_2;

  Eigen::Quaterniond q2;
  Eigen::Matrix4f tranform_mat;
  pcl::PointXYZ minPt, maxPt,minpt_unexplored, maxpt_unexplored;

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

pcl::visualization::PCLVisualizer::Ptr normalsVis (
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
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

  void generate_spherical_cloud()
  {
    // Enter code here...
    //cloud_in.push_back(PointT (4.0, 0.0, 0.0));
    //cloud_in.push_back(PointT (0.5, 0.0, 0.0));
    cloud_in_ptr->width  = 2;
    cloud_in_ptr->height = 1;
    cloud_in_ptr->points.resize (cloud_in_ptr->width * cloud_in_ptr->height);

    cloud_in_ptr->points[0].x = 6.0;
    cloud_in_ptr->points[0].y = 6.0;
    cloud_in_ptr->points[0].z = 6.0;

    cloud_in_ptr->points[1].x = 0.5;
    cloud_in_ptr->points[1].y = 0;
    cloud_in_ptr->points[1].z = 0;

  }

  void get_object_point_cloud()
  {
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("bowl.pcd", *cloud_in_ptr) == -1) // load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return;
    }

    pcl::PointXYZ minpt, maxpt;
    pcl::getMinMax3D (*cloud_in_ptr,minpt,maxpt);

    cout<<"Max X "<<maxpt.x<<endl;
    cout<<"Max Y "<<maxpt.y<<endl;
    cout<<"Max Z "<<maxpt.z<<endl;
    cout<<"Min X "<<minpt.x<<endl;
    cout<<"Min Y "<<minpt.y<<endl;
    cout<<"Min Z "<<minpt.z<<endl;
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


  void ray_tracing1(vector<float> camera_origin)
  {
    //memcpy(cloud_in_ptr, unexplored_pcd_ptr);

    Eigen::Quaternionf quat(1,0,0,0); 
    //unexplored_pcd_ptr->sensor_origin_ = Eigen::Vector4f(0,0,0,1);
    cloud_in_ptr->sensor_origin_ = Eigen::Vector4f(camera_origin[0], camera_origin[1], camera_origin[2], 1);  // set to camera origin
    //unexplored_pcd.sensor_orientation_ = quat;

    pcl::VoxelGridOcclusionEstimation<PointT> voxelFilter; 
    voxelFilter.setInputCloud (cloud_in_ptr); 

    float leaf_size = resolution * 1.0;   // Leaf size should not be less than that in input image
    voxelFilter.setLeafSize (leaf_size, leaf_size, leaf_size); 
    voxelFilter.initializeVoxelGrid(); 

    // estimate the occluded space
    // vector<Eigen::Vector3i > occluded_voxels;
    vector< Eigen::Vector3i, Eigen::aligned_allocator< Eigen::Vector3i > >	occluded_voxels;
    voxelFilter.occlusionEstimationAll (occluded_voxels);
    cout << "occluded voxel length: " << occluded_voxels.size() << endl;

    for (size_t i=0; i<unexplored_pcd_ptr->size(); i++) 
    { 

        PointT pt=unexplored_pcd_ptr->points[i]; 

        Eigen::Vector3i grid_cordinates=voxelFilter.getGridCoordinates (pt.x, pt.y, pt.z); 

        int grid_state; 

        int ret=voxelFilter.occlusionEstimation( grid_state, grid_cordinates ); 
        //cout << "ret: " << ret << endl;

        if (grid_state==1) 
        { 
          cloud_occluded_ptr->push_back(pt); 
        } 
        else 
        { 
          cloud_visible_ptr->push_back(pt); 
          //unexplored_pcd_ptr->points.erase(i);
      //unexplored_pcd_ptr->width--;  // I'm assuming an unorganized pc, so pc.height == 1
      //unexplored_pcd_ptr->points.resize(pointCloud.width)   // actually you don't really need to do this
        } 

    }
    cout << "cloud_visible: " << cloud_visible_ptr->size() << endl;
    cout << "cloud_occluded: " << cloud_occluded_ptr->size() << endl;

  }

  
};



////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
bool srv_callback(testing_image_transport::image_stitching::Request  &req,testing_image_transport::image_stitching::Response &res)
 {
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
  std::cout<<"MSG ROTATION X IS "<<q2.x()<<std::endl;
  std::cout<<"MSG TRANSLATION X IS "<<x<<std::endl;


  Eigen::Matrix3d R = q2.normalized().toRotationMatrix();
  tranform_mat = Eigen::Matrix4f::Identity();
  
  tranform_mat (0,0) = R (0,0);
  tranform_mat (0,1) = R (0,1);
  tranform_mat (0,2) = R (0,2);
  tranform_mat (1,0) = R (1,0);
  tranform_mat (1,1) = R (1,1);
  tranform_mat (1,2) = R (1,2);
  tranform_mat (2,0) = R (2,0);
  tranform_mat (2,1) = R (2,1);
  tranform_mat (2,2) = R (2,2);
  tranform_mat (0,3) = x; //- 1;
  tranform_mat (1,3) = y;
  tranform_mat (2,3) = z; //- 0.1;

  std::cout<<"Transform Matrix NO 1 is:::"<<tranform_mat<<std::endl;



}


void highest_point (const PointCloud::Ptr temp, pcl::PointXYZ minimum, pcl::PointXYZ maximum )
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

}

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
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
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


//need to add orientation but will start with this
bool CollisionCheck(double x, double y, double z, double orientation)
{

  isGrasping = true;


/* Why asyncSpinner is called...
Just as an explanation: MoveGroupInterface::getCurrentPose() relies on ROS spinning being active.
Hence, the spinning needs to be started before this call. 
The only standard way to accomplish this (without blocking) is to use an AsyncSpinner.
 There is no issue in using more than one thread for the asynchronous spinner - 
 despite the fact that message processing order is not guaranteed anymore.
*/
ros::AsyncSpinner spinner(1); spinner.start();

///////////////////////////////// Using a Planning Scene Monitor //////////////////////////////

//Finally got this code up and running, but it seems to always find a successful grasp. 
//Need to test this with a transformed point cloud and see if that works better.
//Note.... with the opml I can get it to work if we switch the solver to speed over distance... working on solving this
const std::string node_name = "panda_link8";
const std::string PLANNING_GROUP = "arm";

moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
group.setStartStateToCurrentState();


robot_model_loader::RobotModelLoaderPtr robot_model_loader(
    new robot_model_loader::RobotModelLoader("robot_description"));
robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();
//Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group
robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));

const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));



 //moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);
group.setStartStateToCurrentState();
robot_state::RobotState start_state(*group.getCurrentState());
//current_State_G = &start_state;

planning_scene->setCurrentState(start_state);
planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(planning_scene, robot_model_loader));

ros::Publisher display_publisher = nh_ptr->advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,true);

//Start Monitors of Planning_Scene_Monitor
    bool use_octomap_monitor = true; // this prevents a /tf warning
    psm->startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,use_octomap_monitor);
    psm->startSceneMonitor();
    psm->startStateMonitor();

while (!psm->getStateMonitor()->haveCompleteState() && ros::ok())
{
  ROS_INFO_STREAM_THROTTLE_NAMED(1, node_name, "Waiting for complete state from topic ");
}

boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
planning_interface::PlannerManagerPtr planner_instance;
std::string planner_plugin_name = "ompl_interface/OMPLPlanner";
//std::string planner_plugin_name = "chomp_interface/CHOMPPlanner";
//this line should look for a planner auotmatically, doesn't seem to work, i believe something else needs to be done 
//if (!nh_ptr->getParam("planning_plugin", planner_plugin_name))
//  ROS_FATAL_STREAM("Could not find planner plugin name");
try
{
  planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
      "moveit_core", "planning_interface::PlannerManager"));
}
catch (pluginlib::PluginlibException& ex)
{
  ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
}
try
{
  planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
  if (!planner_instance->initialize(robot_model, nh_ptr->getNamespace()))
    ROS_FATAL_STREAM("Could not initialize planner instance");
  ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
}
catch (pluginlib::PluginlibException& ex)
{
  const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
  std::stringstream ss;
  for (std::size_t i = 0; i < classes.size(); ++i)
    ss << classes[i] << " ";
  ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                       << "Available plugins: " << ss.str());
}

planning_interface::MotionPlanRequest req;
planning_interface::MotionPlanResponse res;
geometry_msgs::PoseStamped pose;
pose.header.frame_id = "panda_link0";
pose.pose.position.x = x;
pose.pose.position.y = y;
pose.pose.position.z = z;
pose.pose.orientation.w = orientation;

/*
//testing a known successful posistion
pose.pose.orientation.w = -0.011;
  pose.pose.orientation.x= 0.927;
  pose.pose.orientation.y = 0.011;
  pose.pose.orientation.z = 0.374;

  pose.pose.position.x = 0.359500;
  pose.pose.position.y = 0.000000;
  pose.pose.position.z = 0.643499;
*/
/*
//test an unsuccessful position
pose.pose.position.x = 5;
  pose.pose.position.y = 5;
  pose.pose.position.z = 5;
*/


//pose would be so much better but for somereason chomp doesnt work for that... also cant seem to get opml working for that
//still trying to figure out how to get this one to work

  //A tolerance of 0.01 m is specified in position and 0.01 radians in orientation
std::vector<double> tolerance_pose(3, 0.01);
std::vector<double> tolerance_angle(3, 0.01);

moveit_msgs::Constraints pose_goal =
    kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);

/*
robot_state->setFromIK(joint_model_group, pose.pose);
moveit_msgs::Constraints pose_goal =
   kinematic_constraints::constructGoalConstraints(*robot_state, joint_model_group);
*/

req.group_name = PLANNING_GROUP;
req.goal_constraints.push_back(pose_goal);
//req.start_state = group.getCurrentState(true);
moveit::core::robotStateToRobotStateMsg(*group.getCurrentState(), req.start_state );
planning_interface::PlanningContextPtr context =
    planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
context->solve(res);


if (res.error_code_.val != res.error_code_.SUCCESS)
{
  ROS_ERROR("Could not compute plan successfully");
  isGrasping = false;
  return false;
}

isGrasping = false;

return true;
}

double calculateMidpoint(double val1, double val2)
{
  return ((val1+val2)/2);
}
//needs work... but just trying to get the planning scene to work outside the main... in test collision function
float graspSynthesis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

  std::cout<<"entered grasp synthesis function"<<std::endl;

//
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  pcl::VoxelGrid<PointT> grid;

    grid.setLeafSize (0.1, 0.1, 0.1);
    grid.setInputCloud (cloud);
    grid.filter (*src);



  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);
/*  
///////////////////////////////////////////////////////////////////////////////////////////////////////
//visulization
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setKSearch (30);
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    //ne.setRadiusSearch (0.005);
norm_est.setInputCloud (src);
    // Compute the features
    ne.compute (*cloud_normals);

    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*

///////////////////////////////////////////////////////////////////////////////////////
*/
    // combine the point cloud with the normal point cloud

    //pcl::PointCloud<pcl::PointXYZRGB>& src; // Already generated
    pcl::PointCloud<pcl::PointNormal> dst; // To be created

    // Initialization part
    dst.width = src->width;
    dst.height = src->height;
    dst.is_dense = true;
    dst.points.resize(dst.width * dst.height);

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

    //double testValue = cloud_normals->points;
    // std::cout << "normal cloud contains " << cloud_normals->points.size()<< std::endl;
    double ax, bx;
    double ay, by;
    double az, bz;
    double aDotb, magA, magB;
    double theta, base, angle;

    Eigen::Vector3f a, b;
    string temp;
    //we are going to make a vector of potential grasps.. if it meets our angle threshold we will add it to the vector
    std::vector<moveit_msgs::Grasp> grasps;

    moveit_msgs::Grasp testGrasp;

        // visualize normals
  /* pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud1");
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, points_with_normals_src);
*/
/*
pcl::visualization::PCLVisualizer::Ptr viewer;
viewer = normalsVis(cloud, cloud_normals);
while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
   // std::this_thread::sleep_for(1);
  }
*/
    /*viewer.addText3D ("reference", dst.points[1139], 0.003, 1.0, 0.0, 0.0);
    //viewer.addText3D ("p1000", dst.points[1000], 0.003, 0.0, 1.0, 0.0);
    viewer.addText3D ("p2000", dst.points[2000], 0.003, 0.0, 1.0, 0.0);
    viewer.addText3D ("p3000", dst.points[3000], 0.003, 0.0, 1.0, 0.0);
    viewer.addText3D ("p4000", dst.points[4000], 0.003, 0.0, 1.0, 0.0);
    viewer.addText3D ("p5000", dst.points[5000], 0.003, 0.0, 1.0, 0.0);
    viewer.addText3D ("p6000", dst.points[6000], 0.003, 0.0, 1.0, 0.0);
    viewer.addText3D ("p7000", dst.points[7000], 0.003, 0.0, 1.0, 0.0);*/
    //this does not calculate the angle between all points but it is a start to see if the calculation works
//The maximum gripper width is 80 mm. I went a little under, for padding
double gripperWidth = 0.06;
std::cout << "Checking for grasps"<< std::endl;

//For now I am using these to find the highest and lowest values for each axis
double lowX = 0, highX = 0, lowY = 0, highY = 0, lowZ = 0, highZ = 0;  

    //int j = 1139;
    for(int j = 0; j<dst.points.size(); ++j)
    {
      if(dst.points[j].x < lowX)
          lowX = dst.points[j].x;
      if(dst.points[j].x > highX)
          highX = dst.points[j].x;
      if(dst.points[j].y < lowY)
          lowY = dst.points[j].y;
      if(dst.points[j].y > highY)
          highY = dst.points[j].y;
      if(dst.points[j].z < lowZ)
          lowZ = dst.points[j].z;
      if(dst.points[j].z > highZ)
          highZ = dst.points[j].z;
      
      //if(dst.points[j].curvature == 0)
      //{
       for (int i = j+1; i < dst.points.size(); ++i)
       {
        //if(dst.points[i].curvature == 0)
        //{
           //need to find the angle bewteen the two points
          ax = dst.points[j].normal_x;
          ay = dst.points[j].normal_y;
          az = dst.points[j].normal_z - dst.points[j].curvature;
          bx = dst.points[i].normal_x;
          by = dst.points[i].normal_y;
          bz = dst.points[i].normal_z - dst.points[i].curvature;

          aDotb = ax * bx + ay * by + az* bz;
          double d_a = sqrt(ax*ax+ay*ay+az*az);
          double d_b = sqrt(bx*bx+by*by+bz*bz);
          //a <<  ax, ay, az;
        //  b = cloud_normals->points[i];
          theta = acos(aDotb/(d_a * d_b));
          //theta = acos(aDotb/(sqrt(d_a)*sqrt(d_b)));
    //std::cout << "Point #: "<< i << std::endl;
          angle = theta*(180/M_PI);
          base = DistanceCalculation3D(ax, bx, ay, by, az, bz);
          //the angle threshold that we determine
          if((angle < 3) && (base < gripperWidth))
          {


          // temp = "Point "+  i;
          // viewer.addText3D (temp, dst.points[i], 0.002, 0.0, 1.0, 0.0);
           // std::cout << "Points " << i << ": " <<dst.points[j]<<dst.points[i]<< std::endl;
          //  std::cout << "Curvature: " <<cloud_normals->points[i].curvature << std::endl;
            //std::cout << "Points a: (" <<cloud_normals->points[0].normal_x <<", "<<cloud_normals->points[0].normal_y<<", "<<cloud_normals->points[0].normal_z <<")"<< std::endl;
            //std::cout << "Points b: (" <<cloud_normals->points[i].normal_x <<", "<<cloud_normals->points[i].normal_y<<", "<<cloud_normals->points[i].normal_z <<")"<< std::endl;
           // std::cout << "Angle between points: " <<angle<< std::endl;
          //  std::cout << "Distance between points: " <<base<< std::endl;


            // add grasps

            ///////////////////// Testing FCL Collision Checking //////////////////////////////////


                //calculate the position/pose of the hand with the gripper to see if there is a collision
                //Assuming that it will form an isosceles triangle... 
                //  two points are the normal points, the third is the position of the hand... center of hand
                //the base will be the distance between the two points + some padding (safety / size of gripper)
                //    the distance between the points are calculated and the padding is a value that we should assign
                // the height is the distance between the points and the gripper... this should be known


            // testing calculating pose... triangle calculation
            //known values
            double height = 0.058, padding = 0.02;
            //calculated values
            double hyp; 

            hyp = sqrt(height*height + ((base*base)/4));

            // ++count;
            //you would want the height of the gripper to be the same hight as the points... or equally between the points
            //from there you either tilt the arm... or roll the wrist to fine tune the the grip to reach the points
            //For now this only considers a horizontal grab... this is for simplicity sakes to see if it works
            
            ///TODO: change it so... you find the midpoint between the 2 points - the center point of the gripper.... 
            //then the line between the two points should be the orientation of the gripper.
            double cz = (az + bz) / 2;
            double cy = ( (base * base) + (hyp*hyp) - (hyp*hyp) )/(2*base);
            double cx = sqrt((hyp*hyp) - (cy*cy));

            //grasps.resize(count); //I would use pushback but I couldnt get a grasp message to work yet

            testGrasp.grasp_pose.header.frame_id = "panda_link8"; //issue with this line

            tf2::Quaternion orientation;
            orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
            testGrasp.grasp_pose.pose.orientation = tf2::toMsg(orientation);
            testGrasp.grasp_pose.pose.position.x = cx;
            testGrasp.grasp_pose.pose.position.y = cy;
            testGrasp.grasp_pose.pose.position.z = cz;

            //Need to open gripper (distance between points) + (padding)

            //Check for collision before we add the grasp....
            if(CollisionCheck(cx, cy, cz, testGrasp.grasp_pose.pose.orientation.w) == false)
                continue;
              
            grasps.push_back(testGrasp);


            //generate all the grasps for this particular point, 
            //to generate the other grasps positions from the one position all we should need to do is
            //rotate around the x axis... it only rotates in one direction because it has 2 set points
            //to prove this point, you can grab an object by two points and it can only rotate in one direction  
            double newZ, newY; //x stays the same because we are rotating around that axis
            double rotAngleD = 45;
           /* for(int i = 1; i<8; ++i)
            {
              newY = cy*cos((rotAngleD*i)*(M_PI/180))-cz*sin((rotAngleD*i)*(M_PI/180));
              newZ = cy*sin((rotAngleD*i)*(M_PI/180))+cz*cos((rotAngleD*i)*(M_PI/180));
              //++count;
             // grasps.resize(count);
              testGrasp.grasp_pose.pose.orientation = tf2::toMsg(orientation);//the orientation is probably wrong but will test it later
              testGrasp.grasp_pose.pose.position.x = cx;
              testGrasp.grasp_pose.pose.position.y = newY;
              testGrasp.grasp_pose.pose.position.z = newZ;


              //Check for collision before we add the grasp....


              grasps.push_back(testGrasp);
            }//end of for*/
    ////////////////////////////////////////////////////////////////////////////////////////
          }//end of if
       // }//end of if
      }//end of for
    //}//end of if
  }//end of for
std::cout << "# of grasps: "<< grasps.size() << std::endl;

//This should be a function but for now it is just going to be in here.
/*
to calculate grasp quality there are several things to be
taken into consideration. Each of these should be weighted on the importance,
and in the end make it out of 100

some things to consider:
how close to the center of gravity (center of the object) -The only one I am considering right now...
the orientation of the gripper with regard to the object
The angle between the two gripper points
if possible: how well the gripper points can attach = how much surface area of the grippers are covered


*/
double centerX, centerY, centerZ;
centerX = calculateMidpoint(highX, lowX);
centerY = calculateMidpoint(highY, lowY);
centerZ = calculateMidpoint(highZ, lowZ);

double maxDistance = DistanceCalculation3D(highX, lowX, highY, lowY, highZ, lowZ);
double currentDistance = 0;
double calcTemp;
float graspQuality = 0;
std::cout << "Max Distance: " << maxDistance << std::endl;
//find the closest point to the midpoint
for(int l = 0; l < grasps.size(); ++l)
{
  currentDistance = DistanceCalculation3D(grasps[l].grasp_pose.pose.position.x, centerX, grasps[l].grasp_pose.pose.position.y, centerY, grasps[l].grasp_pose.pose.position.z, centerZ);
  calcTemp = ((currentDistance/maxDistance));
  std::cout << "Current Distance: " << currentDistance << " Calc Quality " << calcTemp << std::endl;
  if(currentDistance > maxDistance)
    graspQuality = 0;
  else if(calcTemp > graspQuality)
      graspQuality = calcTemp;
}

//debugging 
std::cout << "Grasp Quality: " << graspQuality << std::endl;
return graspQuality;
}//end of function



void regisration(pcl::PointCloud<pcl::PointXYZ>::Ptr newSegPC)
{
  PointCloud::Ptr result (new PointCloud), source, target;
  PointCloud::Ptr result2 (new PointCloud);
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
//updatedPCL = newSegPC;//testing
  source = updatedPCL;//needs to be what has been registered already
    target = newSegPC;

    PointCloud::Ptr temp (new PointCloud);
    PCL_INFO ("Aligning point clouds.\n");
    pairAlign (source, target, temp, pairTransform, true);

    //transform current pair into the global transform
    pcl::transformPointCloud (*temp, *result, GlobalTransform);

    //update the global transform
    GlobalTransform = GlobalTransform * pairTransform;

    std::cout<<"The Global Transform is"<<GlobalTransform<<endl;
    std::cout<<"Transform Mat"<<tranform_mat<<std::endl;

    updatedPCL = result;
    pcl::transformPointCloud (*result,*result2, tranform_mat);


    pcl::getMinMax3D (*result2, minPt, maxPt);
    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;

    highest_point(result2,minPt,maxPt);

    cout<<"explored object size is "<<result2->size()<<endl;
  // Visulize uniform point cloud
  cout << "It's starting ---------!" << endl;
  unexplored_object_model hidden_model(minPt, maxPt, result2);
  cout << "done! ------" << endl;
  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  //pcl::visualization::CloudViewer viewer1("Cloud Viewer1");


  std::vector<float> camera_origin;
  camera_origin.push_back(0.359500); camera_origin.push_back(0.0); camera_origin.push_back(1.643499);
  //camera_origin.push_back(-5); camera_origin.push_back(0); camera_origin.push_back(0);
  hidden_model.ray_tracing(camera_origin);
  //cout << hidden_model.cloud_occluded.size() << endl;
  //cout << hidden_model.cloud_visible.size() << endl;
    pcl::getMinMax3D (*hidden_model.unexplored_pcd_ptr, minpt_unexplored, maxpt_unexplored);

  highest_point(hidden_model.unexplored_pcd_ptr,minPt,maxPt);    

  cout << "Visulize!------" << endl;
    viewer.showCloud(hidden_model.cloud_occluded_ptr);
  viewer.showCloud(hidden_model.unexplored_pcd_ptr);
  //  viewer1.showCloud(hidden_model.cloud_visible_ptr);
  while (!viewer.wasStopped ())
   {
   }


    //save aligned pair, transformed into the first cloud's frame
    //std::stringstream ss;
    //ss << i << ".pcd";
    //pcl::io::savePCDFile (ss.str (), *result, true);
   //View the result
    simpleVis(result2);

    graspSynthesis(result);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
//filters out the table and just shows the objects on the table
void testCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

  if(isservice == false)
    {
      return;}

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
      //simpleVis(objects);
                        regisration(objects);
    }
    else 
      std::cout << "The chosen hull is not planar." << std::endl;

  }

isservice = false;





}

/*
void test_registration::regImage(int argc, char** argv)
{
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
  ros::init(argc, argv, "image_listener");
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  
  ros::Subscriber subPoints = nh.subscribe("panda_camera/depth/points", 1, testCallback);
}*/

/* ---[ */
int main (int argc, char** argv)
{
  // Load data
  //std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  //loadData (argc, argv, data);
 // Create a PCLVisualizer object
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

  ros::init(argc, argv, "image_listener");

  ros::init(argc, argv, "sub_pcl");

  ros::NodeHandle nh;
nh_ptr = &nh;
ros::AsyncSpinner spinner(1); spinner.start();
  ros::Subscriber transformation_sub = nh.subscribe("Transformation",1, calculate_rotation);
  
  ros::Subscriber subPoints = nh.subscribe("panda_camera/depth/points", 1, testCallback);

  ros::ServiceServer service = nh.advertiseService("image_stitching", srv_callback);
//ros::Subscriber movement = nh.subscribe("panda/joint_states",1,testMovement);
  ros::waitForShutdown();
  //ros::spin();
}
 /*]--- */
