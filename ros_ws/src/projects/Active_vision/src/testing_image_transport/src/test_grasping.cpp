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

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
ros::NodeHandle *nhptr;

using namespace std;
double poseX, poseY, poseZ, OrientationW;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
ros::AsyncSpinner *spinner_ptr;
bool isGrasping = false;
std::vector<double> graspVector;
pcl::PointXYZ minPt, maxPt;

double DistanceCalculation3D(double x1, double x2, double y1, double y2, double z1, double z2)
{
  double x = x2 - x1;
  double y = y2 - y1;
  double z = z2 - z1;

  double sum = x*x + y*y + z*z;
  return std::sqrt(sum);
}

bool CollisionCheck(double point1x, double point1x_normal, double point1y, double point1y_normal, double point1z, double point1z_normal)
{
 float newPt1x, newPt1y, newPt1z, newPt2x, newPt2y, newPt2z;
 newPt1x = float(point1x + 0.02*point1x_normal);
 newPt1y = float(point1y + 0.02*point1y_normal); 
 newPt1z = float(point1z + 0.02*point1z_normal); 

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
      continue;
    }
    else
    {
      angle = acos(v_point.dot(v_camera) / (v_point.norm() * v_camera.norm())); // "division by zero tackled"
    }

    Eigen::Quaternionf q1(cos(angle/2), axis.x()*sin(angle/2), axis.y()*sin(angle/2), axis.z()*sin(angle/2)); //axis angle to quaternion d
    q1.normalize();

    Eigen::Vector3f translation(newPt1x, newPt1y, newPt1z);

    // Convert quaternion to euler angles 
    Eigen::Vector3f euler = q1.toRotationMatrix().eulerAngles(0, 1, 2);

   Eigen::Vector4f min_pt (-0.01, -0.01, -0.01, 1);
   Eigen::Vector4f max_pt (0.01, 0.01, 0.01, 1);
    cropBoxFilter.setMin(min_pt);
    cropBoxFilter.setMax(max_pt);
    cropBoxFilter.setTranslation(translation);
    cropBoxFilter.setRotation(euler);


    cropBoxFilter.filter(*cloud_clip_ptr2);

    if (cloud_clip_ptr2->size() > 0)
    {
      return false
    }
    else
      return true;


}


double calculateMidpoint(double val1, double val2)
{
  return ((val1+val2)/2);
}
//needs work... but just trying to get the planning scene to work outside the main... in test collision function
void graspSynthesis(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{

  std::cout<<"entered grasp synthesis function"<<std::endl;

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
  double ptax, ptay, ptbx, ptby, ptaz, ptbz;
  double aDotb, magA, magB;
  double theta, base, angle;

  Eigen::Vector3f a, b;
  string temp;
  //we are going to make a vector of potential grasps.. if it meets our angle threshold we will add it to the vector
  std::vector<moveit_msgs::Grasp> grasps;

  moveit_msgs::Grasp testGrasp;

//The maximum gripper width is 80 mm. I went a little under, for padding
  double gripperWidth = 0.06;
  std::cout << "Checking for grasps"<< std::endl;

  pcl::getMinMax3D (dst, minPt, maxPt);

    //int j = 1139;
    for(int j = 0; j<dst.points.size(); ++j)
    {
       for (int i = j+1; i < dst.points.size(); ++i)
       {
           //need to find the angle bewteen the two points
          ax = dst.points[j].normal_x;
          ay = dst.points[j].normal_y;
          az = dst.points[j].normal_z; //- dst.points[j].curvature;
          bx = dst.points[i].normal_x;
          by = dst.points[i].normal_y;
          bz = dst.points[i].normal_z;// - dst.points[i].curvature;

          ptax = dst.points[j].x;
          ptay = dst.points[j].y;
          ptaz = dst.points[j].z;
    		  ptbx = dst.points[i].x;
          ptby = dst.points[i].y;
          ptbz = dst.points[i].z;

          aDotb = ax*bx + ay*by + az*bz;
          double d_a = sqrt(ax*ax+ay*ay+az*az);
          double d_b = sqrt(bx*bx+by*by+bz*bz);
          
          theta = acos(aDotb/(d_a * d_b));
          angle1 = theta*(180/M_PI);
          angle2 = 180 - angle1;

          base = DistanceCalculation3D(ptax, ptbx, ptay, ptby, ptaz, ptbz); // Nicholas calculations

          //the angle threshold that we determine
          if((angle1 < 50) && (base < gripperWidth) || (angle2 < 50) && (base < gripperWidth))// || ((angle > 130) && (base < gripperWidth) && (angle < 200))) 
          {

            if((CollisionCheck(ptax, ax, ptay, ay, ptaz, az) == true) && (CollisionCheck(ptbx, bx, ptby, by, ptbz, bz) == true))
              {graspVector.push_back(ptax);
               graspVector.push_back(ptay);
               graspVector.push_back(ptaz);
               graspVector.push_back(ptbx);
               graspVector.push_back(ptby);
               graspVector.push_back(ptbz);}

            else if((CollisionCheck(ptax, -ax, ptay, -ay, ptaz, -az) == true) && (CollisionCheck(ptbx, bx, ptby, by, ptbz, bz) == true)) // || (angle2 < 50) && (base < gripperWidth))// || ((angle > 130) && (base < gripperWidth) && (angle < 200))) 
              {graspVector.push_back(ptax);
               graspVector.push_back(ptay);
               graspVector.push_back(ptaz);
               graspVector.push_back(ptbx);
               graspVector.push_back(ptby);
               graspVector.push_back(ptbz);}

            else if((CollisionCheck(ptax, ax, ptay, ay, ptaz, az) == true) && (CollisionCheck(ptbx, -bx, ptby, -by, ptbz, -bz) == true)) // || (angle2 < 50) && (base < gripperWidth))// || ((angle > 130) && (base < gripperWidth) && (angle < 200))) 
              {graspVector.push_back(ptax);
               graspVector.push_back(ptay);
               graspVector.push_back(ptaz);
               graspVector.push_back(ptbx);
               graspVector.push_back(ptby);
               graspVector.push_back(ptbz);}    

           else if((CollisionCheck(ptax, -ax, ptay, -ay, ptaz, -az) == true) && (CollisionCheck(ptbx, -bx, ptby, -by, ptbz, -bz) == true)) // || (angle2 < 50) && (base < gripperWidth))// || ((angle > 130) && (base < gripperWidth) && (angle < 200))) 
              {graspVector.push_back(ptax);
               graspVector.push_back(ptay);
               graspVector.push_back(ptaz);
               graspVector.push_back(ptbx);
               graspVector.push_back(ptby);
               graspVector.push_back(ptbz);}   

            else
              {continue;}

        }
      }
    }          

    


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

            // double ptcx = (az+bz)/2;
            // double ptcy = (ay+by)/2;
            // double ptcz = (az+bz)/2;

            //grasps.resize(count); //I would use pushback but I couldnt get a grasp message to work yet

            testGrasp.grasp_pose.header.frame_id = "panda_link8"; //issue with this line

            tf2::Quaternion orientation;
            orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
            testGrasp.grasp_pose.pose.orientation = tf2::toMsg(orientation);
            testGrasp.grasp_pose.pose.position.x = cx;
            testGrasp.grasp_pose.pose.position.y = cy;
            testGrasp.grasp_pose.pose.position.z = cz;

            poseX = cx;
            poseY = cy;
            poseZ = cz;
            OrientationW = testGrasp.grasp_pose.pose.orientation.w;
            //Need to open gripper (distance between points) + (padding)
            std::cout<<"Just before collision check"<<std::endl;
            //Check for collision before we add the grasp....
            // if(CollisionCheck(cx, cy, cz, testGrasp.grasp_pose.pose.orientation.w) == false)
            //     continue;
            graspVector.push_back(poseX);
            graspVector.push_back(poseY);
            graspVector.push_back(poseZ);
            graspVector.push_back(OrientationW);

            // std::cout<<"Just after collision check"<<std::endl;

            // grasps.push_back(testGrasp);


            //generate all the grasps for this particular point, 
            //to generate the other grasps positions from the one position all we should need to do is
            //rotate around the x axis... it only rotates in one direction because it has 2 set points
            //to prove this point, you can grab an object by two points and it can only rotate in one direction  

            // double newZ, newY; //x stays the same because we are rotating around that axis
            // double rotAngleD = 45;
           
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

    CheckingForCollisions octomapCollisionCheck;
    int j = 0;
    for(int i = 0; i<graspVector.size()/4; i++)
    {
    bool collisionCheckResult;
    collisionCheckResult = octomapCollisionCheck.CollisionCheck(graspVector[i*4], graspVector[i*4 + 1], graspVector[i*4 + 2], graspVector[i*4 + 3]);
    if (collisionCheckResult == true)
      {
        j++;
        std::cout<<"Number of collision free grasps is"<<j<<std::endl;
      }
    }
// std::cout << "# of grasps: "<< grasps.size() << std::endl;

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
// double centerX, centerY, centerZ;
// centerX = calculateMidpoint(highX, lowX);
// centerY = calculateMidpoint(highY, lowY);
// centerZ = calculateMidpoint(highZ, lowZ);

// // double maxDistance = DistanceCalculation3D(maxPt.x, minPt.x, maxPt.y, minPt.y, maxPt.z, minPt.z);
// double maxDistance = DistanceCalculation3D(highX, lowX, highY, lowY, highZ, lowZ);

// double currentDistance = 0;
// double calcTemp;
// float graspQuality = 0;
// std::cout << "Max Distance: " << maxDistance << std::endl;
// //find the closest point to the midpoint
// for(int l = 0; l < grasps.size(); ++l)
// {
//   currentDistance = DistanceCalculation3D(grasps[l].grasp_pose.pose.position.x, centerX, grasps[l].grasp_pose.pose.position.y, centerY, grasps[l].grasp_pose.pose.position.z, centerZ);
//   calcTemp = ((currentDistance/maxDistance));
//   std::cout << "Current Distance: " << currentDistance << " Calc Quality " << calcTemp << std::endl;
//   if(currentDistance > maxDistance)
//     graspQuality = 0;
//   else if(calcTemp > graspQuality)
//       graspQuality = calcTemp;
// }

// //debugging 
// std::cout << "Grasp Quality: " << graspQuality << std::endl;
// return graspQuality;
}//end of function

int main(int argc, char** argv)
{

   ros::init(argc, argv, "graspSynthesis_listener");

  // ros::init(argc, argv, "sub_pcl");

  ros::NodeHandle nh;

  nhptr = &nh;

  // while(ros::ok())
  // {
  //   ros::AsyncSpinner spinner(1);
  //   spinner_ptr = &spinner;
  //   spinner.start();

    ros::Subscriber graspSynthesis_sub = nh.subscribe("exploredpcd_worldframe_topic", 1, graspSynthesis);
  
  // }

  ros::shutdown();
  // ros::spin();
}

  //subscribe to exploredpcd_worldframe
  // callback would calculate surface normals and then calculate the angle between them
  // callback would later calculate if the angle is below a threshold ; if yes, it'll check for collisions
  // 