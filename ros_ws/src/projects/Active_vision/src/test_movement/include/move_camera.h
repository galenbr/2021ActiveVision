#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/kinematic_constraints/utils.h>

#include <Eigen/Dense>
#include "ros/ros.h"
#include <math.h>
#include <testing_image_transport/image_stitching.h>
#include <cstdlib>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Geometry>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelState.h>
#include <std_msgs/Float32MultiArray.h>

#include <random>
#include <memory>


using namespace std;


class move_camera{
public:
  const float rot = M_PI/10; // Angle of rotation for one step
  const int N = 0;
  const int NW = 1;
  const int W = 2;
  const int SW = 3;
  const int S = 4;
  const int SE = 5;
  const int E = 6;
  const int NE = 7;


  Eigen::Vector3d axis_array[8];
  Eigen::Quaterniond q, q2, q_net, q_net_before;
  Eigen::Matrix3d Rot_Mat;

  ros::NodeHandle node_handle;  

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener* tfListener;
  geometry_msgs::TransformStamped transformStamped;

  int dir;

  Eigen::Vector3d v, v_start, axis, v_initial, v_before, v_random;
  std_msgs::Float32MultiArray transformArray;	

  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name of the group you would like to control and plan for.
  //moveit::planning_interface::MoveGroupInterface group_temp;
  moveit::planning_interface::MoveGroupInterface* group;
  
  // Use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
  const robot_state::JointModelGroup* joint_model_group;

  ros::Publisher display_publisher;
  ros::Publisher transformationpub;
  ros::Publisher camera_origin_coords;
  ros::Publisher robot_feature_vector_pub;
  moveit_msgs::DisplayTrajectory display_trajectory;

  robot_state::RobotState* start_state;
 // group.setStartState(start_state);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success;

  geometry_msgs::Pose initial_pose;

  double robot_pos[3];
  double object_pos[3];
  double object_frame[3];
  double end_effector_pos[3];

  geometry_msgs::Pose target_pose2;
  geometry_msgs::Vector3 camera_coords;
  geometry_msgs::Vector3 robot_feature_vector;
  geometry_msgs::Pose pose_to_go;
  geometry_msgs::Pose initial_pose_to_go;
  float radius_sphere_robot; 
  float theta_robot; 
  float phi_robot;

  move_camera();

  ~move_camera(){}

  geometry_msgs::Pose move_camera_to_initialpos();

  int move_camera_to_random_dir(bool is_random, int dir);

  void move_to_initial_pose(geometry_msgs::Pose initial_pose_to_go);

  int move_to_random_pose(geometry_msgs::Pose initial_pose_to_go);

  void robot_to_object_frame(double robot_pos[3], double obj_pos[3], double (&object_frame)[3]);

  void rotate_vector_by_quaternion(Eigen::Vector3d& v, Eigen::Quaterniond& q, Eigen::Vector3d& vprime);

  void rotate(Eigen::Vector3d& v, Eigen::Vector3d& vprime, int dir);
  
  void cartesian_to_spherical(double end_effector_pos[3]);

  int call_image_stitching(ros::NodeHandle node_handle);

  void set_random_object_orientation(geometry_msgs::Pose pose_to_go);

  void publishTransformation(Eigen::Matrix3d R, geometry_msgs::Pose pose1);

  float return_robot_theta();

  float return_robot_phi();

};
