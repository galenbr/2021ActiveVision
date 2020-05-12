#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/kinematic_constraints/utils.h>
#include <iostream>
#include <Eigen/Dense>
#include "ros/ros.h"
#include <math.h>
#include <testing_image_transport/image_stitching.h>
#include <cstdlib>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>



#define PI 3.142
#define rot PI/16 // Angle of rotation for one step
#define N 0
#define NW 1
#define W 2
#define SW 3
#define S 4
#define SE 5
#define E 6
#define NE 7


Eigen::Vector3d axis_array[8];
Eigen::Quaterniond q, q2; 

void robot_to_object_frame(double robot_pos[3], double obj_pos[3], double (&object_frame)[3])
{
  for(int i = 0; i < 2; i++)
  {
    object_frame[i] = robot_pos[i] - obj_pos[i];
  }
}

void rotate_vector_by_quaternion(Eigen::Vector3d& v, Eigen::Quaterniond& q, Eigen::Vector3d& vprime)
{
    // Extract the vector part of the quaternion
    Eigen::Vector3d u(q.vec());

    // Extract the scalar part of the quaternion
    float s = q.w();

    // Do the math
    vprime = 2.0f * v.dot(u) * u
          + (s*s - u.dot(u)) * v
          + 2.0f * s * u.cross(v);
}

void rotate(Eigen::Vector3d& v, Eigen::Vector3d& vprime, int dir){
  Eigen::Vector3d axis;
  axis = axis_array[dir];  //axis of rotation

  //axis << 2.0, 0.0, -2.0;
  q.w() = cos(rot/2);
  q.x() = axis.x()*sin(rot/2);
  q.y() = axis.y()*sin(rot/2);
  q.z() = axis.z()*sin(rot/2); //axis angle to quaternion d
  q.normalize(); //Need normalization for correct behaviour
  //std::cout<<q.vec()<<"\n";

  rotate_vector_by_quaternion(v, q, vprime);
  //Update all axes
  for (int i = 0; i<8; i++)
  {
    rotate_vector_by_quaternion(axis_array[i], q, axis_array[i]);
  }
}

int call_image_stitching(ros::NodeHandle node_handle){
    ros::ServiceClient client = node_handle.serviceClient<testing_image_transport::image_stitching>("image_stitching");

    testing_image_transport::image_stitching srv;
    srv.request.a = 1;
    if (client.call(srv))
      {
        ROS_INFO("Successful call to service");
        return 0;
      }
      else
      {
        ROS_ERROR("Failed to call service");
        return 1;
      }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  int dir;
  axis_array[0] << 0, 1, 0; 
  axis_array[1] << 0.0, 0.707, -0.707; 
  axis_array[2] << 0, 0, -1;
  axis_array[3] << 0.0, -0.707, -0.707;
  axis_array[4] << 0.0, -1.0, 0.0;
  axis_array[5] << 0.0, -0.707, 0.707;
  axis_array[6] << 0, 0, 1;
  axis_array[7] << 0.0, 0.707, 0.707;

	Eigen::Vector3d v, v_start, axis;

//
  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface group("arm");
  group.setStartStateToCurrentState();
  // Use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
const robot_state::JointModelGroup* joint_model_group =
    group.getCurrentState()->getJointModelGroup("arm");

  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,true);
  ros::Publisher transformation = node_handle.advertise<geometry_msgs::TransformStamped>("Transformation", 100);
  moveit_msgs::DisplayTrajectory display_trajectory;
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

//ROS_INFO("Starting Current pose x: %f", group.getCurrentPose().pose.position.x);
//ROS_INFO("Starting Current pose y: %f", group.getCurrentPose().pose.position.y);
//ROS_INFO("Starting Current pose z: %f", group.getCurrentPose().pose.position.z);
robot_state::RobotState start_state(*group.getCurrentState());
group.setStartState(start_state);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

  geometry_msgs::Pose initial_pose = group.getCurrentPose(group.getEndEffectorLink().c_str()).pose;

 // group.setPoseTarget(initial_pose);
//success = group.plan(my_plan); 

//clear any constraints before moving to initial position
group.clearPathConstraints();

  initial_pose.orientation.w = -0.011;
  initial_pose.orientation.x= 0.927;
  initial_pose.orientation.y = 0.011;
  initial_pose.orientation.z = 0.374;

  initial_pose.position.x = 0.359500;
  initial_pose.position.y = 0.000000;
  initial_pose.position.z = 0.643499;

  Eigen::Quaterniond q_net(initial_pose.orientation.w, initial_pose.orientation.x, initial_pose.orientation.y, initial_pose.orientation.z);
  q_net.normalize();



  start_state.setFromIK(joint_model_group, initial_pose);
  group.setStartState(start_state);

  group.setPoseTarget(initial_pose);
  success = group.plan(my_plan); 

  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
 
  ROS_INFO("test movement 1; (pose goal) %s",success.val ? "":"FAILED");    

  // Sleep to give Rviz time to visualize the plan. 
   //group.move();	
  ROS_INFO("Starting sleep");
  sleep(2.0);
  ROS_INFO("End sleep");	

  group.execute(my_plan);

  sleep(2.0);
  int image_stitching_return;

  // image_stitching_return = call_image_stitching(node_handle);
  // if (image_stitching_return == 1)
  //   return 0;



  //ROS_INFO("Starting execute sleep");
  //sleep(10.0);
  //ROS_INFO("End execute sleep");
  //ROS_INFO("New Current pose x: %f", group.getCurrentPose().pose.position.x);
  //ROS_INFO("New Current pose y: %f", group.getCurrentPose().pose.position.y);
  //ROS_INFO("New Current pose z: %f", group.getCurrentPose().pose.position.z);

  //======================= Testing constraints =====================================
 /* moveit_msgs::OrientationConstraint ocm;
  geometry_msgs::Quaternion quaternion;
  quaternion.w = 1;
  ocm.link_name = group.getEndEffectorLink().c_str();
  ocm.header.frame_id = "world";
  ocm.orientation = quaternion; //The desired orientation of the robot link specified as a quaternion
  ocm.absolute_x_axis_tolerance = 3.14;
  ocm.absolute_y_axis_tolerance = 3.14;
  ocm.absolute_z_axis_tolerance = 3.14;
  ocm.weight = 1.0;
  //Need to convert to constraint to be able to set it for the group
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  group.setPathConstraints(test_constraints);
*/

  ///////////////////////////////////////////////////////// Added from here
  double robot_pos[] = {0.0, 0.0, 1.0};
  double object_pos[] = {1.0, 0.0, 1.1};
  double object_frame[3];

  robot_to_object_frame(robot_pos, object_pos, object_frame);

  v <<  initial_pose.position.x + object_frame[0], initial_pose.position.y + object_frame[1], initial_pose.position.z + object_frame[2]; // robot frame -> world -> object frame

  //Apply rotation
  v_start << -v.norm(), 0, 0;
  axis = v_start.cross(v);
  float angle = acos(v.dot(v_start) / (v.norm() * v_start.norm()));

  Eigen::Quaterniond q1(cos(angle/2), axis.x()*sin(angle/2), axis.y()*sin(angle/2), axis.z()*sin(angle/2)); //axis angle to quaternion d
  q1.normalize(); //Need normalization for correct behaviour
  for (int i = 0; i<8; i++)
  {
    rotate_vector_by_quaternion(axis_array[i], q1, axis_array[i]);
    //std::cout << axis_array[i]<<"\n -------------\n";
  }

  //Eigen::Quaterniond q(0,0,0.707,0.707);
  

  //base_to_object(v, obj_pos);
  //object_to_base(v, obj_pos);
  //v << -1.0, 0.0, 0.0; //point in the front of the object sphere

  //Eigen::AngleAxisd(PI, 0.0, 0.0, 1.0); //check on this later.

  bool A = tfBuffer.canTransform ("camera_optical_link","panda_link0",ros::Time(0)) ;
  ROS_INFO("Can transform result %d", A);

  
 try {
      //tfListener.waitForTransform("camera_link", "panda_link0",ros::Time::now(), ros::Duration(3.0));
      transformStamped = tfBuffer.lookupTransform("panda_link0", "camera_optical_link", ros::Time(0) + ros::Duration(0));
      std::cout << "The stamped transform is " << transformStamped << std::endl;
      std::cout<<"X translation component is "<<transformStamped.transform.translation.x<<std::endl;
      std::cout<<"x rotation component is "<<transformStamped.transform.rotation.x<<std::endl;
      std::cout<<"w rotation component is "<<transformStamped.transform.rotation.w<<std::endl;
  }
  catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
  }

  transformation.publish(transformStamped);
  q2.x() = transformStamped.transform.rotation.x;
  q2.y() = transformStamped.transform.rotation.y;
  q2.z() = transformStamped.transform.rotation.z;
  q2.w() = transformStamped.transform.rotation.w;

  Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  std::cout << "R=" << std::endl << R << std::endl;


  while(true)
  {     
    std::cout<<"Enter a direction (between 0-7): ";
    std::cin>>dir;
    if (abs(dir)>7)
      {
        std::cout<<"Error"<<std::endl;
        continue;
      }
    rotate(v, v, dir);
    std::cout<<"V:"<<v<<"\n";  

    geometry_msgs::Pose target_pose2;
    //robot_state::RobotState start_state(*group.getCurrentState());

    group.setStartState(start_state);

    q_net = q*q_net;
    q_net.normalize();

    target_pose2.orientation.w = (q_net).w();
    target_pose2.orientation.x= (q_net).x();
    target_pose2.orientation.y = (q_net).y();
    target_pose2.orientation.z = (q_net).z();

    /*
    target_pose2.orientation.w = -0.011;
    target_pose2.orientation.x= 0.927;
    target_pose2.orientation.y = 0.011;
    target_pose2.orientation.z = 0.374;
    */

    target_pose2.position.x = v[0] - object_frame[0];
    target_pose2.position.y = v[1] - object_frame[1];
    target_pose2.position.z = v[2] - object_frame[2];

    start_state.setFromIK(joint_model_group, target_pose2);
    group.setPoseTarget(target_pose2);
    group.setPlanningTime(10.0);

    ROS_INFO("Pose Target x: %f", group.getPoseTarget().pose.position.x);
    ROS_INFO("Pose Target y: %f", group.getPoseTarget().pose.position.y);
    ROS_INFO("Pose Target z: %f", group.getPoseTarget().pose.position.z);


    success = group.plan(my_plan);

    ROS_INFO("test movement 2 (pose goal) %s",success.val ? "":"FAILED");    

    // Sleep to give Rviz time to visualize the plan. 
    //group.move();	
    ROS_INFO("Starting sleep");
    sleep(2.0);
    ROS_INFO("End sleep");	

    group.execute(my_plan);

    sleep(2.0);


bool A = tfBuffer.canTransform ("camera_optical_link","panda_link0",ros::Time(0)) ;
ROS_INFO("Can transform result %d", A);

    
 try {
      //tfListener.waitForTransform("camera_link", "panda_link0",ros::Time::now(), ros::Duration(3.0));
      transformStamped = tfBuffer.lookupTransform("panda_link0", "camera_optical_link", ros::Time(0) + ros::Duration(0));
      std::cout << "The stamped transform is " << transformStamped << std::endl;
  }
  catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
  }

  transformation.publish(transformStamped);
  q2.x() = transformStamped.transform.rotation.x;
  q2.y() = transformStamped.transform.rotation.y;
  q2.z() = transformStamped.transform.rotation.z;
  q2.w() = transformStamped.transform.rotation.w;

  Eigen::Matrix3d R = q2.normalized().toRotationMatrix();
  std::cout << "R=" << std::endl << R << std::endl;


  //   image_stitching_return = call_image_stitching(node_handle);
  // if (image_stitching_return == 1)
  //   return 0;


  }

 //////// Added till here


geometry_msgs::Pose target_pose2;
start_state.setFromIK(joint_model_group, initial_pose);
group.setStartState(start_state);
//group.setStartStateToCurrentState();

  target_pose2.orientation.w = -0.011;
  target_pose2.orientation.x= 0.927;
  target_pose2.orientation.y = 0.011;
  target_pose2.orientation.z = 0.374;

  target_pose2.position.x = 0.369500+0.1;
  target_pose2.position.y = -0.000000;
  target_pose2.position.z = 0.643499-0.2;

group.setPoseTarget(target_pose2);
group.setPlanningTime(20.0);

success = group.plan(my_plan); 

//ROS_INFO("Pose Target x: %f", group.getPoseTarget().pose.position.x);
//ROS_INFO("Pose Target y: %f", group.getPoseTarget().pose.position.y);
//ROS_INFO("Pose Target z: %f", group.getPoseTarget().pose.position.z);

  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
 
  ROS_INFO("test movement 2 (+0.1,0,-0.2) (pose goal) %s",success.val ? "":"FAILED");    

  // Sleep to give Rviz time to visualize the plan. 
 //group.move();	
ROS_INFO("Starting sleep");
sleep(2.0);
ROS_INFO("End sleep");	

group.execute(my_plan);

//ROS_INFO("Starting execute sleep");
//sleep(10.0);
//ROS_INFO("End execute sleep");
//ROS_INFO("Pose Target x: %f", group.getPoseTarget().pose.position.x);
//ROS_INFO("Pose Target y: %f", group.getPoseTarget().pose.position.y);
//ROS_INFO("Pose Target z: %f", group.getPoseTarget().pose.position.z);
//ROS_INFO("New Current pose x: %f", group.getCurrentPose().pose.position.x);
//ROS_INFO("New Current pose y: %f", group.getCurrentPose().pose.position.y);
//ROS_INFO("New Current pose z: %f", group.getCurrentPose().pose.position.z);

  ros::shutdown();  

 return 0;
}

