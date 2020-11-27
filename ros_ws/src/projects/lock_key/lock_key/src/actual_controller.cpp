#include <ros/ros.h>
#include "moveit_planner/MoveCart.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
    // BEGIN_SUB_TUTORIAL closed_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as closed. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
    // BEGIN_SUB_TUTORIAL open_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    // BEGIN_SUB_TUTORIAL table1
    //
    // Creating Environment
    // ^^^^^^^^^^^^^^^^^^^^
    // Create vector to hold 3 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    collision_objects[1].header.frame_id = "panda_link0";
    collision_objects[1].id = "object";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.02;
    collision_objects[1].primitives[0].dimensions[1] = 0.02;
    collision_objects[1].primitives[0].dimensions[2] = 0.2;

    /* Define the pose of the object. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.5;
    collision_objects[1].primitive_poses[0].position.y = 0;
    collision_objects[1].primitive_poses[0].position.z = 0.5;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[1].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "actual_control");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient cartMoveClient = n.serviceClient<moveit_planner::MoveCart>("cartesian_move");
    ros::service::waitForService("cartesian_move",-1);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    addCollisionObjects(planning_scene_interface);
    ros::WallDuration(1.0).sleep();

    // Move to Key
    moveit_planner::MoveCart move;
    geometry_msgs::Pose p;
    p.orientation.x = 0.83;
    p.orientation.y = -0.55;
    p.orientation.z = 0.01;
    p.orientation.w = -0.01;
    p.position.x = 0.5;
    p.position.y = 0.2;
    p.position.z = 0.1;
    move.request.val.push_back(p);
    move.request.execute = true;
    cartMoveClient.call(move);
    ROS_INFO("Reached key!");

    // Move to Lock
    moveit_planner::MoveCart move2;
    geometry_msgs::Pose p2;
    p2.orientation.x = 0.96;
    p2.orientation.y = -0.26;
    p2.orientation.z = 0.01;
    p2.orientation.w = -0.01;
    p2.position.x = 0.5;
    p2.position.y = -0.2;
    p2.position.z = 0.1;
    move2.request.val.push_back(p2);
    move2.request.execute = true;
    cartMoveClient.call(move2);
    ROS_INFO("Reached lock!");

    // Move to Home (roughly)
    moveit_planner::MoveCart move3;
    geometry_msgs::Pose p3;
    p3.orientation.x = 0.83;
    p3.orientation.y = -0.55;
    p3.orientation.z = 0.01;
    p3.orientation.w = -0.01;
    p3.position.x = 0.3;
    p3.position.y = 0.0;
    p3.position.z = 0.6;
    move3.request.val.push_back(p3);
    move3.request.execute = true;
    cartMoveClient.call(move3);
    ROS_INFO("Reached Home!");

    ros::waitForShutdown();
    return 0;

}