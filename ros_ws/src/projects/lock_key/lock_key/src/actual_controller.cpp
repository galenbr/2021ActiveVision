#include <ros/ros.h>
#include "moveit_planner/MoveCart.h"
#include "moveit_planner/MoveJoint.h"
#include "moveit_planner/SetVelocity.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>
#include <lock_key/SpiralInsertAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef actionlib::SimpleActionClient<lock_key::SpiralInsertAction> SpiralClient;

double x_misalignment{0.0};
double y_misalignment{0.0};

//Defines target end-effector point with offsets
struct point_goal{
    double x{0.0};
    double y{0.0};
    double z{0.0};

    double z_offset_far{0.0};
    double z_offset_close{0.0};
};

//Defines arm position in joint space
struct joint_space_pos{
    double j1{0.0};
    double j2{0.0};
    double j3{0.0};
    double j4{0.0};
    double j5{0.0};
    double j6{0.0};
    double j7{0.0};
};

void moveGripper(double Grasp_Width,double gripper_timout){
    actionlib::SimpleActionClient<franka_gripper::GraspAction> ac("franka_gripper/grasp", true);
    ac.waitForServer();

    franka_gripper::GraspGoal goal;
    goal.width = Grasp_Width;   // Distance between fingers [m]
    goal.speed = 0.1;           // Closing speed. [m/s]
    goal.force = 40;            // Grasping (continuous) force [N]
    goal.epsilon.inner = 0.05;  // Maximum tolerated deviation when the actual grasped width is
                                // smaller than the commanded grasp width.
    goal.epsilon.outer = 0.05;  // Maximum tolerated deviation when the actual grasped width is
                                // larger than the commanded grasp width.
    ac.sendGoal(goal);          // Sending the Grasp command to gripper

    bool finished_before_timeout = ac.waitForResult(ros::Duration(gripper_timout));

    if (finished_before_timeout){
    ROS_INFO("Gripper action finished.");
    }
    else {
    ROS_INFO("Gripper action did not finish before the time out.");
    }
}

// void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
// {
//     // BEGIN_SUB_TUTORIAL table1
//     //
//     // Creating Environment
//     // ^^^^^^^^^^^^^^^^^^^^
//     // Create vector to hold 3 collision objects.
//     std::vector<moveit_msgs::CollisionObject> collision_objects;
//     collision_objects.resize(3);

//     // Add the first table where the cube will originally be kept.
//     collision_objects[0].id = "table1";
//     collision_objects[0].header.frame_id = "panda_link0";

//     /* Define the primitive and its dimensions. */
//     collision_objects[0].primitives.resize(1);
//     collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
//     collision_objects[0].primitives[0].dimensions.resize(3);
//     collision_objects[0].primitives[0].dimensions[0] = 0.2;
//     collision_objects[0].primitives[0].dimensions[1] = 0.4;
//     collision_objects[0].primitives[0].dimensions[2] = 0.4;

//     /* Define the pose of the table. */
//     collision_objects[0].primitive_poses.resize(1);
//     collision_objects[0].primitive_poses[0].position.x = 0.5;
//     collision_objects[0].primitive_poses[0].position.y = 0;
//     collision_objects[0].primitive_poses[0].position.z = 0.2;
//     // END_SUB_TUTORIAL

//     collision_objects[0].operation = collision_objects[0].ADD;

//     // BEGIN_SUB_TUTORIAL object
//     // Define the object that we will be manipulating
//     collision_objects[1].header.frame_id = "panda_link0";
//     collision_objects[1].id = "object";

//     /* Define the primitive and its dimensions. */
//     collision_objects[1].primitives.resize(1);
//     collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
//     collision_objects[1].primitives[0].dimensions.resize(3);
//     collision_objects[1].primitives[0].dimensions[0] = 0.02;
//     collision_objects[1].primitives[0].dimensions[1] = 0.02;
//     collision_objects[1].primitives[0].dimensions[2] = 0.2;

//     /* Define the pose of the object. */
//     collision_objects[1].primitive_poses.resize(1);
//     collision_objects[1].primitive_poses[0].position.x = 0.5;
//     collision_objects[1].primitive_poses[0].position.y = 0;
//     collision_objects[1].primitive_poses[0].position.z = 0.5;
//     // END_SUB_TUTORIAL

//     collision_objects[1].operation = collision_objects[1].ADD;

//     planning_scene_interface.applyCollisionObjects(collision_objects);
// }

int main(int argc, char ** argv){
    ros::init(argc, argv, "actual_control");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    double insert_timout=120.0;

    ros::ServiceClient cartMoveClient = n.serviceClient<moveit_planner::MoveCart>("cartesian_move");
    ros::ServiceClient velScalingClient = n.serviceClient<moveit_planner::SetVelocity>("set_velocity_scaling");
    ros::ServiceClient jointSpaceClient = n.serviceClient<moveit_planner::MoveJoint>("move_to_joint_space");
    ros::service::waitForService("cartesian_move",-1);
    ros::service::waitForService("set_velocity_scaling",-1);
    ros::service::waitForService("move_to_joint_space", -1);
    moveit_planner::SetVelocity velscale;

    //Initialize goal positions
    point_goal key_goal;
    point_goal padlock_goal;
    joint_space_pos home;

    //Retrieve parameters and populate goals
    n.getParam("key_goal/x",key_goal.x);
    n.getParam("key_goal/y",key_goal.y);
    n.getParam("key_goal/z",key_goal.z);
    n.getParam("key_goal/z_offset",key_goal.z_offset_far);

    n.getParam("padlock_goal/x",padlock_goal.x);
    n.getParam("padlock_goal/y",padlock_goal.y);
    n.getParam("padlock_goal/z",padlock_goal.z);
    n.getParam("padlock_goal/z_offset_far",padlock_goal.z_offset_far);
    n.getParam("padlock_goal/z_offset_close",padlock_goal.z_offset_close);
    n.getParam("padlock_goal/x_misalignment",x_misalignment);
    n.getParam("padlock_goal/y_misalignment",y_misalignment);
    // Retrieve Joint Values
    n.getParam("home/j1", home.j1);
    n.getParam("home/j2", home.j2);
    n.getParam("home/j3", home.j3);
    n.getParam("home/j4", home.j4);
    n.getParam("home/j5", home.j5);
    n.getParam("home/j6", home.j6);
    n.getParam("home/j7", home.j7);
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // moveit::planning_interface::MoveGroupInterface group("panda_arm");
    // addCollisionObjects(planning_scene_interface);
    // ros::WallDuration(1.0).sleep();

    // Move to above Key
    moveit_planner::MoveCart move1;
    geometry_msgs::Pose p1;
    p1.orientation.x = 1.0;
    p1.orientation.y = -0.5;
    p1.orientation.z = 0.0;
    p1.orientation.w = 0.0;
    p1.position.x = key_goal.x;
    p1.position.y = key_goal.y;
    p1.position.z = key_goal.z+key_goal.z_offset_far;
    move1.request.val.push_back(p1);
    move1.request.execute = true;
    cartMoveClient.call(move1);
    ROS_INFO("Reached pre-key pose!");

    // Move to Key
    moveit_planner::MoveCart move2;
    geometry_msgs::Pose p2;
    p2.orientation.x = 1.0;
    p2.orientation.y = -0.5;
    p2.orientation.z = 0.0;
    p2.orientation.w = 0.0;
    p2.position.x = p1.position.x;
    p2.position.y = p1.position.y;
    p2.position.z = key_goal.z;
    move2.request.val.push_back(p2);
    move2.request.execute = true;
    cartMoveClient.call(move2);
    ROS_INFO("Reached key!");

    // Begin grasp
    moveGripper(0.02,5.0);

    // Move to above Key after grasping it
    moveit_planner::MoveCart move1a;
    geometry_msgs::Pose p1a;
    p1a.orientation.x = 1.0;
    p1a.orientation.y = -0.5;
    p1a.orientation.z = 0.0;
    p1a.orientation.w = 0.0;
    p1a.position.x = p1.position.x;
    p1a.position.y = p1.position.y;
    p1a.position.z = p1.position.z;
    move1a.request.val.push_back(p1a);
    move1a.request.execute = true;
    cartMoveClient.call(move1a);
    ROS_INFO("Reached post-key pose!");

    // Move to above Lock
    moveit_planner::MoveCart move3;
    geometry_msgs::Pose p3;
    p3.orientation.x = 1.0;
    p3.orientation.y = -0.5;
    p3.orientation.z = 0.0;
    p3.orientation.w = 0.0;
    p3.position.x = padlock_goal.x+x_misalignment;
    p3.position.y = padlock_goal.y+y_misalignment;
    p3.position.z = padlock_goal.z+padlock_goal.z_offset_far;
    move3.request.val.push_back(p3);
    move3.request.execute = true;
    cartMoveClient.call(move3);
    ROS_INFO("Reached far pre-lock pose!");

    // Move to above Lock (but closer)
    moveit_planner::MoveCart move4;
    geometry_msgs::Pose p4;
    p4.orientation.x = 1.0;
    p4.orientation.y = -0.5;
    p4.orientation.z = 0.0;
    p4.orientation.w = 0.0;
    p4.position.x = p3.position.x;
    p4.position.y = p3.position.y;
    p4.position.z = padlock_goal.z+padlock_goal.z_offset_close;
    move4.request.val.push_back(p4);
    move4.request.execute = true;
    cartMoveClient.call(move4);
    ROS_INFO("Reached close pre-lock pose!");

    // reducing velocity
    velscale.request.velScaling = 0.3;
    velScalingClient.call(velscale);

    // Start spiral insert routine
    ROS_INFO("Waiting for Insertion Action Server");
    SpiralClient client("spiral_insert_key", true); // true -> don't need ros::spin()
    client.waitForServer();

    lock_key::SpiralInsertGoal key_spiral_goal;

    ROS_INFO("Retrieving spiral parameters");
    n.getParam("spiral/Ft", key_spiral_goal.Ft);
    n.getParam("spiral/Fd", key_spiral_goal.Fd);
    n.getParam("spiral/Fi", key_spiral_goal.Fi);
    n.getParam("spiral/delta_max", key_spiral_goal.delta_max);

    ROS_INFO("Sending action goal");
    client.sendGoal(key_spiral_goal);

    bool insert_finished_before_timeout = client.waitForResult(ros::Duration(insert_timout));

    if (insert_finished_before_timeout){
        ROS_INFO("Insertion action finished.");
    }
    else {
        ROS_INFO("Insertion action did not finish before the time out.");
    }

    // Release grasp
    moveGripper(0.1,5.0);

    // increasing velocity
    velscale.request.velScaling = 1.0;
    velScalingClient.call(velscale);

    // Move to above Lock after key insertion
    moveit_planner::MoveCart move3a;
    geometry_msgs::Pose p3a;
    p3a.orientation.x = 1.0;
    p3a.orientation.y = -0.5;
    p3a.orientation.z = 0.0;
    p3a.orientation.w = 0.0;
    p3a.position.x = p3.position.x;
    p3a.position.y = p3.position.y;
    p3a.position.z = p3.position.z;
    move3a.request.val.push_back(p3a);
    move3a.request.execute = true;
    cartMoveClient.call(move3a);
    ROS_INFO("Reached post-lock pose!");

    // Move to Home
    moveit_planner::MoveJoint jpos;
    jpos.request.execute = true;
    // Add to request in order
    jpos.request.val.push_back(home.j1);
    jpos.request.val.push_back(home.j2);
    jpos.request.val.push_back(home.j3);
    jpos.request.val.push_back(home.j4);
    jpos.request.val.push_back(home.j5);
    jpos.request.val.push_back(home.j6);
    jpos.request.val.push_back(home.j7);
    // Execute
    jointSpaceClient.call(jpos);
    ROS_INFO("Reached Home!");

    return 0;

}