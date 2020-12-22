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

    //Retrieve parameters
    n.getParam("x_misalignment",x_misalignment);
    n.getParam("y_misalignment",y_misalignment);

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
    p1.position.x = 0.5715;
    p1.position.y = 0.2623;
    p1.position.z = 0.1665+0.15;
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
    p2.position.z = 0.1665;
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
    p3.position.x = 0.5715+x_misalignment;
    p3.position.y = 0.1907+y_misalignment;
    p3.position.z = 0.1665+0.15;
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
    p4.position.z = 0.1665+0.03;
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

    lock_key::SpiralInsertGoal key_goal;

    ROS_INFO("Retrieving spiral parameters");
    n.getParam("spiral_Ft", key_goal.Ft);
    n.getParam("spiral_Fd", key_goal.Fd);
    n.getParam("spiral_Fi", key_goal.Fi);
    n.getParam("spiral_delta_max", key_goal.delta_max);

    ROS_INFO("Sending action goal");
    client.sendGoal(key_goal);

    // // Move to Lock
    // moveit_planner::MoveCart move4;
    // geometry_msgs::Pose p4;
    // p4.orientation.x = 1.0;
    // p4.orientation.y = -0.5;
    // p4.orientation.z = 0.0;
    // p4.orientation.w = 0.0;
    // p4.position.x = 0.52;
    // p4.position.y = -0.16;
    // p4.position.z = 0.15;
    // move4.request.val.push_back(p4);
    // move4.request.execute = true;
    // cartMoveClient.call(move4);
    // ROS_INFO("Reached lock!");

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

    // Move to Home (roughly)
    // JOINT SPACE IMPLEMENTATION
    moveit_planner::MoveJoint jpos;
    // panda_joint1 - panda_joint7
    jpos.request.execute = true;
    jpos.request.val.push_back(0.0); //0.34518408463181594
    jpos.request.val.push_back(-0.785); //-0.5886091012133754
    jpos.request.val.push_back(0.0); //-0.23394426883041586
    jpos.request.val.push_back(-2.356); //-2.0712576495897927
    jpos.request.val.push_back(0.0); //-0.1384774200436345
    jpos.request.val.push_back(1.57); //1.5505549706586192
    jpos.request.val.push_back(0.784); //0.7042995433536605
    jointSpaceClient.call(jpos);
    // END - JOINT SPACE IMPLEMENTATION

    // TASK SPACE IMPLEMENTATION
    // moveit_planner::MoveCart move5;
    // geometry_msgs::Pose p5;
    // p5.orientation.x = 0.83;
    // p5.orientation.y = -0.55;
    // p5.orientation.z = 0.01;
    // p5.orientation.w = -0.01;
    // p5.position.x = 0.3;
    // p5.position.y = 0.0;
    // p5.position.z = 0.6;
    // move5.request.val.push_back(p5);
    // move5.request.execute = true;
    // cartMoveClient.call(move5);
    // END - TASK SPACE IMPLEMENTATION
    ROS_INFO("Reached Home!");

    // ros::waitForShutdown();
    return 0;

}