#include <ros/ros.h>
#include "moveit_planner/AddCollision.h"
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "collision_obj");
    ros::NodeHandle n;

    // Wait for Service
    int32_t timeout = 1000;
    ros::service::waitForService("add_collision_object",timeout);
    
    // Creating Service Client object
    ros::ServiceClient addCollisionClient = n.serviceClient<moveit_planner::AddCollision>("add_collision_object");

    // Creating Message Object
    moveit_planner::AddCollision collisionObj;
    moveit_planner::AddCollision collisionObj2;

    // Adding the padlock as a collision object
    moveit_msgs::CollisionObject padlock;
    padlock.header.frame_id = "panda_link0";
    padlock.id = "padlock";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.08;
    primitive.dimensions[1] = 0.025;
    primitive.dimensions[2] = 0.05;

    geometry_msgs::Pose padlock_pose;
    padlock_pose.orientation.w = 1;
    padlock_pose.position.x = 0.61;
    padlock_pose.position.y = 0.0125;
    padlock_pose.position.z = 0.35;

    padlock.primitives.push_back(primitive);
    padlock.primitive_poses.push_back(padlock_pose);
    padlock.operation = padlock.ADD;

    // Sending objects to client
    collisionObj.request.collObject = padlock;
    addCollisionClient.call(collisionObj);

   // Adding the key as a collision object
    moveit_msgs::CollisionObject key;
    key.header.frame_id = "panda_link0";
    key.id = "key";

    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = 0.0145;     
    primitive2.dimensions[1] = 0.0055;
    primitive2.dimensions[2] = 0.06;

    geometry_msgs::Pose key_pose;
    key_pose.orientation.w = 1;
    key_pose.position.x = 0.007250;
    key_pose.position.y = 0.002750;
    key_pose.position.z = 0.10000;

    key.primitives.push_back(primitive);
    key.primitive_poses.push_back(key_pose);
    key.operation = key.ADD;

    // Sending objects to client
    collisionObj2.request.collObject = key;
    addCollisionClient.call(collisionObj2);

    // Adding the table as a collision object
    // moveit_msgs::CollisionObject surface;
    // surface.header.frame_id = "panda_link0";
    // surface.id = "table_surface";

    // shape_msgs::SolidPrimitive primitive;
    // primitive.type = primitive.BOX;
    // primitive.dimensions.resize(3);
    // primitive.dimensions[0] = 0.6;
    // primitive.dimensions[1] = 1;
    // primitive.dimensions[2] = 0.005;

    // geometry_msgs::Pose surface_pose;
    // surface_pose.orientation.w = 1;
    // surface_pose.position.x = 0.75;
    // surface_pose.position.y = 0.0;
    // surface_pose.position.z = 0.4;

    // surface.primitives.push_back(primitive);
    // surface.primitive_poses.push_back(surface_pose);
    // surface.operation = surface.ADD;

    // // Sending objects to client
    // collisionObj.request.collObject = surface;
    // addCollisionClient.call(collisionObj);
    
    return 0;
}

