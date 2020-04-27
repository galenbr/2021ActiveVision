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

    // Adding the table as a collision object
    moveit_msgs::CollisionObject surface;
    surface.header.frame_id = "panda_link0";
    surface.id = "table_surface";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.6;
    primitive.dimensions[1] = 1;
    primitive.dimensions[2] = 0.005;

    geometry_msgs::Pose surface_pose;
    surface_pose.orientation.w = 1;
    surface_pose.position.x = 0.75;
    surface_pose.position.y = 0.0;
    surface_pose.position.z = 0.25;

    surface.primitives.push_back(primitive);
    surface.primitive_poses.push_back(surface_pose);
    surface.operation = surface.ADD;

    // Sending objects to client
    collisionObj.request.collObject = surface;
    addCollisionClient.call(collisionObj);

    ros::spin();
    
    return 0;
}

