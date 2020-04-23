#include <ros/ros.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "collision_obj");
    ros::NodeHandle n;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(surface);

    planning_scene_interface.addCollisionObjects(collision_objects);

    ros::spin();
    
    return 0;
}

