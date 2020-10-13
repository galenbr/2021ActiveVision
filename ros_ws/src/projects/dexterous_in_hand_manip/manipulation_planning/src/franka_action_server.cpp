#include <ros/ros.h>
#include <moveit_planner.hpp>
#include <manipulation_planning/ArmPose.h>
class FrankaActions
{
private:
  ros::NodeHandle n_;
  ros::Duration dur;
public:
    bool move_pose(manipulation_planning::ArmPose::Request &req, manipulation_planning::ArmPose::Response &res){
      ros::service::waitForService("move_to_pose");
      ros::ServiceClient client_pose = n_.serviceClient<moveit_planner::MovePose>("move_to_pose");
      moveit_planner::MovePose srv;
      geometry_msgs::Pose goal;
      srv.request.val = req.pose;
      srv.request.execute = 1;
      if (client_pose.call(srv)){
        ROS_INFO("Pose reached");
        return true;
      }
      else{
        ROS_ERROR("Pose failed");
        return false;
      }
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "franka_actions_server");
  ros::NodeHandle n;

  ros::service::waitForService("add_collision_object");
  ros::ServiceClient add_coll = n.serviceClient<moveit_planner::AddCollision>("add_collision_object");
  moveit_planner::AddCollision srv;
  moveit_msgs::CollisionObject table;
  table.header.frame_id = "/world";
  table.id = "table";

    /* A default pose */
  geometry_msgs::Pose pose;
  pose.position.x = 0.8;
  pose.position.y = 0.0;
  pose.position.z = 0.4;
  pose.orientation.w = 1.0;

  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.5;
  primitive.dimensions[1] = 0.5;
  primitive.dimensions[2] = 0.05;

  table.primitives.push_back(primitive);
  table.primitive_poses.push_back(pose);

  srv.request.collObject = table;

  if (add_coll.call(srv)){
    ROS_INFO("Collision added");
  }
  else{
    ROS_ERROR("Collision adding failed");
  }


  FrankaActions franka_acts;
  ros::ServiceServer srv_pose = n.advertiseService("franka_act/move_pose", &FrankaActions::move_pose, &franka_acts);
  ros::spin();
  return 0;
}
