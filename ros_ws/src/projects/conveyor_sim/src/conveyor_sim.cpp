#include <ros/ros.h>
#include <string>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

double conveyor_width;
double conveyor_segment_length{0.05}; // Hardcoded
double conveyor_length;
double conveyor_speed;
std::string conveyor_urdf;

int main(int argc, char** argv) {
  ros::init(argc, argv, "conveyor_sim_node");
  ros::NodeHandle nh;

  if(!nh.getParam("/conveyor/width", conveyor_width) ||
     !nh.getParam("/conveyor/length", conveyor_length) ||
     !nh.getParam("/conveyor/speed", conveyor_speed) ||
     !nh.getParam("/conveyor/description",  conveyor_urdf)) {
    ROS_ERROR("Could not load conveyor parameters");
    return 1;
  }

  ros::Duration d{0.1};
  while(!ros::service::exists("/gazebo/spawn_urdf_model", true)) {
    ROS_WARN("Waiting for spawn_urdf_model service");
    d.sleep();
  }

  ros::ServiceClient spawnModelClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

  // Spawn conveyor links
  gazebo_msgs::SpawnModel modelSpawn;
  modelSpawn.request.model_xml = conveyor_urdf;
  modelSpawn.request.initial_pose.orientation.w = 1;
  for(int i = 0; i < (int)(conveyor_length/conveyor_segment_length); ++i) {
    modelSpawn.request.model_name = "segment_" + std::to_string(i);
    modelSpawn.request.initial_pose.position.y += conveyor_segment_length;
    spawnModelClient.call(modelSpawn);
  }

  return 0;
}
