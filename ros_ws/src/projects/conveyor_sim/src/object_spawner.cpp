#include "ros/ros.h"

#include "gazebo_msgs/SpawnModel.h"

double conveyor_width;
double conveyor_length;
double spawn_speed;
int curID(0);

int getID() {
  curID++;
  return (curID - 1);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_spawner_node");
  ros::NodeHandle nh;

  if(!nh.getParam("/conveyor/width", conveyor_width) ||
     !nh.getParam("/spawner/rate", spawn_speed)) {
    ROS_ERROR("Could not get spawner/conveyor params");
    return 1;
  }

  ros::Duration d{0.1};
  while(!ros::service::exists("/gazebo/spawn_urdf_model", true)) {
    ROS_WARN("Waiting for spawn_urdf_model service");
    d.sleep();
  }
  ros::ServiceClient spawnModelClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

  ros::Duration dur{spawn_speed};
  while(ros::ok()) {
    // Spawn an object here

    // Check if any objects are OOB (out of bounds)
    ros::spinOnce();
    dur.sleep();
  }

  return 0;
}
