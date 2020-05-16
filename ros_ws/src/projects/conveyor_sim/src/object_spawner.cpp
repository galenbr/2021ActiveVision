#include "ros/ros.h"

#include "gazebo_msgs/SpawnModel.h"

#include <vector>
#include <string>
#include <time.h>
#include <stdlib.h>

double conveyor_width;
double conveyor_length;
double x, y, z;
double spawn_speed;
int curID{0};

int getID() {
  curID++;
  return (curID - 1);
}

// Attempts to load a parameter from the server, trying several times while waiting in-between
std::string attemptLoad(const std::string& param, ros::NodeHandle& nh, uint attempts, double waitTime, bool& found) {
  found = false;
  ros::Duration d{waitTime};
  std::string ret;
  for(int i = 0; i < attempts; ++i) {
    if(!nh.getParam(param, ret))
      d.sleep();
    else {
      found = true;
      return ret;
    }
  }

  return "";
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_spawner_node");
  ros::NodeHandle nh;

  srand(time(NULL));		// Start rand number generator

  if(!nh.getParam("/conveyor/width", conveyor_width) ||
     !nh.getParam("/spawner/rate", spawn_speed) ||
     !nh.getParam("/conveyor/x", x) ||
     !nh.getParam("/conveyor/y", y) ||
     !nh.getParam("/conveyor/z", z)) {
    ROS_ERROR("Could not get spawner/conveyor params");
    return 1;
  }

  std::vector<std::string> models;
  if(!nh.getParam("/spawner/models", models)) {
    ROS_ERROR("Could not get spawner models");
    return 1;
  }
  if(models.size() == 0) {
    ROS_WARN_STREAM("No objects found in models to spawn, returning");
    return 0;
  }

  ros::Duration d{0.5};
  std::vector<std::string> urdfs;
  for(int i = 0; i < models.size(); ++i) {
    bool found{false};
    std::string tempStr = attemptLoad("/ycb_models/urdfs/" + models[i], nh, 3, 0.5, found);
    if(!found)
      ROS_WARN_STREAM("Could not find " << models[i] << " after " << 3 << " attempts, moving on");
    else
      urdfs.push_back(tempStr);
  }
  if(urdfs.size() == 0) {
    ROS_ERROR("Could not load any model, exiting");
    return 1;
  }

  ros::service::waitForService("/gazebo/spawn_urdf_model");
  ros::ServiceClient spawnModelClient = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

  ros::Rate r{1/spawn_speed};
  int toSpawn{0};
  gazebo_msgs::SpawnModel spawnModelMsg;
  // Values that do not change
  spawnModelMsg.request.initial_pose.position.x = x;
  spawnModelMsg.request.initial_pose.position.y = y + 0.05;
  spawnModelMsg.request.initial_pose.position.z = z + 0.2;
  spawnModelMsg.request.initial_pose.orientation.w = 1;
  while(ros::ok()) {
    int toSpawn = rand() % urdfs.size();
    spawnModelMsg.request.model_name = "spawner_id_" + std::to_string(getID());
    spawnModelMsg.request.model_xml = urdfs[toSpawn];
    if(!spawnModelClient.call(spawnModelMsg))
      ROS_WARN("Could not spawn model");

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
