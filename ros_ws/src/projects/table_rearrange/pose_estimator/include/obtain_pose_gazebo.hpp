#include "ros/ros.h"

#include "gazebo_msgs/ModelStates.h"
#include "pose_estimator/PoseEstimation.h"

#include <vector>
#include <string>

class ObtainPoseGazebo {
public:
  ObtainPoseGazebo(ros::NodeHandle& n) : nh{n}, topic_name{"/pose_estimator/gazebo_pose"} {
    setupTopics();

    publishData();
  }
  ~ObtainPoseGazebo() {}	// Default destructor

  // Callbacks
  void receivedGazeboPoses(gazebo_msgs::ModelStates states) {
    for(int i = 0; i < object_names.size(); ++i)
      for(int j = 0; j < states.name.size(); ++j)
	if(object_names[i] == states.name[j])
	  object_poses[i] = states.pose[j];
  };
  bool getPoses(pose_estimator::PoseEstimation::Request &req,
		pose_estimator::PoseEstimation::Response &res) {
    // Do not need point cloud data
    res.detected_object_names = object_names;
    res.detected_object_poses = object_poses;

    return true;
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber gazeboSub;
  ros::ServiceServer modelPoseServer;
  
  std::string topic_name;
  std::vector<std::string> object_names;
  std::vector<geometry_msgs::Pose> object_poses;

  // Setup
  void setupTopics() {
    nh.getParam("/table_objects", object_names); // Load desired objects from param server
    for(int i = 0; i < object_names.size(); ++i)
      object_poses.push_back(geometry_msgs::Pose{});

    gazeboSub = nh.subscribe("/gazebo/model_states", 1, &ObtainPoseGazebo::receivedGazeboPoses, this);
    modelPoseServer = nh.advertiseService(topic_name, &ObtainPoseGazebo::getPoses, this);
  }

  // Actual loop
  void publishData() {
    ros::Rate r(30);
    while(ros::ok()) {
      ros::spinOnce();
      // Publish pose information
      r.sleep();
    }
  }

};
