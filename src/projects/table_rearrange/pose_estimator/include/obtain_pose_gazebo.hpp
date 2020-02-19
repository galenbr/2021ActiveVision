#include "ros/ros.h"

#include "pose_estimator/PoseEstimation.h"

#include <vector>
#include <string>

class ObtainPoseGazebo {
public:
  ObtainPoseGazebo(ros::NodeHandle& n) : nh{n} {
    if(!setupParameters())	// Parameters could not be setup, do not spin()
      ROS_ERROR("Failed to setup parameters");
    else {			// Setup remaining services and spin()
      setupServices();
      ros::spin();
    }
  }
  ~ObtainPoseGazebo() {}	// Default destructor

  // Callbacks
  bool poseEstimatorCallback(pose_estimator::PoseEstimation::Request& req,
			     pose_estimator::PoseEstimation::Response& res) {
    // TODO: Estimate and return pose
    return false;		// TEMP
  }

private:
  // NodeHandle
  ros::NodeHandle nh;
  ros::ServiceServer poseEstimatorServer;

  // Setup
  void setupServices() {
    poseEstimatorServer = nh.advertiseService(topic_name,
					      &ObtainPoseGazebo::poseEstimatorCallback,
					      this);
  }
  bool setupParameters() {
    if(!nh.getParam("/table_objects/names", obj_names)) {
      ROS_ERROR("Failed to get /table_objects/names");
      return false;
    }
    if(!nh.getParam("/pose_estimator_topic", topic_name)) {
      ROS_ERROR("Failed to get /pose_estimator_topic");
      return false;
    }

    ROS_INFO_STREAM("Publishing pose_estimation onto " << topic_name);
    return true;
  }

  // Variables
  std::vector<std::string> obj_names;
  std::string topic_name;
};
