#include <ros/ros.h>

#include <franka_gripper_gazebo/GripMsg.h>

class FrankaGripper {
public:
  FrankaGripper(ros::NodeHandle& nh);
  ~FrankaGripper();

  bool grip(franka_gripper_gazebo::GripMsg::Request& req,
	    franka_gripper_gazebo::GripMsg::Response& res);
private:
  ros::NodeHandle& _nh;
  
  ros::ServiceServer _grip_server;
  
  ros::Publisher _finger1_publisher;
  ros::Publisher _finger2_publisher;
};
