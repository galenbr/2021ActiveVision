#include "ros/ros.h"
#include <geometry_msgs/WrenchStamped.h>
#include "lock_key/getWrench.h"
#include <vector>
using namespace std;

ros::NodeHandle* n_ptr;
vector<double> forces (3);
vector<double> torques (3);

void wrenchSubCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    ROS_INFO("Received wrist wrench");
    forces[0]=msg->wrench.force.x;
    forces[1]=msg->wrench.force.y;
    forces[2]=msg->wrench.force.z;
    torques[0]=msg->wrench.torque.x;
    torques[1]=msg->wrench.torque.y;
    torques[2]=msg->wrench.torque.z;
}

bool wrenchServiceCallback(lock_key::getWrench::Request &req, 
                           lock_key::getWrench::Response &res){
  ROS_INFO("Getting wrist wrench.");
  res.fx = forces[0];
  res.fy = forces[1];
  res.fz = forces[2];
  res.tx = torques[0];
  res.ty = torques[1];
  res.tz = torques[2];
  ROS_INFO("Fx: %.2f, Fy: %.2f, Fz: %.2f", res.fx, res.fy, res.fz);
  ROS_INFO("Tx: %.2f, Ty: %.2f, Tz: %.2f", res.tx, res.ty, res.tz);
  return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrench_server");
    ros::NodeHandle n;
    n_ptr=&n;

    // Use AsyncSpinner so that wrench values will update
    // during action server execution.
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Subscriber wrench_sub = n.subscribe("panda_joint7_wrench", 1000, wrenchSubCallback);
    ros::ServiceServer wrench_server = n.advertiseService("getWrench", wrenchServiceCallback);


    ros::waitForShutdown();
}