#include <ros/ros.h>
#include "moveit_planner/SetConstraints.h"

int main(int argc, char **argv){

ros::init(argc, argv, "arm_constraints");
ros::NodeHandle n;

ros::ServiceClient constraintClient = n.serviceClient<moveit_planner::SetConstraints>("set_constraints");
moveit_planner::SetConstraints cons ;
moveit_msgs::JointConstraint jc;
    jc.position = 0.0;
    jc.tolerance_above = 1.4;//3.14159;
    jc.tolerance_below = 0.4;
    jc.joint_name = "panda_joint2";
    jc.weight = 1;

    cons.request.constraints.joint_constraints.push_back(jc);

    return 0;
}