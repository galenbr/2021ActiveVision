#include <ros/ros.h>
#include <moveit_msgs/JointConstraint.h>
#include <moveit_msgs/Constraints.h>

int main(int argc, char **argv){

ros::init(argc, argv, "arm_constraints");
ros::NodeHandle n;

moveit_msgs::JointConstraint jc;
    jc.position = -0.765;
    jc.tolerance_above = 3.14159;
    jc.tolerance_below = 0.4;
    jc.joint_name = "panda_joint2";
    jc.weight = 1;

    moveit_msgs::Constraints cons;
    cons.joint_constraints.push_back(jc);
    // moveit::planning_interface::MoveGroupInterface moveGroup;
    // moveGroup.setPathConstraints(cons);



    return 0;
}