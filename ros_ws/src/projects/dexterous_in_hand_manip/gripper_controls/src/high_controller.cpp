#include "gripper_controls/Hand.h"
#include <ros/ros.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "High_level_action_server");
  ros::NodeHandle n;
  Hand my_hand;
  ros::ServiceServer service_slfd = n.advertiseService("Slide_Left_Finger_Down", &Hand::slide_left_down, &my_hand);
  ros::ServiceServer service_slfu = n.advertiseService("Slide_Left_Finger_Up", &Hand::slide_left_up, &my_hand);
  ros::ServiceServer service_srfd = n.advertiseService("Slide_Right_Finger_Down", &Hand::slide_right_down, &my_hand);
  ros::ServiceServer service_srfu = n.advertiseService("Slide_Right_Finger_Up", &Hand::slide_right_up, &my_hand);
  ros::ServiceServer service_ranti = n.advertiseService("Rotate_anticlockwise", &Hand::rotate_anticlockwise, &my_hand);
  ros::ServiceServer service_cw = n.advertiseService("Rotate_clockwise", &Hand::rotate_clockwise, &my_hand);
  ros::ServiceServer service_hold = n.advertiseService("Hold_object", &Hand::hold_object, &my_hand);
  ros::spin();
  return 0;
}
