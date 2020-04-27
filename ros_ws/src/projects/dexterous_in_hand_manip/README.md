# Dexterous In Hand Manipulation
This folder currently contains an implementation of the variable friction gripper to be simulated in gazebo.

## Packages:

### gripper
This package contains:
1. Meshes of the variable friction gripper
2. Descripiton of the variable friction gripper in urdf
3. yaml config file for the finger controllers
4. Launch files for the gripper

### gripper_controls
This package contains low level and high level controllers.

### object_description
This package contains models for the manipulation environment in urdf.

### simulation_plugings
This package contains a gazebo plugin for varying friction in runtime.

## Testing:
1. Set the gazebo plugin path:
  >cd ~/mer_lab/ros_ws/devel/lib

  >export GAZEBO_PLUGIN_PATH=$PWD

2. Launch the variable friction gripper and its controllers in a manipulation environment using the following command:
  >roslaunch gripper manipulation_test.launch

3. Use following rosservices to command the gripper:

  a. Hold_object

  b. Rotate_clockwise

  c. Rotate_anticlockwise

  d. Slide_Right_Finger_Up

  e. Slide_Right_Finger_Down

  f. Slide_Left_Finger_Up

  g. Slide_Left_Finger_Down
