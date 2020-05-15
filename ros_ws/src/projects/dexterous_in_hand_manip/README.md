# Dexterous In Hand Manipulation
This project investigates in-hand manipulation strategies based on the integration of a variable friction (VF) hand and Franka Emika Panda arm. This folder currently contains low and high level planning nodes for both Franka arm and variable friction hand.

## Packages:

### arm_controls
This package is used for high level motion planning of the Franka arm. Using the services advertised by the moveit_planner package, high level commands such as rest_pose, initialize_arm_pose, grasp_pose, move_up and move_down are implemented. To run the node, you can either launch arm_controller.launch or include the arm_ctrl node in your project specific launch file.
> This package contains:
> 1. yaml config files for parameters such as, initial, rest and grasp pose
> 2. launch file for arm_ctrl node
> 3. arm_ctrl node
> 4. Readme file for available services

### gripper_controls
This package is based on the motion planning nodes in variable_friction_gripper project, which prepared to be used on hardware. Same logic and functionality is transferred to the gazebo simulation in this package.

Low level controller is based on the class LowLvlController and deals with low level tasks such as receiving and publishing effort, position references, friction values for the left and right fingers.

High level controller is based on the class Hand. It advertises services of high level commands such as, Hold_object, Slide_Left_Finger_Down, Slide_Left_Finger_Up, Slide_Right_Finger_Down, Slide_Right_Finger_Up, Rotate_clockwise, Rotate_anticlockwise, by using the low level commands.

Also a sequential command node is included, that helps testing the services advertised by the high level controller. It is possible to use the torque publisher and a plotting method of your own choice to analyse the gripper behavior during tasks.

high_lvl_ctrl and low_lvl_ctrl nodes can either be run seperately, or you may use the launch file that launches both controllers.
> This package contains:
> 1. yaml config files for low level and high level controller parameters
> 2. launch file for gripper controller
> 3. Implementation of high and low level controllers
> 4. A sequential command node for testing the advertised services
> 4. A torque publisher for analysing the gripper behavior
> 5. Readme file for available services

### manipulation_env
This package is used to generate a manipulation environment. Manipulation environment currently consists of a table and a rectangular block object. Collision characteristics of the block object plays an important role in the manipulation performance. Parameters such as friction coefficients, Kp and Kd can be modified within the corresponding description file.
> This package contains:
> 1. description files for objects
> 2. launch file for launching the integrated arm & gripper in a manipulation environment
> 3. launch file for gripper only (for testing purposes) in a manipulation environment

### manipulation_exp
This package is used for a higher level planner implementation that combines services advertised by arm_controls and gripper_controls into some sequential manipulation strategies.
> This package contains:
> 1. Implementation of the ManipulationSequence class
> 2. Server node for manipulation sequence services
> 3. Deprecated test node for sequential manipulation test
> 4. Readme file for available services

### simulation_plugings
> This package contains a gazebo plugin for varying friction in runtime.

### vf_moveit_config
> This package contains moveit config and launch files for arm and gripper.

## Testing example:
1. Source the workspace
```
  source ~/mer_lab/ros_ws/devel/setup.bash
```
2. Launch the manipulation environment including integrated arm & gripper
```
  roslaunch manipulation_env franka_vf.launch
```
3. Sequence through following motion
```
  rosservice call /ih_manip/reset_sequence "start: 0"
```
```
  rosservice call /ih_manip/prepare_grasp "start: 0"
```
```
  rosservice call /ih_manip/object_grasp "start: 0"
```
```
  rosservice call /ih_manip/prehensile_pushing "start: 0"
```
```
  rosservice call /ih_manip/gravity_exploit "start: 0"
```
