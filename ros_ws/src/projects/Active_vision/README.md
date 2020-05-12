# Active Vision - Optimizing Camera Viewpoint for Grasping Objects using Reinforcement Learning 
This project deals with moving an eye-in-hand depth camera in as few steps as possible in order to successfully grasp an object in simulation. This optimization in movement is done using reinforcement learning, particularly we used Q-learning. We only consider the motion of the camera and ignore the motion of a robot manipulator for simplicity. The final learned policy moves the camera in one of eight directions (north, northeast, northwest, east, west, south, southeast and southwest) in its viewsphere optimally in each time step until it satisfies the grasp quality threshold.

## Important Packages
Here are a list of some important packages and their uses:
1) object_models: As the name suggests, this package contains the urdf files for some objects to spawn in Gazebo simulation like spoon, bowl, plate, mug etc. In order to spawn a specific object, go to object_models/models/urdf/ and select the urdf file for the desired object and copy its path into line of the node camera_data_gen.cpp node in test_movement/src.
2) franka_gazebo: Contains the world file to spawn the table and kinect camera to Gazebo
3) testing_image_transport: Contains nodes for point cloud registration and stitching.
4) test_movement: Contains the main nodes for Q-learning training and testing.

## Dependencies
1) Ubuntu 16.04 LTS or above
2) ROS Kinetic
3) Gazebo v 7.16
4) C++ 11
5) PCL v 1.7
6) Gazebo's Kinect Plugin

For installing the kinect plugin, please closely follow the tutorial provided in this link: http://gazebosim.org/tutorials/?tut=ros_depth_camera. If you are able to view the point cloud data output in RViz like in the last step of the tutorial, it means that your plugin works!

## How to Run
### Testing
1) First build the ROS workspace and source the setup file using the following commands:
Within Active_vision/ do:
```
catkin_make
```
2) Then open two other terminal windows in the same directory location and in each of them do:
```
source ./devel/setup.bash
```
3) Now select an object model from object_models/models/urdf/ and its urdf file, and copy its absolute path into the ifstream file parameter where the object file is specified in the main function of the node camera_data_gen.cpp node in test_movement/src. For example, if you wanted to spawn a sugar box object and its path is: "/home/ajay/Downloads/Active_vision/src/object_models/models/urdf/sugar_box.urdf", then the code would look like this:
```
ifstream file("/home/ajay/Downloads/Active_vision/src/object_models/models/urdf/sugar_box.urdf");
```
4) Also change the absolute path of the save location of the output of the testing trials in line 435 of test_movement/src/camera_data_gen.cpp to your desired location.
5) In the first terminal window, launch the object, table and kinect models in gazebo using this command:
```
roslaunch franka_control test.launch
```
6) In the second terminal window, use this command to run the registration and ray tracing code:
```
rosrun testing_image_transport test_registration_testing
```
7) In the third terminal window, use the following command to begin the testing:
```
rosrun test_movement camera_data_gen_node
```

### Training 
If you wish to learn your own final policy, you may use the same exact steps as above but you will need to copy the contents of test_movement/src/q-training.cpp to test_movement/src/camera_data_gen.cpp. Everything else remains unchanged.

## Demo of Final Q-Learning Policy
Below is a demo showing a run of our final Q-Learning policy optimizing the viewpoint for grasping the sugar box object in Gazebo. As you can see, the camera reached its optimal viewpoint in three steps. We also highlight the final grasp points (in blue) for a successful grasp of the sugar box at the optimal viewpoint.

<img src="https://github.com/berkcalli/mer_lab/blob/master/ros_ws/src/projects/Active_vision/figures/ezgif.com-video-to-gif.gif" align="middle" width=80% height=80%>

## Q-Learning Training and Testing Plots
Here are the plots for Q-learning for training and testing. During training, the number of steps needed to reach the optimal viewpoint decreases, and the average grasp quality per trial increases. The testing performance shows how the final learned policy performs for 10 different poses of the sugar box. We can see that it takes 2-3 steps atmost for reaching the final optimal viewpoint. Also, the average grasp quality per trial is above the required grasp quality threshold.

<img src="https://github.com/berkcalli/mer_lab/blob/master/ros_ws/src/projects/Active_vision/figures/QL_Training-Final.png" width=65% height=65%>

<img src="https://github.com/berkcalli/mer_lab/blob/master/ros_ws/src/projects/Active_vision/figures/Q-Learning-Final-Testing.png" width=65% height=65%>

## Acknowledgement
We thank professor Berk Calli for his advice and continuous support, and the members of the MERL lab for their help without which we would not have been able to complete this project.
