Active vision project

Run "roslaunch active_vision workspace.launch" to start gazebo with table, kinect and a object

1) rosrun active_vision testingModel_v1
   Code with the "environment" class to do the following functions:
   a) Communicate with gazebo to move kinect / spawn & delete object.
   b) Read the kinect data
   c) Fuse the point clouds
   d) Extract the table and object
   e) Generate unexplored point cloud and update it
   f) Load and update gripper
   g) Find possible grasps and apply collision check with gripper
   h) Save / Rollback a environment configuration
   i) Reset the environment

2) rosrun active_vision supervisorNode
   Code to generate data sets and store point clouds for training purpose. Store the data to a csv.

3) rosrun active_vision genStateVec
   Code to generate the state vector from the saved point clouds
   (Example usage : rosrun active_vision genStateVec ./DataRecAV/ 2020_11_02_135501_dataRec.csv 1)

4) rosrun active_vision visDataRec
   Code to visualize the point clouds saved by supervisorNode
   (Example usage : rosrun active_vision visDataRec ./DataRecAV/ 2020_11_02_135501_dataRec.csv 1)

5) rosrun active_vision visViewsphere
   Code to visualize the viewsphere and navigate through it along the 8 directions.

6) rosrun active_vision findStablePos
   Code to spawn the object in gazebo in different orientations and record the stable orientations.
   It only records the Z, Roll, Pitch.
   Note: Some poses returned can be repetitive / locally stable which needs to manually analyzed removed.
