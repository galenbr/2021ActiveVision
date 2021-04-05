# Active vision project
Grasping novel objects using Heuristic and ML based active vision.
> Franka Emika Panda <br>
> Parallel jaw gripper <br>
> Intel Realsense D435i <br>

## Bash scripts

1. **dataCollector.sh** :
    <br> TODO

2. **policyEvaluator.sh** :
    <br> TODO

## Launch files

1. **workspace.launch** :
    <br> Starts the environment for the required simulation mode.
    <br> ```roslaunch active_vision workspace.launch simulationMode:="FRANKASIMULATION"```

    | Argument       | Default | Values |
    | -------------- | ------- | ------ |
    | simulationMode | SIMULATION | SIMULATION <br> FRANKASIMULATION <br> FRANKA |
    | visual         | ON | ON <br> OFF |

2. **policyTester.launch** :
    <br> Starts the environment, starts the required policy and test the policy.
    <br> ``` roslaunch active_vision policyTester.launch policy:="3DHEURISTIC" ```

    | Argument       | Default | Values |
    | -------------- | ------- | ------ |
    | simulationMode | SIMULATION | SIMULATION <br> FRANKASIMULATION <br> FRANKA |
    | policy         | NIL | NIL <br> HEURISTIC <br> 3DHEURISTIC <br> PCA_LDA <br> PCA_LDA_LR <br> PCA_LR <br> RANDOM |

## Active vision algorithm scripts

1. **3DheuristicPolicyService.cpp** :
    <br> TODO

2. **heuristicPolicyService.cpp** :
    <br> TODO

3. **trainedPolicyService.py** :
    <br> TODO

4. **trainingPolicy.py** :
    <br> TODO

5. **QLearning3.py** :
    <br> TODO

## Data Collection and Policy testing scripts

1. **environmentTesting.cpp** :
    <br> Code to test the individual functions of environment.cpp file. Used to test any changes to the environment.cpp file.
    <br> ``` rosrun active_vision environmentTesting ```

    | S.No. | Functions available |
    | :---: | ------------------- |
    | 01 | Spawn and delete objects and its configurations on the table |
    | 02 | Load and view the gripper model |
    | 03 | Move the realsense/franka to a custom position |
    | 04 | Continuously move the realsense in a viewsphere with centre on the table |
    | 05 | Read and view the data from realsense |
    | 06 | Read and fuse the data from 4 different viewpoints |
    | 07 | Extract the table and object from the environment |
    | 08 | Generate the initial unexplored pointcloud based on the object |
    | 09 | Update the unexplored pointcloud based on 4 different viewpoints |
    | 10 | Grasp synthesis after fusing 4 viewpoints |
    | 11 | Selecting a grasp after grasp synthesis and collision check for a object |
    | 12 | Store and rollback configurations |
    | 13 | Save point clouds |
    | 14 | Read CSV and print a column |
    | 15 | Read PCD based on CSV data |
    | 16 | Surface patch testing |
    | 17 | Gripper open close testing |
    | 18 | Testing object pickup |
    | 19 | Testing moveit collision add/remove |
    | 20 | Testing moveit constraint |

2. **dataCollector.cpp** :
     <br> Code to generate data sets and store point clouds for training purpose. Data is stored to a csv file.
     <br> ``` rosrun active_vision dataCollector <mode> ```

3. **policyTester.cpp** :
    <br> TODO

3. **genStateVec.cpp** :
    <br> Code to generate the state vector from the saved point clouds.
    <br> ``` rosrun active_vision genStateVec <Directory> <CSV filename> <Type> <Max Steps> ```

## Visualization scripts

1. **visDataRec.cpp** :
    <br> Code to visualize the point clouds saved by dataCollector.
    <br> ``` rosrun active_vision visDataRec <Directory> <CSV filename> <Visual> ```

2. **visDataRecV2.cpp** :
    <br> TODO

3. **visDataCollectionPaths.cpp** :
    <br> TODO

4. **visAllPaths.cpp** :
    <br> TODO

5. **visViewsphere.cpp** :
    <br> Code to visualize the viewsphere and navigate through it along the 8 directions.
    <br> ``` rosrun active_vision visViewsphere ```

6. **visBFS.py** :
    <br> TODO

7. **visWorkSpace.py** :
    <br> TODO

## Useful tools

1. **toolDataHandling.cpp** :
    <br> TODO

2. **toolStateVector.cpp** :
    <br> TODO

3. **toolViewPointCalc.cpp** :
    <br> TODO

4. **toolVisualization.cpp** :
    <br> TODO

## Helpful tips

1. Viewing the arguments for a roslaunch file
    <br> ``` roslaunch --ros-args <pkg-name> <launch-file-name> ```
