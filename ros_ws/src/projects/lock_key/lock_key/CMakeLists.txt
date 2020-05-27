cmake_minimum_required(VERSION 2.8.3)
project(lock_key)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pluginlib
  geometry_msgs
  pcl_ros
  pcl_conversions
  sensor_msgs
  moveit_planner
  moveit_ros_planning_interface
  moveit_msgs
  tf
  franka_gripper
  )

  find_package(PCL REQUIRED)# COMPONENTS
  #common
  #io
#)

find_package(Boost REQUIRED)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   imgCapture.srv
   findKey.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
   geometry_msgs
 )
catkin_package(DEPENDS moveit_planner)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(arm_move_node src/arm_move.cpp)
add_executable(img_capture_node src/img_capture.cpp)
add_executable(control_node src/controller.cpp)
add_executable(find_key_node src/find_key.cpp)
add_executable(collision_obj_node src/collision_obj.cpp)
add_executable(key_grasp_node src/key_grasp.cpp)

add_dependencies(arm_move_node moveit_planner_gencpp)
add_dependencies(img_capture_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(control_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(find_key_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(collision_obj_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(key_grasp_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(arm_move_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(img_capture_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(control_node ${catkin_LIBRARIES})
target_link_libraries(find_key_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(collision_obj_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(key_grasp_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})