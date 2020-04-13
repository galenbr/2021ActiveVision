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
  )

  find_package(PCL REQUIRED)# COMPONENTS
  #common
  #io
#)

find_package(Boost REQUIRED)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   imgCapture.srv
   findKey.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

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

add_dependencies(arm_move_node moveit_planner_gencpp)
add_dependencies(img_capture_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(control_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(find_key_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(arm_move_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(img_capture_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(control_node ${catkin_LIBRARIES})
target_link_libraries(find_key_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})