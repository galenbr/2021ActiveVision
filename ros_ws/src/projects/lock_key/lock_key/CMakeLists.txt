cmake_minimum_required(VERSION 2.8.3)
project(lock_key)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
#  moveit_core
#  moveit_ros_perception
#  moveit_ros_planning_interface
#  moveit_visual_tools
#  moveit_msgs
  pluginlib
  geometry_msgs
  piercing_experiments
  piercing_experiments_msgs
  pcl_ros
  pcl_conversions
  sensor_msgs
  moveit_planner
  )

  find_package(PCL 1.3 REQUIRED COMPONENTS
  common
  io
)

find_package(Boost REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )
catkin_package(DEPENDS piercing_experiments_msgs piercing_experiments moveit_planner)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(arm_move_node src/arm_move.cpp)

target_link_libraries(arm_move_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(arm_move_node moveit_planner_gencpp)