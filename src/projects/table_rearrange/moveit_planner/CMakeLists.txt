cmake_minimum_required(VERSION 2.8.3)
project(moveit_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  geometry_msgs
)

add_service_files(
  FILES
  MovePoint.srv
  MoveQuat.srv
  MovePose.srv
  MoveJoint.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_libraries(${catkin_LIBRARIES})

add_executable(main_node src/main.cpp src/moveit_planner.cpp)
add_dependencies(main_node moveit_planner_gencpp)
