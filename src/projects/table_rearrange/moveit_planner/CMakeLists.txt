cmake_minimum_required(VERSION 2.8.3)
project(moveit_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
)
catkin_package(
  CATKIN_DEPENDS
    moveit_core
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_libraries(${catkin_LIBRARIES})

add_executable(main_node src/main.cpp src/moveit_planner.cpp)
