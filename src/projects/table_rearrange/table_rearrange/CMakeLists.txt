cmake_minimum_required(VERSION 2.8.3)

project(ref_benchmark)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
)

catkin_package(
	CATKIN_DEPENDS roscpp
)

include_directories(${catkin_INCLUDE_DIRS})
link_libraries(${catkin_LIBRARIES})

add_executable(ref_benchmark_node src/main.cpp)
