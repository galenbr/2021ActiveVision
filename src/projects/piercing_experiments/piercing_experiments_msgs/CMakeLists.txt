cmake_minimum_required(VERSION 2.8.3)
project(piercing_experiments_msgs)

 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
geometry_msgs
)

 find_package(Boost REQUIRED)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   PrePose.srv
   CartesianPath.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})


# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(${PROJECT_NAME}_node src/piercing_experiments_msgs_node.cpp)

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})