cmake_minimum_required(VERSION 2.8.3)
project(owi_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  Eigen3
)

find_package(Boost REQUIRED COMPONENTS system)

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

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(owi_controller_node src/owi_controller.cpp)
add_dependencies(owi_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(owi_controller_node ${catkin_LIBRARIES})
