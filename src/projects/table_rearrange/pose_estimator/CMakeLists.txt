cmake_minimum_required(VERSION 2.8.3)
project(pose_estimator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  message_runtime
  sensor_msgs
  geometry_msgs
)

add_service_files(
  FILES
  PoseEstimation.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(obtain_pose_gazebo_node src/obtain_pose_gazebo.cpp)
target_link_libraries(obtain_pose_gazebo_node ${catkin_LIBRARIES})
add_dependencies(obtain_pose_gazebo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
