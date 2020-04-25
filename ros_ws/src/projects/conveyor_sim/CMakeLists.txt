cmake_minimum_required(VERSION 2.8.3)
project(conveyor_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_msgs
)
find_package(gazebo REQUIRED)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(conveyor_sim_node src/conveyor_sim.cpp)
target_link_libraries(conveyor_sim_node ${catkin_LIBRARIES})

# Conveyor Plugin
add_library(ConveyorBeltPlugin SHARED plugins/gazebo_conveyor_plugin.cc)
target_link_libraries(ConveyorBeltPlugin ${GAZEBO_LIBRARIES})
install(TARGETS ConveyorBeltPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)
