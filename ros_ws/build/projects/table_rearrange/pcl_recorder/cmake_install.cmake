# Install script for directory: /home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/agandhi2/mer_lab/ros_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl_recorder/srv" TYPE FILE FILES
    "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/GetPointCloud.srv"
    "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/BeginPointCloudRange.srv"
    "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/EndPointCloudRange.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl_recorder/cmake" TYPE FILE FILES "/home/agandhi2/mer_lab/ros_ws/build/projects/table_rearrange/pcl_recorder/catkin_generated/installspace/pcl_recorder-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/agandhi2/mer_lab/ros_ws/devel/include/pcl_recorder")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/agandhi2/mer_lab/ros_ws/devel/share/roseus/ros/pcl_recorder")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/agandhi2/mer_lab/ros_ws/devel/share/common-lisp/ros/pcl_recorder")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/agandhi2/mer_lab/ros_ws/devel/share/gennodejs/ros/pcl_recorder")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/agandhi2/mer_lab/ros_ws/devel/lib/python2.7/dist-packages/pcl_recorder")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/agandhi2/mer_lab/ros_ws/devel/lib/python2.7/dist-packages/pcl_recorder")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/agandhi2/mer_lab/ros_ws/build/projects/table_rearrange/pcl_recorder/catkin_generated/installspace/pcl_recorder.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl_recorder/cmake" TYPE FILE FILES "/home/agandhi2/mer_lab/ros_ws/build/projects/table_rearrange/pcl_recorder/catkin_generated/installspace/pcl_recorder-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl_recorder/cmake" TYPE FILE FILES
    "/home/agandhi2/mer_lab/ros_ws/build/projects/table_rearrange/pcl_recorder/catkin_generated/installspace/pcl_recorderConfig.cmake"
    "/home/agandhi2/mer_lab/ros_ws/build/projects/table_rearrange/pcl_recorder/catkin_generated/installspace/pcl_recorderConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pcl_recorder" TYPE FILE FILES "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/package.xml")
endif()

