# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pcl_recorder: 0 messages, 3 services")

set(MSG_I_FLAGS "-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pcl_recorder_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/BeginPointCloudRange.srv" NAME_WE)
add_custom_target(_pcl_recorder_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pcl_recorder" "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/BeginPointCloudRange.srv" ""
)

get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/GetPointCloud.srv" NAME_WE)
add_custom_target(_pcl_recorder_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pcl_recorder" "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/GetPointCloud.srv" "sensor_msgs/PointField:std_msgs/Header:sensor_msgs/PointCloud2"
)

get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/EndPointCloudRange.srv" NAME_WE)
add_custom_target(_pcl_recorder_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pcl_recorder" "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/EndPointCloudRange.srv" "sensor_msgs/PointField:std_msgs/Header:sensor_msgs/PointCloud2"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(pcl_recorder
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/BeginPointCloudRange.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pcl_recorder
)
_generate_srv_cpp(pcl_recorder
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/GetPointCloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pcl_recorder
)
_generate_srv_cpp(pcl_recorder
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/EndPointCloudRange.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pcl_recorder
)

### Generating Module File
_generate_module_cpp(pcl_recorder
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pcl_recorder
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pcl_recorder_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pcl_recorder_generate_messages pcl_recorder_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/BeginPointCloudRange.srv" NAME_WE)
add_dependencies(pcl_recorder_generate_messages_cpp _pcl_recorder_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/GetPointCloud.srv" NAME_WE)
add_dependencies(pcl_recorder_generate_messages_cpp _pcl_recorder_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/EndPointCloudRange.srv" NAME_WE)
add_dependencies(pcl_recorder_generate_messages_cpp _pcl_recorder_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pcl_recorder_gencpp)
add_dependencies(pcl_recorder_gencpp pcl_recorder_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pcl_recorder_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(pcl_recorder
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/BeginPointCloudRange.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pcl_recorder
)
_generate_srv_eus(pcl_recorder
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/GetPointCloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pcl_recorder
)
_generate_srv_eus(pcl_recorder
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/EndPointCloudRange.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pcl_recorder
)

### Generating Module File
_generate_module_eus(pcl_recorder
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pcl_recorder
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(pcl_recorder_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(pcl_recorder_generate_messages pcl_recorder_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/BeginPointCloudRange.srv" NAME_WE)
add_dependencies(pcl_recorder_generate_messages_eus _pcl_recorder_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/GetPointCloud.srv" NAME_WE)
add_dependencies(pcl_recorder_generate_messages_eus _pcl_recorder_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/EndPointCloudRange.srv" NAME_WE)
add_dependencies(pcl_recorder_generate_messages_eus _pcl_recorder_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pcl_recorder_geneus)
add_dependencies(pcl_recorder_geneus pcl_recorder_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pcl_recorder_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(pcl_recorder
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/BeginPointCloudRange.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pcl_recorder
)
_generate_srv_lisp(pcl_recorder
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/GetPointCloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pcl_recorder
)
_generate_srv_lisp(pcl_recorder
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/EndPointCloudRange.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pcl_recorder
)

### Generating Module File
_generate_module_lisp(pcl_recorder
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pcl_recorder
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(pcl_recorder_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(pcl_recorder_generate_messages pcl_recorder_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/BeginPointCloudRange.srv" NAME_WE)
add_dependencies(pcl_recorder_generate_messages_lisp _pcl_recorder_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/GetPointCloud.srv" NAME_WE)
add_dependencies(pcl_recorder_generate_messages_lisp _pcl_recorder_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/EndPointCloudRange.srv" NAME_WE)
add_dependencies(pcl_recorder_generate_messages_lisp _pcl_recorder_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pcl_recorder_genlisp)
add_dependencies(pcl_recorder_genlisp pcl_recorder_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pcl_recorder_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(pcl_recorder
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/BeginPointCloudRange.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pcl_recorder
)
_generate_srv_nodejs(pcl_recorder
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/GetPointCloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pcl_recorder
)
_generate_srv_nodejs(pcl_recorder
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/EndPointCloudRange.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pcl_recorder
)

### Generating Module File
_generate_module_nodejs(pcl_recorder
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pcl_recorder
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(pcl_recorder_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(pcl_recorder_generate_messages pcl_recorder_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/BeginPointCloudRange.srv" NAME_WE)
add_dependencies(pcl_recorder_generate_messages_nodejs _pcl_recorder_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/GetPointCloud.srv" NAME_WE)
add_dependencies(pcl_recorder_generate_messages_nodejs _pcl_recorder_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/EndPointCloudRange.srv" NAME_WE)
add_dependencies(pcl_recorder_generate_messages_nodejs _pcl_recorder_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pcl_recorder_gennodejs)
add_dependencies(pcl_recorder_gennodejs pcl_recorder_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pcl_recorder_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(pcl_recorder
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/BeginPointCloudRange.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pcl_recorder
)
_generate_srv_py(pcl_recorder
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/GetPointCloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pcl_recorder
)
_generate_srv_py(pcl_recorder
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/EndPointCloudRange.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pcl_recorder
)

### Generating Module File
_generate_module_py(pcl_recorder
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pcl_recorder
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pcl_recorder_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pcl_recorder_generate_messages pcl_recorder_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/BeginPointCloudRange.srv" NAME_WE)
add_dependencies(pcl_recorder_generate_messages_py _pcl_recorder_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/GetPointCloud.srv" NAME_WE)
add_dependencies(pcl_recorder_generate_messages_py _pcl_recorder_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/pcl_recorder/srv/EndPointCloudRange.srv" NAME_WE)
add_dependencies(pcl_recorder_generate_messages_py _pcl_recorder_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pcl_recorder_genpy)
add_dependencies(pcl_recorder_genpy pcl_recorder_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pcl_recorder_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pcl_recorder)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pcl_recorder
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(pcl_recorder_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pcl_recorder)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pcl_recorder
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(pcl_recorder_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pcl_recorder)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pcl_recorder
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(pcl_recorder_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pcl_recorder)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pcl_recorder
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(pcl_recorder_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pcl_recorder)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pcl_recorder\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pcl_recorder
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(pcl_recorder_generate_messages_py sensor_msgs_generate_messages_py)
endif()
