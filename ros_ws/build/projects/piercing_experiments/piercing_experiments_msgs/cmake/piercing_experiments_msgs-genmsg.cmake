# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "piercing_experiments_msgs: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(piercing_experiments_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/PrePose.srv" NAME_WE)
add_custom_target(_piercing_experiments_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "piercing_experiments_msgs" "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/PrePose.srv" "geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Point"
)

get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/CartesianPath.srv" NAME_WE)
add_custom_target(_piercing_experiments_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "piercing_experiments_msgs" "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/CartesianPath.srv" "geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(piercing_experiments_msgs
  "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/PrePose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/piercing_experiments_msgs
)
_generate_srv_cpp(piercing_experiments_msgs
  "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/CartesianPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/piercing_experiments_msgs
)

### Generating Module File
_generate_module_cpp(piercing_experiments_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/piercing_experiments_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(piercing_experiments_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(piercing_experiments_msgs_generate_messages piercing_experiments_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/PrePose.srv" NAME_WE)
add_dependencies(piercing_experiments_msgs_generate_messages_cpp _piercing_experiments_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/CartesianPath.srv" NAME_WE)
add_dependencies(piercing_experiments_msgs_generate_messages_cpp _piercing_experiments_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(piercing_experiments_msgs_gencpp)
add_dependencies(piercing_experiments_msgs_gencpp piercing_experiments_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS piercing_experiments_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(piercing_experiments_msgs
  "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/PrePose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/piercing_experiments_msgs
)
_generate_srv_eus(piercing_experiments_msgs
  "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/CartesianPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/piercing_experiments_msgs
)

### Generating Module File
_generate_module_eus(piercing_experiments_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/piercing_experiments_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(piercing_experiments_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(piercing_experiments_msgs_generate_messages piercing_experiments_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/PrePose.srv" NAME_WE)
add_dependencies(piercing_experiments_msgs_generate_messages_eus _piercing_experiments_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/CartesianPath.srv" NAME_WE)
add_dependencies(piercing_experiments_msgs_generate_messages_eus _piercing_experiments_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(piercing_experiments_msgs_geneus)
add_dependencies(piercing_experiments_msgs_geneus piercing_experiments_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS piercing_experiments_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(piercing_experiments_msgs
  "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/PrePose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/piercing_experiments_msgs
)
_generate_srv_lisp(piercing_experiments_msgs
  "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/CartesianPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/piercing_experiments_msgs
)

### Generating Module File
_generate_module_lisp(piercing_experiments_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/piercing_experiments_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(piercing_experiments_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(piercing_experiments_msgs_generate_messages piercing_experiments_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/PrePose.srv" NAME_WE)
add_dependencies(piercing_experiments_msgs_generate_messages_lisp _piercing_experiments_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/CartesianPath.srv" NAME_WE)
add_dependencies(piercing_experiments_msgs_generate_messages_lisp _piercing_experiments_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(piercing_experiments_msgs_genlisp)
add_dependencies(piercing_experiments_msgs_genlisp piercing_experiments_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS piercing_experiments_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(piercing_experiments_msgs
  "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/PrePose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/piercing_experiments_msgs
)
_generate_srv_nodejs(piercing_experiments_msgs
  "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/CartesianPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/piercing_experiments_msgs
)

### Generating Module File
_generate_module_nodejs(piercing_experiments_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/piercing_experiments_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(piercing_experiments_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(piercing_experiments_msgs_generate_messages piercing_experiments_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/PrePose.srv" NAME_WE)
add_dependencies(piercing_experiments_msgs_generate_messages_nodejs _piercing_experiments_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/CartesianPath.srv" NAME_WE)
add_dependencies(piercing_experiments_msgs_generate_messages_nodejs _piercing_experiments_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(piercing_experiments_msgs_gennodejs)
add_dependencies(piercing_experiments_msgs_gennodejs piercing_experiments_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS piercing_experiments_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(piercing_experiments_msgs
  "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/PrePose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/piercing_experiments_msgs
)
_generate_srv_py(piercing_experiments_msgs
  "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/CartesianPath.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/piercing_experiments_msgs
)

### Generating Module File
_generate_module_py(piercing_experiments_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/piercing_experiments_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(piercing_experiments_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(piercing_experiments_msgs_generate_messages piercing_experiments_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/PrePose.srv" NAME_WE)
add_dependencies(piercing_experiments_msgs_generate_messages_py _piercing_experiments_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/piercing_experiments/piercing_experiments_msgs/srv/CartesianPath.srv" NAME_WE)
add_dependencies(piercing_experiments_msgs_generate_messages_py _piercing_experiments_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(piercing_experiments_msgs_genpy)
add_dependencies(piercing_experiments_msgs_genpy piercing_experiments_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS piercing_experiments_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/piercing_experiments_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/piercing_experiments_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(piercing_experiments_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(piercing_experiments_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/piercing_experiments_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/piercing_experiments_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(piercing_experiments_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(piercing_experiments_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/piercing_experiments_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/piercing_experiments_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(piercing_experiments_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(piercing_experiments_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/piercing_experiments_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/piercing_experiments_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(piercing_experiments_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(piercing_experiments_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/piercing_experiments_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/piercing_experiments_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/piercing_experiments_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(piercing_experiments_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(piercing_experiments_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
