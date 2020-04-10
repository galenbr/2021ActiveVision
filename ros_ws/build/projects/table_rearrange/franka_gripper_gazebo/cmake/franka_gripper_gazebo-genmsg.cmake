# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "franka_gripper_gazebo: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(franka_gripper_gazebo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/franka_gripper_gazebo/srv/GripMsg.srv" NAME_WE)
add_custom_target(_franka_gripper_gazebo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_gripper_gazebo" "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/franka_gripper_gazebo/srv/GripMsg.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(franka_gripper_gazebo
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/franka_gripper_gazebo/srv/GripMsg.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_gripper_gazebo
)

### Generating Module File
_generate_module_cpp(franka_gripper_gazebo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_gripper_gazebo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(franka_gripper_gazebo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(franka_gripper_gazebo_generate_messages franka_gripper_gazebo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/franka_gripper_gazebo/srv/GripMsg.srv" NAME_WE)
add_dependencies(franka_gripper_gazebo_generate_messages_cpp _franka_gripper_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_gripper_gazebo_gencpp)
add_dependencies(franka_gripper_gazebo_gencpp franka_gripper_gazebo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_gripper_gazebo_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(franka_gripper_gazebo
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/franka_gripper_gazebo/srv/GripMsg.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_gripper_gazebo
)

### Generating Module File
_generate_module_eus(franka_gripper_gazebo
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_gripper_gazebo
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(franka_gripper_gazebo_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(franka_gripper_gazebo_generate_messages franka_gripper_gazebo_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/franka_gripper_gazebo/srv/GripMsg.srv" NAME_WE)
add_dependencies(franka_gripper_gazebo_generate_messages_eus _franka_gripper_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_gripper_gazebo_geneus)
add_dependencies(franka_gripper_gazebo_geneus franka_gripper_gazebo_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_gripper_gazebo_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(franka_gripper_gazebo
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/franka_gripper_gazebo/srv/GripMsg.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_gripper_gazebo
)

### Generating Module File
_generate_module_lisp(franka_gripper_gazebo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_gripper_gazebo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(franka_gripper_gazebo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(franka_gripper_gazebo_generate_messages franka_gripper_gazebo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/franka_gripper_gazebo/srv/GripMsg.srv" NAME_WE)
add_dependencies(franka_gripper_gazebo_generate_messages_lisp _franka_gripper_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_gripper_gazebo_genlisp)
add_dependencies(franka_gripper_gazebo_genlisp franka_gripper_gazebo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_gripper_gazebo_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(franka_gripper_gazebo
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/franka_gripper_gazebo/srv/GripMsg.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_gripper_gazebo
)

### Generating Module File
_generate_module_nodejs(franka_gripper_gazebo
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_gripper_gazebo
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(franka_gripper_gazebo_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(franka_gripper_gazebo_generate_messages franka_gripper_gazebo_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/franka_gripper_gazebo/srv/GripMsg.srv" NAME_WE)
add_dependencies(franka_gripper_gazebo_generate_messages_nodejs _franka_gripper_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_gripper_gazebo_gennodejs)
add_dependencies(franka_gripper_gazebo_gennodejs franka_gripper_gazebo_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_gripper_gazebo_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(franka_gripper_gazebo
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/franka_gripper_gazebo/srv/GripMsg.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_gripper_gazebo
)

### Generating Module File
_generate_module_py(franka_gripper_gazebo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_gripper_gazebo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(franka_gripper_gazebo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(franka_gripper_gazebo_generate_messages franka_gripper_gazebo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/franka_gripper_gazebo/srv/GripMsg.srv" NAME_WE)
add_dependencies(franka_gripper_gazebo_generate_messages_py _franka_gripper_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_gripper_gazebo_genpy)
add_dependencies(franka_gripper_gazebo_genpy franka_gripper_gazebo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_gripper_gazebo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_gripper_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_gripper_gazebo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(franka_gripper_gazebo_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_gripper_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_gripper_gazebo
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(franka_gripper_gazebo_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_gripper_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_gripper_gazebo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(franka_gripper_gazebo_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_gripper_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_gripper_gazebo
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(franka_gripper_gazebo_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_gripper_gazebo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_gripper_gazebo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_gripper_gazebo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(franka_gripper_gazebo_generate_messages_py std_msgs_generate_messages_py)
endif()
