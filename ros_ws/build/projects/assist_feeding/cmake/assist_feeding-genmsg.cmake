# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "assist_feeding: 0 messages, 3 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(assist_feeding_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_open.srv" NAME_WE)
add_custom_target(_assist_feeding_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "assist_feeding" "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_open.srv" ""
)

get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/finger_orientation.srv" NAME_WE)
add_custom_target(_assist_feeding_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "assist_feeding" "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/finger_orientation.srv" ""
)

get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_close.srv" NAME_WE)
add_custom_target(_assist_feeding_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "assist_feeding" "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_close.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(assist_feeding
  "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_open.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/assist_feeding
)
_generate_srv_cpp(assist_feeding
  "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/finger_orientation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/assist_feeding
)
_generate_srv_cpp(assist_feeding
  "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_close.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/assist_feeding
)

### Generating Module File
_generate_module_cpp(assist_feeding
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/assist_feeding
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(assist_feeding_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(assist_feeding_generate_messages assist_feeding_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_open.srv" NAME_WE)
add_dependencies(assist_feeding_generate_messages_cpp _assist_feeding_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/finger_orientation.srv" NAME_WE)
add_dependencies(assist_feeding_generate_messages_cpp _assist_feeding_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_close.srv" NAME_WE)
add_dependencies(assist_feeding_generate_messages_cpp _assist_feeding_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(assist_feeding_gencpp)
add_dependencies(assist_feeding_gencpp assist_feeding_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS assist_feeding_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(assist_feeding
  "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_open.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/assist_feeding
)
_generate_srv_eus(assist_feeding
  "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/finger_orientation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/assist_feeding
)
_generate_srv_eus(assist_feeding
  "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_close.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/assist_feeding
)

### Generating Module File
_generate_module_eus(assist_feeding
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/assist_feeding
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(assist_feeding_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(assist_feeding_generate_messages assist_feeding_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_open.srv" NAME_WE)
add_dependencies(assist_feeding_generate_messages_eus _assist_feeding_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/finger_orientation.srv" NAME_WE)
add_dependencies(assist_feeding_generate_messages_eus _assist_feeding_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_close.srv" NAME_WE)
add_dependencies(assist_feeding_generate_messages_eus _assist_feeding_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(assist_feeding_geneus)
add_dependencies(assist_feeding_geneus assist_feeding_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS assist_feeding_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(assist_feeding
  "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_open.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/assist_feeding
)
_generate_srv_lisp(assist_feeding
  "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/finger_orientation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/assist_feeding
)
_generate_srv_lisp(assist_feeding
  "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_close.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/assist_feeding
)

### Generating Module File
_generate_module_lisp(assist_feeding
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/assist_feeding
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(assist_feeding_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(assist_feeding_generate_messages assist_feeding_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_open.srv" NAME_WE)
add_dependencies(assist_feeding_generate_messages_lisp _assist_feeding_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/finger_orientation.srv" NAME_WE)
add_dependencies(assist_feeding_generate_messages_lisp _assist_feeding_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_close.srv" NAME_WE)
add_dependencies(assist_feeding_generate_messages_lisp _assist_feeding_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(assist_feeding_genlisp)
add_dependencies(assist_feeding_genlisp assist_feeding_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS assist_feeding_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(assist_feeding
  "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_open.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/assist_feeding
)
_generate_srv_nodejs(assist_feeding
  "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/finger_orientation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/assist_feeding
)
_generate_srv_nodejs(assist_feeding
  "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_close.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/assist_feeding
)

### Generating Module File
_generate_module_nodejs(assist_feeding
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/assist_feeding
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(assist_feeding_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(assist_feeding_generate_messages assist_feeding_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_open.srv" NAME_WE)
add_dependencies(assist_feeding_generate_messages_nodejs _assist_feeding_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/finger_orientation.srv" NAME_WE)
add_dependencies(assist_feeding_generate_messages_nodejs _assist_feeding_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_close.srv" NAME_WE)
add_dependencies(assist_feeding_generate_messages_nodejs _assist_feeding_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(assist_feeding_gennodejs)
add_dependencies(assist_feeding_gennodejs assist_feeding_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS assist_feeding_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(assist_feeding
  "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_open.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assist_feeding
)
_generate_srv_py(assist_feeding
  "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/finger_orientation.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assist_feeding
)
_generate_srv_py(assist_feeding
  "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_close.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assist_feeding
)

### Generating Module File
_generate_module_py(assist_feeding
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assist_feeding
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(assist_feeding_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(assist_feeding_generate_messages assist_feeding_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_open.srv" NAME_WE)
add_dependencies(assist_feeding_generate_messages_py _assist_feeding_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/finger_orientation.srv" NAME_WE)
add_dependencies(assist_feeding_generate_messages_py _assist_feeding_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/assist_feeding/srv/gripper_close.srv" NAME_WE)
add_dependencies(assist_feeding_generate_messages_py _assist_feeding_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(assist_feeding_genpy)
add_dependencies(assist_feeding_genpy assist_feeding_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS assist_feeding_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/assist_feeding)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/assist_feeding
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(assist_feeding_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/assist_feeding)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/assist_feeding
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(assist_feeding_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/assist_feeding)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/assist_feeding
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(assist_feeding_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/assist_feeding)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/assist_feeding
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(assist_feeding_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assist_feeding)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assist_feeding\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assist_feeding
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(assist_feeding_generate_messages_py std_msgs_generate_messages_py)
endif()
