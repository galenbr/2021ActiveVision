# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "randomizer: 0 messages, 1 services")

set(MSG_I_FLAGS "")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(randomizer_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/randomizer/srv/Rand.srv" NAME_WE)
add_custom_target(_randomizer_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "randomizer" "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/randomizer/srv/Rand.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(randomizer
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/randomizer/srv/Rand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/randomizer
)

### Generating Module File
_generate_module_cpp(randomizer
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/randomizer
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(randomizer_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(randomizer_generate_messages randomizer_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/randomizer/srv/Rand.srv" NAME_WE)
add_dependencies(randomizer_generate_messages_cpp _randomizer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(randomizer_gencpp)
add_dependencies(randomizer_gencpp randomizer_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS randomizer_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(randomizer
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/randomizer/srv/Rand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/randomizer
)

### Generating Module File
_generate_module_eus(randomizer
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/randomizer
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(randomizer_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(randomizer_generate_messages randomizer_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/randomizer/srv/Rand.srv" NAME_WE)
add_dependencies(randomizer_generate_messages_eus _randomizer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(randomizer_geneus)
add_dependencies(randomizer_geneus randomizer_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS randomizer_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(randomizer
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/randomizer/srv/Rand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/randomizer
)

### Generating Module File
_generate_module_lisp(randomizer
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/randomizer
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(randomizer_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(randomizer_generate_messages randomizer_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/randomizer/srv/Rand.srv" NAME_WE)
add_dependencies(randomizer_generate_messages_lisp _randomizer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(randomizer_genlisp)
add_dependencies(randomizer_genlisp randomizer_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS randomizer_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(randomizer
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/randomizer/srv/Rand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/randomizer
)

### Generating Module File
_generate_module_nodejs(randomizer
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/randomizer
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(randomizer_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(randomizer_generate_messages randomizer_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/randomizer/srv/Rand.srv" NAME_WE)
add_dependencies(randomizer_generate_messages_nodejs _randomizer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(randomizer_gennodejs)
add_dependencies(randomizer_gennodejs randomizer_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS randomizer_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(randomizer
  "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/randomizer/srv/Rand.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/randomizer
)

### Generating Module File
_generate_module_py(randomizer
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/randomizer
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(randomizer_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(randomizer_generate_messages randomizer_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agandhi2/mer_lab/ros_ws/src/projects/table_rearrange/randomizer/srv/Rand.srv" NAME_WE)
add_dependencies(randomizer_generate_messages_py _randomizer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(randomizer_genpy)
add_dependencies(randomizer_genpy randomizer_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS randomizer_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/randomizer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/randomizer
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/randomizer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/randomizer
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/randomizer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/randomizer
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/randomizer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/randomizer
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/randomizer)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/randomizer\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/randomizer
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
