# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mapviz: 0 messages, 1 services")

set(MSG_I_FLAGS "-Imarti_common_msgs:/opt/ros/kinetic/share/marti_common_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mapviz_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz/srv/AddMapvizDisplay.srv" NAME_WE)
add_custom_target(_mapviz_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mapviz" "/home/alex/dai/drones4life_integration/src/mapviz/mapviz/srv/AddMapvizDisplay.srv" "marti_common_msgs/KeyValue"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(mapviz
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz/srv/AddMapvizDisplay.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/marti_common_msgs/cmake/../msg/KeyValue.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapviz
)

### Generating Module File
_generate_module_cpp(mapviz
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapviz
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mapviz_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mapviz_generate_messages mapviz_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz/srv/AddMapvizDisplay.srv" NAME_WE)
add_dependencies(mapviz_generate_messages_cpp _mapviz_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapviz_gencpp)
add_dependencies(mapviz_gencpp mapviz_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapviz_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(mapviz
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz/srv/AddMapvizDisplay.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/marti_common_msgs/cmake/../msg/KeyValue.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapviz
)

### Generating Module File
_generate_module_eus(mapviz
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapviz
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mapviz_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mapviz_generate_messages mapviz_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz/srv/AddMapvizDisplay.srv" NAME_WE)
add_dependencies(mapviz_generate_messages_eus _mapviz_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapviz_geneus)
add_dependencies(mapviz_geneus mapviz_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapviz_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(mapviz
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz/srv/AddMapvizDisplay.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/marti_common_msgs/cmake/../msg/KeyValue.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapviz
)

### Generating Module File
_generate_module_lisp(mapviz
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapviz
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mapviz_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mapviz_generate_messages mapviz_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz/srv/AddMapvizDisplay.srv" NAME_WE)
add_dependencies(mapviz_generate_messages_lisp _mapviz_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapviz_genlisp)
add_dependencies(mapviz_genlisp mapviz_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapviz_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(mapviz
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz/srv/AddMapvizDisplay.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/marti_common_msgs/cmake/../msg/KeyValue.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapviz
)

### Generating Module File
_generate_module_nodejs(mapviz
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapviz
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mapviz_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mapviz_generate_messages mapviz_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz/srv/AddMapvizDisplay.srv" NAME_WE)
add_dependencies(mapviz_generate_messages_nodejs _mapviz_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapviz_gennodejs)
add_dependencies(mapviz_gennodejs mapviz_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapviz_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(mapviz
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz/srv/AddMapvizDisplay.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/marti_common_msgs/cmake/../msg/KeyValue.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapviz
)

### Generating Module File
_generate_module_py(mapviz
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapviz
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mapviz_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mapviz_generate_messages mapviz_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz/srv/AddMapvizDisplay.srv" NAME_WE)
add_dependencies(mapviz_generate_messages_py _mapviz_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapviz_genpy)
add_dependencies(mapviz_genpy mapviz_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapviz_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapviz)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapviz
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET marti_common_msgs_generate_messages_cpp)
  add_dependencies(mapviz_generate_messages_cpp marti_common_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapviz)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapviz
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET marti_common_msgs_generate_messages_eus)
  add_dependencies(mapviz_generate_messages_eus marti_common_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapviz)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapviz
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET marti_common_msgs_generate_messages_lisp)
  add_dependencies(mapviz_generate_messages_lisp marti_common_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapviz)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapviz
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET marti_common_msgs_generate_messages_nodejs)
  add_dependencies(mapviz_generate_messages_nodejs marti_common_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapviz)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapviz\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapviz
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET marti_common_msgs_generate_messages_py)
  add_dependencies(mapviz_generate_messages_py marti_common_msgs_generate_messages_py)
endif()
