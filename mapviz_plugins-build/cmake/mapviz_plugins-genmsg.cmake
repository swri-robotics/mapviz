# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mapviz_plugins: 1 messages, 2 services")

set(MSG_I_FLAGS "-Imapviz_plugins:/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mapviz_plugins_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg" NAME_WE)
add_custom_target(_mapviz_plugins_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mapviz_plugins" "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg" ""
)

get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/GPSCommand.srv" NAME_WE)
add_custom_target(_mapviz_plugins_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mapviz_plugins" "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/GPSCommand.srv" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/AvailableServices.srv" NAME_WE)
add_custom_target(_mapviz_plugins_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mapviz_plugins" "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/AvailableServices.srv" "mapviz_plugins/Location"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mapviz_plugins
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapviz_plugins
)

### Generating Services
_generate_srv_cpp(mapviz_plugins
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/GPSCommand.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapviz_plugins
)
_generate_srv_cpp(mapviz_plugins
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/AvailableServices.srv"
  "${MSG_I_FLAGS}"
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapviz_plugins
)

### Generating Module File
_generate_module_cpp(mapviz_plugins
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapviz_plugins
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mapviz_plugins_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mapviz_plugins_generate_messages mapviz_plugins_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg" NAME_WE)
add_dependencies(mapviz_plugins_generate_messages_cpp _mapviz_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/GPSCommand.srv" NAME_WE)
add_dependencies(mapviz_plugins_generate_messages_cpp _mapviz_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/AvailableServices.srv" NAME_WE)
add_dependencies(mapviz_plugins_generate_messages_cpp _mapviz_plugins_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapviz_plugins_gencpp)
add_dependencies(mapviz_plugins_gencpp mapviz_plugins_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapviz_plugins_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mapviz_plugins
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapviz_plugins
)

### Generating Services
_generate_srv_eus(mapviz_plugins
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/GPSCommand.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapviz_plugins
)
_generate_srv_eus(mapviz_plugins
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/AvailableServices.srv"
  "${MSG_I_FLAGS}"
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapviz_plugins
)

### Generating Module File
_generate_module_eus(mapviz_plugins
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapviz_plugins
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mapviz_plugins_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mapviz_plugins_generate_messages mapviz_plugins_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg" NAME_WE)
add_dependencies(mapviz_plugins_generate_messages_eus _mapviz_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/GPSCommand.srv" NAME_WE)
add_dependencies(mapviz_plugins_generate_messages_eus _mapviz_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/AvailableServices.srv" NAME_WE)
add_dependencies(mapviz_plugins_generate_messages_eus _mapviz_plugins_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapviz_plugins_geneus)
add_dependencies(mapviz_plugins_geneus mapviz_plugins_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapviz_plugins_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mapviz_plugins
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapviz_plugins
)

### Generating Services
_generate_srv_lisp(mapviz_plugins
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/GPSCommand.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapviz_plugins
)
_generate_srv_lisp(mapviz_plugins
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/AvailableServices.srv"
  "${MSG_I_FLAGS}"
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapviz_plugins
)

### Generating Module File
_generate_module_lisp(mapviz_plugins
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapviz_plugins
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mapviz_plugins_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mapviz_plugins_generate_messages mapviz_plugins_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg" NAME_WE)
add_dependencies(mapviz_plugins_generate_messages_lisp _mapviz_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/GPSCommand.srv" NAME_WE)
add_dependencies(mapviz_plugins_generate_messages_lisp _mapviz_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/AvailableServices.srv" NAME_WE)
add_dependencies(mapviz_plugins_generate_messages_lisp _mapviz_plugins_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapviz_plugins_genlisp)
add_dependencies(mapviz_plugins_genlisp mapviz_plugins_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapviz_plugins_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mapviz_plugins
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapviz_plugins
)

### Generating Services
_generate_srv_nodejs(mapviz_plugins
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/GPSCommand.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapviz_plugins
)
_generate_srv_nodejs(mapviz_plugins
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/AvailableServices.srv"
  "${MSG_I_FLAGS}"
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapviz_plugins
)

### Generating Module File
_generate_module_nodejs(mapviz_plugins
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapviz_plugins
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mapviz_plugins_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mapviz_plugins_generate_messages mapviz_plugins_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg" NAME_WE)
add_dependencies(mapviz_plugins_generate_messages_nodejs _mapviz_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/GPSCommand.srv" NAME_WE)
add_dependencies(mapviz_plugins_generate_messages_nodejs _mapviz_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/AvailableServices.srv" NAME_WE)
add_dependencies(mapviz_plugins_generate_messages_nodejs _mapviz_plugins_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapviz_plugins_gennodejs)
add_dependencies(mapviz_plugins_gennodejs mapviz_plugins_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapviz_plugins_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mapviz_plugins
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapviz_plugins
)

### Generating Services
_generate_srv_py(mapviz_plugins
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/GPSCommand.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapviz_plugins
)
_generate_srv_py(mapviz_plugins
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/AvailableServices.srv"
  "${MSG_I_FLAGS}"
  "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapviz_plugins
)

### Generating Module File
_generate_module_py(mapviz_plugins
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapviz_plugins
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mapviz_plugins_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mapviz_plugins_generate_messages mapviz_plugins_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/msg/Location.msg" NAME_WE)
add_dependencies(mapviz_plugins_generate_messages_py _mapviz_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/GPSCommand.srv" NAME_WE)
add_dependencies(mapviz_plugins_generate_messages_py _mapviz_plugins_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alex/dai/drones4life_integration/src/mapviz/mapviz_plugins/srv/AvailableServices.srv" NAME_WE)
add_dependencies(mapviz_plugins_generate_messages_py _mapviz_plugins_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mapviz_plugins_genpy)
add_dependencies(mapviz_plugins_genpy mapviz_plugins_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mapviz_plugins_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapviz_plugins)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mapviz_plugins
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mapviz_plugins_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(mapviz_plugins_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapviz_plugins)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mapviz_plugins
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mapviz_plugins_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(mapviz_plugins_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapviz_plugins)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mapviz_plugins
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mapviz_plugins_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(mapviz_plugins_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapviz_plugins)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mapviz_plugins
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mapviz_plugins_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(mapviz_plugins_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapviz_plugins)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapviz_plugins\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mapviz_plugins
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mapviz_plugins_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(mapviz_plugins_generate_messages_py geometry_msgs_generate_messages_py)
endif()
