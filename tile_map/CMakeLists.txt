cmake_minimum_required(VERSION 2.8.3)
project(tile_map)

set(DEPENDENCIES
  mapviz
  pluginlib
  roscpp
  swri_math_util
  swri_transform_util
  swri_yaml_util
  tf
)
find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)

### Qt
find_package(Qt5Network REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
set(Qt_FOUND TRUE)
set(Qt_INCLUDE_DIRS "${Qt5Network_INCLUDE_DIRS};${Qt5Core_INCLUDE_DIRS};${Qt5Gui_INCLUDE_DIRS};${Qt5OpenGL_INCLUDE_DIRS};${Qt5Widgets_INCLUDE_DIRS}")
set(Qt_LIBRARIES
    Qt5::Network
    Qt5::Core
    Qt5::Gui
    Qt5::OpenGL
    Qt5::Widgets
    )
add_definitions(-DWFlags=WindowFlags)

### OpenGL ###
if(${CMAKE_VERSION} VERSION_EQUAL "3.11.0" OR ${CMAKE_VERSION} VERSION_GREATER "3.11.0")
  cmake_policy(SET CMP0072 NEW)
endif()
find_package(OpenGL REQUIRED)

### GLEW ###
find_package(GLEW REQUIRED)

### PkgConfig ###
find_package(PkgConfig REQUIRED)

### Use PkgConfig to find jsoncpp ###
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

catkin_package(
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_plugin
  INCLUDE_DIRS include
  DEPENDS Qt
  CATKIN_DEPENDS
    ${DEPENDENCIES}
)

# Fix conflict between Boost signals used by tf and QT signals used by mapviz
add_definitions(-DQT_NO_KEYWORDS)

# Need to include the current dir to include the results of building Qt UI files
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

set(TILE_SRC_FILES
  src/image_cache.cpp
  src/texture_cache.cpp
  src/bing_source.cpp
  src/tile_source.cpp
  src/wmts_source.cpp
  src/${PROJECT_NAME}_view.cpp
)
set(QT_HEADERS
  include/${PROJECT_NAME}/image_cache.h
  include/${PROJECT_NAME}/tile_source.h
  include/${PROJECT_NAME}/wmts_source.h
  include/${PROJECT_NAME}/bing_source.h
)
qt5_wrap_cpp(TILE_SRC_FILES ${QT_HEADERS})
add_library(${PROJECT_NAME} ${TILE_SRC_FILES})
target_link_libraries(${PROJECT_NAME}
  ${Qt_LIBRARIES}
  ${OPENGL_glu_LIBRARY}
  ${JSONCPP_LIBRARIES}
  ${catkin_LIBRARIES}
)
if ($ENV{ROS_DISTRO} STREQUAL "kinetic" OR $ENV{ROS_DISTRO} STRLESS "kinetic")
  set_target_properties(${PROJECT_NAME} PROPERTIES 
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
  )
endif()

set(PLUGIN_SRC_FILES
  src/${PROJECT_NAME}_plugin.cpp)
set(PLUGIN_UI_FILES
  src/${PROJECT_NAME}_config.ui)

qt5_wrap_ui(PLUGIN_SRC_FILES ${PLUGIN_UI_FILES})
qt5_wrap_cpp(PLUGIN_SRC_FILES include/${PROJECT_NAME}/${PROJECT_NAME}_plugin.h)

add_library(${PROJECT_NAME}_plugin ${PLUGIN_SRC_FILES})
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME})
if ($ENV{ROS_DISTRO} STREQUAL "kinetic" OR $ENV{ROS_DISTRO} STRLESS "kinetic")
  set_target_properties(${PROJECT_NAME}_plugin PROPERTIES 
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
  )
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES mapviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
