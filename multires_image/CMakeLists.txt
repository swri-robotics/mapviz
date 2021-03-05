cmake_minimum_required(VERSION 2.8.3)
project(multires_image)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gps_common
  mapviz
  pluginlib
  roscpp 
  tf 
  swri_transform_util 
  swri_math_util
  swri_yaml_util
)

### Boost ###
find_package(Boost REQUIRED thread)

### OpenGL ###
if(${CMAKE_VERSION} VERSION_EQUAL "3.11.0" OR ${CMAKE_VERSION} VERSION_GREATER "3.11.0")
  cmake_policy(SET CMP0072 NEW)
endif()
find_package(OpenGL REQUIRED)

### OpenCV ###
# multires_image doesn't directly use OpenCV, but it uses swri_transform_util, and
# we need to include OpenCV to make one of its headers compile.
find_package(OpenCV COMPONENTS core imgproc REQUIRED)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
# Setting these variable so catkin_package can export Qt as a dependency
set(Qt_FOUND TRUE)
set(Qt_INCLUDE_DIRS "${Qt5Core_INCLUDE_DIRS};${Qt5Gui_INCLUDE_DIRS};${Qt5OpenGL_INCLUDE_DIRS};${Qt5Widgets_INCLUDE_DIRS}")
set(Qt_LIBRARIES
    Qt5::Core
    Qt5::Gui
    Qt5::OpenGL
    Qt5::Widgets
    )
add_definitions(-DWFlags=WindowFlags)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_plugin multires_widget
  DEPENDS Qt
  CATKIN_DEPENDS
    mapviz
    pluginlib
    roscpp
    swri_transform_util
    swri_yaml_util
    tf
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Build libtile_cache
set(TILE_SRC_FILES 
   src/tile.cpp
   src/tile_cache.cpp
   src/tile_set.cpp
   src/tile_set_layer.cpp)

qt5_wrap_cpp(TILE_SRC_FILES include/${PROJECT_NAME}/tile_cache.h)

add_library(${PROJECT_NAME} 
  ${TILE_SRC_FILES}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Qt_LIBRARIES}
  ${GLU_LIBRARY}
)
if ($ENV{ROS_DISTRO} STREQUAL "kinetic" OR $ENV{ROS_DISTRO} STRLESS "kinetic")
  set_target_properties(${PROJECT_NAME} PROPERTIES 
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
  )
endif()

# Build libmultires_widget
set(UI_FILES
  src/QGLMap.ui
  src/multires_config.ui
)
set(WIDGET_SRC_FILES src/tile_view.cpp src/QGLMap.cpp)

qt5_wrap_ui(WIDGET_SRC_FILES ${UI_FILES})
qt5_wrap_cpp(WIDGET_SRC_FILES include/${PROJECT_NAME}/QGLMap.h)

add_library(multires_widget ${WIDGET_SRC_FILES})
target_link_libraries(multires_widget ${PROJECT_NAME})
if ($ENV{ROS_DISTRO} STREQUAL "kinetic" OR $ENV{ROS_DISTRO} STRLESS "kinetic")
  set_target_properties(multires_widget PROPERTIES 
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
  )
endif()

# Build nodes
set(VIEW_NODE_SRC_FILES src/nodes/multires_view_node.cpp)
qt5_wrap_cpp(VIEW_NODE_SRC_FILES include/${PROJECT_NAME}/multires_view_node.h)
add_executable(multires_view_node ${VIEW_NODE_SRC_FILES})
target_link_libraries(multires_view_node multires_widget)
if ($ENV{ROS_DISTRO} STREQUAL "kinetic" OR $ENV{ROS_DISTRO} STRLESS "kinetic")
  set_target_properties(multires_view_node PROPERTIES 
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
  )
endif()

# Build mapviz plugin
set(MAPVIZ_SRC_FILES 
  src/${PROJECT_NAME}_plugin.cpp
  src/multires_view.cpp)
qt5_wrap_ui(MAPVIZ_SRC_FILES src/multires_config.ui)
qt5_wrap_cpp(MAPVIZ_SRC_FILES include/${PROJECT_NAME}/${PROJECT_NAME}_plugin.h)

add_library(${PROJECT_NAME}_plugin ${MAPVIZ_SRC_FILES})
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME})
if ($ENV{ROS_DISTRO} STREQUAL "kinetic" OR $ENV{ROS_DISTRO} STRLESS "kinetic")
  set_target_properties(${PROJECT_NAME}_plugin PROPERTIES 
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
  )
endif()


### Install ${PROJECT_NAME} plugin ###
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  multires_view_node
  multires_widget
  ${PROJECT_NAME}_plugin
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/mapviz_tile_loader
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES mapviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
