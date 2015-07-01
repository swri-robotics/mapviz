cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_find_ros_package(build_tools)
include(${build_tools_PACKAGE_PATH}/scripts/build_common.cmake)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Require glu
FIND_PACKAGE(GLU REQUIRED)
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtNetwork QtOpenGl)

include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIRS} ${QT_QTOPENGL_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_definitions(${QT_DEFINITIONS})

set(QT_USE_QTOPENGL TRUE)

# Fix conflict between Boost signals used by tf and QT signals used by mapviz
add_definitions(-DQT_NO_KEYWORDS)

rosbuild_add_boost_directories()

file (GLOB TILE_SRC_FILES 
   src/image_cache.cpp
   src/texture_cache.cpp
   src/tile_map_view.cpp)
QT4_WRAP_CPP(TILE_SRC_FILES include/tile_map/image_cache.h)
rosbuild_add_library(${PROJECT_NAME} ${TILE_SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARIES} ${GLU_LIBRARY})

file (GLOB PLUGIN_SRC_FILES src/tile_map_plugin.cpp)
file (GLOB PLUGIN_UI_FILES src/*.ui)
QT4_WRAP_UI(PLUGIN_SRC_FILES ${PLUGIN_UI_FILES})
QT4_WRAP_CPP(PLUGIN_SRC_FILES include/tile_map/tile_map_plugin.h)

rosbuild_add_library(${PROJECT_NAME}_plugin ${PLUGIN_SRC_FILES})
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME})
