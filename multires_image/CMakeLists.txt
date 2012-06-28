cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# Require glu
FIND_PACKAGE(GLU REQUIRED)

find_package(Qt4 REQUIRED)

include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR} ${QT_QTOPENGL_INCLUDE_DIR})
add_definitions(${QT_DEFINITIONS})

set(QT_USE_QTOPENGL TRUE)

rosbuild_add_boost_directories()

# Build libtile_cache
file (GLOB TILE_SRC_FILES 
   src/geo_register.cpp
   src/image_manip.cpp
   src/math_util.cpp
   src/point.cpp
   src/string_util.cpp
   src/tile.cpp
   src/tile_cache.cpp
   src/tile_set.cpp
   src/tile_set_layer.cpp)
QT4_WRAP_CPP(TILE_SRC_FILES include/multires_image/tile_cache.h)
rosbuild_add_library(${PROJECT_NAME} ${TILE_SRC_FILES})
rosbuild_link_boost(${PROJECT_NAME} thread)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARIES} ${GLU_LIBRARY})

# Build libmultires_widget
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(src)
file (GLOB UI_FILES src/*.ui)
file (GLOB WIDGET_SRC_FILES src/tile_view.cpp src/QGLMap.cpp)
QT4_WRAP_UI(WIDGET_SRC_FILES ${UI_FILES})
QT4_WRAP_CPP(WIDGET_SRC_FILES include/multires_image/QGLMap.h)
rosbuild_add_library(multires_widget ${WIDGET_SRC_FILES})
target_link_libraries(multires_widget ${PROJECT_NAME})

# Build nodes
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/nodes)
include_directories(src)
file (GLOB VIEW_NODE_SRC_FILES src/nodes/multires_view_node.cpp)
QT4_WRAP_CPP(VIEW_NODE_SRC_FILES include/multires_image/multires_view_node.h)
rosbuild_add_executable(multires_view_node ${VIEW_NODE_SRC_FILES})
target_link_libraries(multires_view_node multires_widget)
