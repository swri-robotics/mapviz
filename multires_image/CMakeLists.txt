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

# Build libtile_cache
file (GLOB TILE_SRC_FILES src/libtile_cache/*.cpp)
QT4_WRAP_CPP(TILE_SRC_FILES src/libtile_cache/tile_cache.h)
rosbuild_add_library(tile_cache ${TILE_SRC_FILES})
target_link_libraries(tile_cache ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARIES} ${GLU_LIBRARY})

# Build libmultires_widget
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libmultires_widget)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libtile_cache)
include_directories(src/libtile_cache)
file (GLOB UI_FILES src/libmultires_widget/*.ui)
file (GLOB WIDGET_SRC_FILES src/libmultires_widget/*.cpp)
QT4_WRAP_UI(WIDGET_SRC_FILES ${UI_FILES})
QT4_WRAP_CPP(WIDGET_SRC_FILES src/libmultires_widget/QGLMap.h src/libmultires_widget/tile_view.h)
rosbuild_add_library(multires_widget ${WIDGET_SRC_FILES})
target_link_libraries(multires_widget tile_cache ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARIES} ${GLU_LIBRARY})

# Build nodes
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/nodes)
include_directories(src/libmultires_widget)
file (GLOB VIEW_NODE_SRC_FILES src/nodes/multires_view_node.cpp)
QT4_WRAP_CPP(VIEW_NODE_SRC_FILES src/nodes/multires_view_node.h)
rosbuild_add_executable(multires_view_node ${VIEW_NODE_SRC_FILES})
target_link_libraries(multires_view_node multires_widget ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARIES} ${GLU_LIBRARY})