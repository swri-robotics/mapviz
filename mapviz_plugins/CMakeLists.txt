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

# Require glu
FIND_PACKAGE(GLU REQUIRED)

# Fix conflict between Boost signals used by tf and QT signals used by mapviz
add_definitions(-DQT_NO_KEYWORDS)

find_package(Qt4 REQUIRED)

include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR}  ${QT_QTOPENGL_INCLUDE_DIR})
add_definitions(${QT_DEFINITIONS})

set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTCORE TRUE)
set(QT_USE_QTGUI TRUE)

rosbuild_add_boost_directories()

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
file (GLOB UI_FILES src/*.ui)

file (GLOB SRC_FILES 
	src/multires_image_plugin.cpp 
	src/image_plugin.cpp
	src/disparity_plugin.cpp 
	src/multires_view.cpp 
	src/odometry_plugin.cpp 
	src/marker_plugin.cpp 
	src/grid_plugin.cpp
	src/path_plugin.cpp
	src/robot_image_plugin.cpp)
	
QT4_WRAP_UI(SRC_FILES ${UI_FILES})

QT4_WRAP_CPP(SRC_FILES 
	src/multires_image_plugin.h 
	src/image_plugin.h
	src/disparity_plugin.h
	src/odometry_plugin.h 
	src/marker_plugin.h 
	src/grid_plugin.h
	src/path_plugin.h
	src/robot_image_plugin.h)

rosbuild_add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARIES} ${GLU_LIBRARY})
