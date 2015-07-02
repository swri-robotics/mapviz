cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_find_ros_package(build_tools)
include(${build_tools_PACKAGE_PATH}/scripts/build_common.cmake)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Require glu
FIND_PACKAGE(GLU REQUIRED)
FIND_PACKAGE(GLUT REQUIRED)

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
    src/textured_marker_plugin.cpp
    src/tf_frame_plugin.cpp
    src/robot_image_plugin.cpp
    src/gps_plugin.cpp)
    
QT4_WRAP_UI(SRC_FILES ${UI_FILES})

QT4_WRAP_CPP(SRC_FILES 
    include/mapviz_plugins/multires_image_plugin.h 
    include/mapviz_plugins/image_plugin.h
    include/mapviz_plugins/disparity_plugin.h
    include/mapviz_plugins/odometry_plugin.h 
    include/mapviz_plugins/marker_plugin.h 
    include/mapviz_plugins/grid_plugin.h
    include/mapviz_plugins/path_plugin.h
    include/mapviz_plugins/textured_marker_plugin.h
    include/mapviz_plugins/tf_frame_plugin.h
    include/mapviz_plugins/robot_image_plugin.h
    include/mapviz_plugins/gps_plugin.h)

rosbuild_add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARIES} ${GLU_LIBRARY} ${GLUT_LIBRARY})
