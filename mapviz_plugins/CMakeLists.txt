cmake_minimum_required(VERSION 2.8.3)
project(mapviz_plugins)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  pluginlib 
  tf 
  std_msgs 
  nav_msgs
  sensor_msgs
  stereo_msgs
  visualization_msgs
  marti_visualization_msgs
  mapviz
  multires_image
  cv_bridge
  transform_util
  math_util
  image_util
  yaml_util)

find_package(Qt4 REQUIRED)

find_package(GLU REQUIRED)
find_package(GLUT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS libqt4
  CATKIN_DEPENDS 
    roscpp 
    pluginlib 
    tf 
    std_msgs 
    nav_msgs
    sensor_msgs
    stereo_msgs
    visualization_msgs
    marti_visualization_msgs
    mapviz
    multires_image
    cv_bridge
    transform_util
    math_util
    image_util
    yaml_util
)

# Fix conflict between Boost signals used by tf and QT signals used by mapviz
add_definitions(-DQT_NO_KEYWORDS)

include(${QT_USE_FILE})
include_directories(include ${catkin_INCLUDE_DIRS} ${QT_INCLUDE_DIR} ${QT_QTOPENGL_INCLUDE_DIR})
add_definitions(${QT_DEFINITIONS})

set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTCORE TRUE)
set(QT_USE_QTGUI TRUE)

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

add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARIES} ${GLU_LIBRARY} ${GLUT_LIBRARY} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES mapviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
