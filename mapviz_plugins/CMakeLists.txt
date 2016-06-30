cmake_minimum_required(VERSION 2.8.3)
project(mapviz_plugins)

set(DEPENDENCIES
  cv_bridge
  image_transport
  mapviz
  marti_visualization_msgs
  marti_common_msgs
  marti_nav_msgs
  nav_msgs
  pluginlib 
  roscpp 
  sensor_msgs
  std_msgs 
  stereo_msgs
  swri_image_util
  swri_math_util
  swri_route_util
  swri_transform_util
  swri_yaml_util
  tf 
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS ${DEPENDENCIES})

find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
set(QT_USE_QTOPENGL TRUE)
find_package(OpenCV 2 REQUIRED)
find_package(GLU REQUIRED)
find_package(GLUT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Qt4
  CATKIN_DEPENDS ${DEPENDENCIES}
)

# Fix conflict between Boost signals used by tf and QT signals used by mapviz
add_definitions(-DQT_NO_KEYWORDS)

include(${QT_USE_FILE})
include_directories(include ${catkin_INCLUDE_DIRS} ${QT_INCLUDE_DIR} ${QT_QTOPENGL_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR})
add_definitions(${QT_DEFINITIONS})

set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTCORE TRUE)
set(QT_USE_QTGUI TRUE)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
file (GLOB UI_FILES src/*.ui)

file (GLOB SRC_FILES
    src/attitude_indicator_plugin.cpp
    src/canvas_click_filter.cpp
    src/disparity_plugin.cpp 
    src/grid_plugin.cpp
    src/image_plugin.cpp
    src/laserscan_plugin.cpp
    src/marker_plugin.cpp 
    src/navsat_plugin.cpp
    src/odometry_plugin.cpp 
    src/path_plugin.cpp
    src/placeable_window_proxy.cpp
    src/plan_route_plugin.cpp
    src/point_click_publisher_plugin.cpp
    src/pointcloud2_plugin.cpp
    src/robot_image_plugin.cpp
    src/route_plugin.cpp
    src/string_plugin.cpp
    src/textured_marker_plugin.cpp
    src/tf_frame_plugin.cpp
    src/point_drawing_plugin.cpp
)
    
QT4_WRAP_UI(SRC_FILES ${UI_FILES})

QT4_WRAP_CPP(SRC_FILES
    include/mapviz_plugins/attitude_indicator_plugin.h
    include/mapviz_plugins/canvas_click_filter.h
    include/mapviz_plugins/disparity_plugin.h
    include/mapviz_plugins/grid_plugin.h
    include/mapviz_plugins/image_plugin.h
    include/mapviz_plugins/laserscan_plugin.h
    include/mapviz_plugins/marker_plugin.h 
    include/mapviz_plugins/navsat_plugin.h
    include/mapviz_plugins/odometry_plugin.h 
    include/mapviz_plugins/path_plugin.h
    include/mapviz_plugins/placeable_window_proxy.h
    include/mapviz_plugins/plan_route_plugin.h    
    include/mapviz_plugins/point_click_publisher_plugin.h
    include/mapviz_plugins/pointcloud2_plugin.h
    include/mapviz_plugins/robot_image_plugin.h
    include/mapviz_plugins/route_plugin.h
    include/mapviz_plugins/string_plugin.h
    include/mapviz_plugins/textured_marker_plugin.h
    include/mapviz_plugins/tf_frame_plugin.h
    include/mapviz_plugins/point_drawing_plugin.h
)

add_library(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARIES} ${GLU_LIBRARY} ${GLUT_LIBRARY} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} marti_visualization_msgs_generate_messages_cpp)

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

