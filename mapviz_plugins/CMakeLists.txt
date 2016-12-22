cmake_minimum_required(VERSION 2.8.3)
project(mapviz_plugins)

set(DEPENDENCIES
  cv_bridge
  gps_common
  image_transport
  mapviz
  marti_common_msgs
  marti_nav_msgs
  marti_visualization_msgs
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
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIR}
    ${PROJECT_BINARY_DIR}
    ${QT_INCLUDE_DIR}
    ${QT_QTOPENGL_INCLUDE_DIR}
)
add_definitions(${QT_DEFINITIONS})

set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTCORE TRUE)
set(QT_USE_QTGUI TRUE)

set(UI_FILES
    ui/attitude_indicator_config.ui
    ui/disparity_config.ui
    ui/draw_polygon_config.ui
    ui/gps_config.ui
    ui/grid_config.ui
    ui/image_config.ui
    ui/laserscan_config.ui
    ui/marker_config.ui
    ui/navsat_config.ui
    ui/odometry_config.ui
    ui/path_config.ui
    ui/plan_route_config.ui
    ui/point_click_publisher_config.ui
    ui/pointcloud2_config.ui
    ui/robot_image_config.ui
    ui/route_config.ui
    ui/string_config.ui
    ui/tf_frame_config.ui
    ui/topic_select.ui
)

set(SRC_FILES
    src/attitude_indicator_plugin.cpp 
    src/canvas_click_filter.cpp
    src/disparity_plugin.cpp
    src/draw_polygon_plugin.cpp
    src/gps_plugin.cpp
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
    src/point_drawing_plugin.cpp
    src/robot_image_plugin.cpp
    src/route_plugin.cpp
    src/string_plugin.cpp
    src/textured_marker_plugin.cpp
    src/tf_frame_plugin.cpp
)

set(HEADER_FILES
    include/${PROJECT_NAME}/attitude_indicator_plugin.h
    include/${PROJECT_NAME}/canvas_click_filter.h
    include/${PROJECT_NAME}/disparity_plugin.h
    include/${PROJECT_NAME}/draw_polygon_plugin.h
    include/${PROJECT_NAME}/gps_plugin.h
    include/${PROJECT_NAME}/grid_plugin.h
    include/${PROJECT_NAME}/image_plugin.h
    include/${PROJECT_NAME}/path_plugin.h
    include/${PROJECT_NAME}/placeable_window_proxy.h
    include/${PROJECT_NAME}/laserscan_plugin.h
    include/${PROJECT_NAME}/marker_plugin.h
    include/${PROJECT_NAME}/navsat_plugin.h
    include/${PROJECT_NAME}/odometry_plugin.h
    include/${PROJECT_NAME}/path_plugin.h
    include/${PROJECT_NAME}/plan_route_plugin.h
    include/${PROJECT_NAME}/point_click_publisher_plugin.h
    include/${PROJECT_NAME}/pointcloud2_plugin.h
    include/${PROJECT_NAME}/robot_image_plugin.h
    include/${PROJECT_NAME}/route_plugin.h
    include/${PROJECT_NAME}/string_plugin.h
    include/${PROJECT_NAME}/textured_marker_plugin.h
    include/${PROJECT_NAME}/tf_frame_plugin.h
    include/${PROJECT_NAME}/point_drawing_plugin.h
)
    
qt4_wrap_ui(UI_SRC_FILES ${UI_FILES})

qt4_wrap_cpp(MOC_FILES ${HEADER_FILES})

add_library(${PROJECT_NAME}
    ${MOC_FILES}
    ${SRC_FILES}
    ${UI_SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GLU_LIBRARY}
    ${GLUT_LIBRARY}
    ${OpenCV_LIBRARIES}
    ${QT_LIBRARIES}
    ${QT_QTOPENGL_LIBRARIES}
)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11 -D__STDC_FORMAT_MACROS")

add_dependencies(${PROJECT_NAME} marti_visualization_msgs_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES ${PROJECT_NAME}.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

