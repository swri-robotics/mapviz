cmake_minimum_required(VERSION 3.10)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(mapviz_plugins)

# Ros Packages
find_package(ament_cmake REQUIRED)
# find_package(actionlib REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(gps_common REQUIRED)
find_package(image_transport REQUIRED)
find_package(mapviz REQUIRED)
find_package(marti_common_msgs REQUIRED)
find_package(move_base_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(swri_image_util REQUIRED)
find_package(swri_math_util REQUIRED)
find_package(swri_route_util REQUIRED)
find_package(swri_transform_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

### QT ###
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
set(Qt_FOUND TRUE)
set(Qt_INCLUDE_DIRS "${Qt5Core_INCLUDE_DIRS};${Qt5Gui_INCLUDE_DIRS};${Qt5OpenGL_INCLUDE_DIRS};${Qt5Widgets_INCLUDE_DIRS}")
set(Qt_LIBRARIES "${Qt5Core_LIBRARIES};${Qt5Gui_LIBRARIES};${Qt5OpenGL_LIBRARIES};${Qt5Widgets_LIBRARIES}")
set(Qt_LIBS
    Qt5::Core
    Qt5::Gui
    Qt5::OpenGL
    Qt5::Widgets
    )
add_definitions(-DWFlags=WindowFlags)

find_package(OpenCV 3.1 COMPONENTS core imgproc REQUIRED)

### OpenGL ###
if(${CMAKE_VERSION} VERSION_EQUAL "3.11.0" OR ${CMAKE_VERSION} VERSION_GREATER "3.11.0")
  cmake_policy(SET CMP0072 NEW)
endif()
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   DEPENDS Qt
#   CATKIN_DEPENDS ${DEPENDENCIES}
# )

# Fix conflict between Boost signals used by tf and QT signals used by mapviz
add_definitions(-DQT_NO_KEYWORDS)

# Need to include the current dir to include the results of building Qt UI files
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Qt_INCLUDE_DIRS}
)

set(UI_FILES
    ui/attitude_indicator_config.ui
    ui/coordinate_picker_config.ui
    ui/disparity_config.ui
    ui/draw_polygon_config.ui
    ui/gps_config.ui
    ui/grid_config.ui
    ui/image_config.ui
    ui/laserscan_config.ui
    ui/marker_config.ui
    ui/measuring_config.ui
    ui/move_base_config.ui
    ui/navsat_config.ui
    ui/occupancy_grid_config.ui
    ui/odometry_config.ui
    ui/path_config.ui
    ui/plan_route_config.ui
    ui/point_click_publisher_config.ui
    ui/pointcloud2_config.ui
    ui/pose_config.ui
    ui/robot_image_config.ui
    ui/route_config.ui
    ui/string_config.ui
    ui/textured_marker_config.ui
    ui/tf_frame_config.ui
    ui/topic_select.ui
)

set(SRC_FILES
    src/attitude_indicator_plugin.cpp 
    src/canvas_click_filter.cpp
    src/coordinate_picker_plugin.cpp
    src/disparity_plugin.cpp
    src/draw_polygon_plugin.cpp
    src/gps_plugin.cpp
    src/grid_plugin.cpp
    src/image_plugin.cpp
    src/laserscan_plugin.cpp
    src/marker_plugin.cpp 
    src/measuring_plugin.cpp 
    src/move_base_plugin.cpp
    src/navsat_plugin.cpp
    src/occupancy_grid_plugin.cpp
    src/odometry_plugin.cpp 
    src/path_plugin.cpp
    src/placeable_window_proxy.cpp
    src/plan_route_plugin.cpp
    src/point_click_publisher_plugin.cpp
    src/pointcloud2_plugin.cpp
    src/point_drawing_plugin.cpp
    src/pose_plugin.cpp
    src/robot_image_plugin.cpp
    src/route_plugin.cpp
    src/string_plugin.cpp
    src/textured_marker_plugin.cpp
    src/tf_frame_plugin.cpp
)

set(HEADER_FILES
    include/${PROJECT_NAME}/attitude_indicator_plugin.h
    include/${PROJECT_NAME}/canvas_click_filter.h
    include/${PROJECT_NAME}/coordinate_picker_plugin.h
    include/${PROJECT_NAME}/disparity_plugin.h
    include/${PROJECT_NAME}/draw_polygon_plugin.h
    include/${PROJECT_NAME}/gps_plugin.h
    include/${PROJECT_NAME}/grid_plugin.h
    include/${PROJECT_NAME}/image_plugin.h
    include/${PROJECT_NAME}/laserscan_plugin.h
    include/${PROJECT_NAME}/marker_plugin.h
    include/${PROJECT_NAME}/measuring_plugin.h
    include/${PROJECT_NAME}/move_base_plugin.h
    include/${PROJECT_NAME}/navsat_plugin.h
    include/${PROJECT_NAME}/occupancy_grid_plugin.h
    include/${PROJECT_NAME}/odometry_plugin.h
    include/${PROJECT_NAME}/path_plugin.h
    include/${PROJECT_NAME}/path_plugin.h
    include/${PROJECT_NAME}/placeable_window_proxy.h
    include/${PROJECT_NAME}/plan_route_plugin.h
    include/${PROJECT_NAME}/point_click_publisher_plugin.h
    include/${PROJECT_NAME}/pointcloud2_plugin.h
    include/${PROJECT_NAME}/point_drawing_plugin.h
    include/${PROJECT_NAME}/pose_plugin.h
    include/${PROJECT_NAME}/robot_image_plugin.h
    include/${PROJECT_NAME}/route_plugin.h
    include/${PROJECT_NAME}/string_plugin.h
    include/${PROJECT_NAME}/textured_marker_plugin.h
    include/${PROJECT_NAME}/tf_frame_plugin.h
)
    
qt5_wrap_ui(UI_SRC_FILES ${UI_FILES})
qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})

add_library(${PROJECT_NAME}
    ${MOC_FILES}
    ${SRC_FILES}
    ${UI_SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GLU_LIBRARY}
    ${GLUT_LIBRARY}
    ${OpenCV_LIBS}
    ${Qt_LIBRARIES}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS "-std=c++11 -D__STDC_FORMAT_MACROS"
)


### Install the plugins ###
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# install(TARGETS ${PROJECT_NAME}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

install(TARGETS ${PROJECT_NAME} rqt_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# install(FILES ${PROJECT_NAME}.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${RUNTIME_DEPS} Qt)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
