cmake_minimum_required(VERSION 3.10...3.17)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(mapviz_plugins
  LANGUAGES CXX)

function(mapviz_plugins_ensure_interface_alias alias_target dependency_target)
  if(TARGET ${alias_target})
    return()
  endif()

  add_library(${alias_target} INTERFACE IMPORTED)
  set_target_properties(${alias_target} PROPERTIES
    INTERFACE_LINK_LIBRARIES ${dependency_target})
endfunction()

function(mapviz_plugins_ensure_opencv_component component)
  if(TARGET opencv_${component})
    return()
  endif()

  if(TARGET OpenCV::opencv_${component})
    mapviz_plugins_ensure_interface_alias(opencv_${component} OpenCV::opencv_${component})
    return()
  endif()

  if(TARGET PkgConfig::OpenCV)
    mapviz_plugins_ensure_interface_alias(opencv_${component} PkgConfig::OpenCV)
    return()
  endif()

  message(FATAL_ERROR "Unable to resolve OpenCV component target opencv_${component}")
endfunction()

set(MAPVIZ_PLUGINS_AUTOGEN_INCLUDE_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_autogen/include)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(map_msgs REQUIRED)
find_package(mapviz REQUIRED)
find_package(marti_common_msgs REQUIRED)
find_package(marti_nav_msgs REQUIRED)
find_package(marti_sensor_msgs REQUIRED)
find_package(marti_visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(swri_image_util REQUIRED)
find_package(swri_math_util REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(swri_route_util REQUIRED)
find_package(swri_transform_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(assimp REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

### QT ###
set(QT_VERSION_MAJOR "${qt_gui_cpp_USE_QT_MAJOR_VERSION}")
set(MAPVIZ_PLUGINS_QT_COMPONENTS Core Gui OpenGL Widgets)
if(QT_VERSION_MAJOR EQUAL 6)
  list(APPEND MAPVIZ_PLUGINS_QT_COMPONENTS OpenGLWidgets)
endif()
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS ${MAPVIZ_PLUGINS_QT_COMPONENTS})

find_package(OpenCV QUIET CONFIG COMPONENTS core imgproc highgui)
if(NOT OpenCV_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(OpenCV REQUIRED IMPORTED_TARGET opencv4)
endif()
foreach(component IN ITEMS core imgproc highgui)
  mapviz_plugins_ensure_opencv_component(${component})
endforeach()

# Need to include the current dir to include the results of building Qt UI files
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)
set(UI_FILES
  ui/attitude_indicator_config.ui
  ui/coordinate_picker_config.ui
  ui/disparity_config.ui
  ui/draw_polygon_config.ui
  ui/float_config.ui
  ui/gps_config.ui
  ui/grid_config.ui
  ui/image_config.ui
  ui/laserscan_config.ui
  ui/marker_config.ui
  ui/measuring_config.ui
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
  ui/robot_model_config.ui
  ui/tf_frame_config.ui
  ui/topicselect.ui)

set(SRC_FILES
  src/attitude_indicator_plugin.cpp
  src/canvas_click_filter.cpp
  src/coordinate_picker_plugin.cpp
  src/disparity_plugin.cpp
  src/draw_polygon_plugin.cpp
  src/float_plugin.cpp
  src/gps_plugin.cpp
  src/grid_plugin.cpp
  src/image_plugin.cpp
  src/laserscan_plugin.cpp
  src/marker_plugin.cpp
  src/measuring_plugin.cpp
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
  src/robot_model_plugin.cpp
  src/robot_image_plugin.cpp
  src/route_plugin.cpp
  src/string_plugin.cpp
  src/textured_marker_plugin.cpp
  src/tf_frame_plugin.cpp
  src/topic_select.cpp)

set(HEADER_FILES
  include/${PROJECT_NAME}/attitude_indicator_plugin.hpp
  include/${PROJECT_NAME}/canvas_click_filter.hpp
  include/${PROJECT_NAME}/coordinate_picker_plugin.hpp
  include/${PROJECT_NAME}/disparity_plugin.hpp
  include/${PROJECT_NAME}/draw_polygon_plugin.hpp
  include/${PROJECT_NAME}/float_plugin.hpp
  include/${PROJECT_NAME}/gps_plugin.hpp
  include/${PROJECT_NAME}/grid_plugin.hpp
  include/${PROJECT_NAME}/image_plugin.hpp
  include/${PROJECT_NAME}/laserscan_plugin.hpp
  include/${PROJECT_NAME}/marker_plugin.hpp
  include/${PROJECT_NAME}/measuring_plugin.hpp
  include/${PROJECT_NAME}/navsat_plugin.hpp
  include/${PROJECT_NAME}/occupancy_grid_plugin.hpp
  include/${PROJECT_NAME}/odometry_plugin.hpp
  include/${PROJECT_NAME}/path_plugin.hpp
  include/${PROJECT_NAME}/placeable_window_proxy.hpp
  include/${PROJECT_NAME}/plan_route_plugin.hpp
  include/${PROJECT_NAME}/point_click_publisher_plugin.hpp
  include/${PROJECT_NAME}/pointcloud2_plugin.hpp
  include/${PROJECT_NAME}/point_drawing_plugin.hpp
  include/${PROJECT_NAME}/pose_plugin.hpp
  include/${PROJECT_NAME}/robot_model_plugin.hpp
  include/${PROJECT_NAME}/robot_image_plugin.hpp
  include/${PROJECT_NAME}/route_plugin.hpp
  include/${PROJECT_NAME}/string_plugin.hpp
  include/${PROJECT_NAME}/textured_marker_plugin.hpp
  include/${PROJECT_NAME}/tf_frame_plugin.hpp
  include/${PROJECT_NAME}/topic_select.hpp)

set(MAPVIZ_PLUGINS_QT_LIBRARIES
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::Gui
  Qt${QT_VERSION_MAJOR}::OpenGL
  Qt${QT_VERSION_MAJOR}::Widgets
)

if(QT_VERSION_MAJOR EQUAL 6)
  list(APPEND MAPVIZ_PLUGINS_QT_LIBRARIES Qt${QT_VERSION_MAJOR}::OpenGLWidgets)
endif()
set(MAPVIZ_PLUGINS_QT_EXPORT_DEPENDENCIES
  Qt${QT_VERSION_MAJOR}Core
  Qt${QT_VERSION_MAJOR}Gui
  Qt${QT_VERSION_MAJOR}OpenGL
  Qt${QT_VERSION_MAJOR}Widgets)

if(QT_VERSION_MAJOR EQUAL 6)
  list(APPEND MAPVIZ_PLUGINS_QT_EXPORT_DEPENDENCIES Qt${QT_VERSION_MAJOR}OpenGLWidgets)
endif()

add_library(${PROJECT_NAME} SHARED
    ${HEADER_FILES}
    ${SRC_FILES}
    ${UI_FILES}
)
set_target_properties(${PROJECT_NAME} PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
  AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/ui"
  QT_MAJOR_VERSION ${QT_VERSION_MAJOR})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PACKAGE_NAME}>)

target_compile_definitions(${PROJECT_NAME}
  PUBLIC
    "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME}
  PRIVATE
    WFlags=WindowFlags)

# Iron and later switched some cv_bridge files to .hpp from .h
if ("$ENV{ROS_DISTRO}" STRLESS "iron")
  target_compile_definitions(${PROJECT_NAME} PRIVATE USE_CVBRIDGE_H_FILES)
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC
    ament_index_cpp::ament_index_cpp
    cv_bridge::cv_bridge
    ${gps_msgs_TARGETS}
    image_transport::image_transport
    ${map_msgs_TARGETS}
    ${mapviz_TARGETS}
    ${marti_common_msgs_TARGETS}
    ${marti_nav_msgs_TARGETS}
    ${marti_sensor_msgs_TARGETS}
    ${marti_visualization_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    opencv_core
    opencv_imgproc
    opencv_highgui
    pluginlib::pluginlib
    ${MAPVIZ_PLUGINS_QT_LIBRARIES}
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    sensor_msgs::sensor_msgs_library
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${stereo_msgs_TARGETS}
    ${swri_image_util_TARGETS}
    ${swri_transform_util_TARGETS}
    ${swri_route_util_TARGETS}
    tf2::tf2
    tf2_ros::tf2_ros
    ${urdf_TARGETS}
    ${visualization_msgs_TARGETS})

target_link_libraries(${PROJECT_NAME} PRIVATE assimp::assimp)

### Install the plugins ###
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp")

install(DIRECTORY ${MAPVIZ_PLUGINS_AUTOGEN_INCLUDE_DIR}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "ui_*.h")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

pluginlib_export_plugin_description_file(mapviz mapviz_plugins.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  cv_bridge
  geometry_msgs
  gps_msgs
  image_transport
  map_msgs
  mapviz
  marti_common_msgs
  marti_nav_msgs
  marti_sensor_msgs
  marti_visualization_msgs
  nav_msgs
  pluginlib
  ${MAPVIZ_PLUGINS_QT_EXPORT_DEPENDENCIES}
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  stereo_msgs
  swri_image_util
  swri_route_util
  swri_transform_util
  tf2
  tf2_ros
  urdf
  visualization_msgs)
ament_package(CONFIG_EXTRAS cmake/mapviz_plugins-extras.cmake)
