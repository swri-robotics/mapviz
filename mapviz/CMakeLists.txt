### SET CMAKE POLICIES ###
cmake_minimum_required(VERSION 3.10...3.17)

if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(${CMAKE_VERSION} VERSION_EQUAL "3.11.0" OR ${CMAKE_VERSION} VERSION_GREATER "3.11.0")
  cmake_policy(SET CMP0072 NEW)
endif()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
### END CMAKE POLICIES ###

project(mapviz 
  DESCRIPTION "Mapviz is a ROS based visualization tool with a plug-in system similar to RVIZ focused on visualizing 2D data."
  LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(image_transport REQUIRED)
find_package(mapviz_interfaces REQUIRED)
find_package(marti_common_msgs REQUIRED)
find_package(OpenCV COMPONENTS core imgproc imgcodecs videoio REQUIRED)
find_package(OpenGL REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(swri_math_util REQUIRED)
find_package(swri_route_util REQUIRED)
find_package(swri_transform_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

### PkgConfig ###
find_package(PkgConfig REQUIRED)
pkg_check_modules(YamlCpp yaml-cpp)

# ### Qt5 on ROS2 ###
add_definitions(-DWFlags=WindowFlags)
set(QT_USE_QTOPENGL TRUE)

# Need to include the current dir to include the results of building Qt UI files
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Qt UI files
set(UI_FILES
  ui/configitem.ui
  ui/mapviz.ui
  ui/pluginselect.ui
)
# Headers containing QObject definitions
set(HEADER_FILES
  include/${PROJECT_NAME}/color_button.h
  include/${PROJECT_NAME}/config_item.h
  include/${PROJECT_NAME}/map_canvas.h
  include/${PROJECT_NAME}/${PROJECT_NAME}.hpp
  include/${PROJECT_NAME}/${PROJECT_NAME}_plugin.h
  include/${PROJECT_NAME}/rqt_${PROJECT_NAME}.h
  include/${PROJECT_NAME}/select_frame_dialog.h
  include/${PROJECT_NAME}/select_service_dialog.h
  include/${PROJECT_NAME}/video_writer.h
  include/${PROJECT_NAME}/widgets.h
)
# Source files for mapviz
set(SRC_FILES
  src/${PROJECT_NAME}.cpp
  src/color_button.cpp
  src/config_item.cpp
  src/${PROJECT_NAME}_application.cpp
  src/map_canvas.cpp
  src/rqt_${PROJECT_NAME}.cpp
  src/select_frame_dialog.cpp
  src/select_service_dialog.cpp
  src/video_writer.cpp
)

### Qt5 on ROS2 ###
qt5_add_resources(RCC_SRCS src/resources/icons.qrc)
qt5_wrap_ui(UI_SRC_FILES ${UI_FILES})
qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})

### Build mapviz as an rqt plugin ###
add_library(rqt_${PROJECT_NAME}
  ${MOC_FILES}
  ${SRC_FILES}
  ${UI_SRC_FILES}
  ${RCC_SRCS}
)
target_include_directories(rqt_${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(rqt_${PROJECT_NAME} PUBLIC
  cv_bridge::cv_bridge
  OpenGL::GL
  ${geometry_msgs_TARGETS}
  GLEW::GLEW
  ${GLUT_LIBRARIES}
  image_transport::image_transport
  opencv_core
  opencv_highgui
  opencv_imgproc
  opencv_imgcodecs
  opencv_videoio
  ${mapviz_interfaces_TARGETS}
  ${marti_common_msgs_TARGETS}
  pluginlib::pluginlib
  Qt5::Concurrent
  Qt5::Core
  Qt5::Gui
  Qt5::OpenGL
  Qt5::Widgets
  rclcpp::rclcpp
  rqt_gui_cpp::rqt_gui_cpp
  ${std_srvs_TARGETS}
  ${swri_math_util_TARGETS}
  ${swri_route_util_TARGETS}
  ${swri_transform_util_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${YamlCpp_LIBRARIES}
)
set_target_properties(rqt_${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS "-D__STDC_FORMAT_MACROS")

### Build mapviz as a standalone executable ###
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_main.cpp)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rqt_${PROJECT_NAME}
  ${GLU_LIBRARY}
  ${swri_math_util_TARGETS}
  ${swri_transform_util_TARGETS})

set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS "-D__STDC_FORMAT_MACROS")

### Install mapviz ###
install(DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS rqt_${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME} 
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${RUNTIME_DEPS}
  GLEW
  image_transport
  Qt5Concurrent
  Qt5Core
  Qt5Gui
  Qt5OpenGL
  Qt5Widgets
  mapviz_interfaces
  rqt_gui_cpp
  std_srvs
  swri_route_util
)
ament_package()
