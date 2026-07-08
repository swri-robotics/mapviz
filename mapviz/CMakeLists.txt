cmake_minimum_required(VERSION 3.10...3.17)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(mapviz 
  DESCRIPTION "Mapviz is a ROS based visualization tool with a plug-in system similar to RVIZ focused on visualizing 2D data."
  LANGUAGES CXX)

function(mapviz_ensure_interface_alias alias_target dependency_target)
  if(TARGET ${alias_target})
    return()
  endif()

  add_library(${alias_target} INTERFACE IMPORTED)
  set_target_properties(${alias_target} PROPERTIES
    INTERFACE_LINK_LIBRARIES ${dependency_target})
endfunction()

# Ensures that the OpenCV component target exists by finding it from various sources
# (direct target, OpenCV namespace target, or pkg-config) and creating an alias if needed
function(mapviz_ensure_opencv_component component)
  if(TARGET opencv_${component})
    return()
  endif()

  if(TARGET OpenCV::opencv_${component})
    mapviz_ensure_interface_alias(opencv_${component} OpenCV::opencv_${component})
    return()
  endif()

  if(TARGET PkgConfig::OpenCV)
    mapviz_ensure_interface_alias(opencv_${component} PkgConfig::OpenCV)
    return()
  endif()

  message(FATAL_ERROR "Unable to resolve OpenCV component target opencv_${component}")
endfunction()

set(MAPVIZ_AUTOGEN_INCLUDE_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_core_autogen/include)
set(MAPVIZ_AUTOGEN_UI_INCLUDE_DIR
  ${MAPVIZ_AUTOGEN_INCLUDE_DIR}/ui)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(qt_gui_cpp REQUIRED)
set(QT_VERSION_MAJOR "${qt_gui_cpp_USE_QT_MAJOR_VERSION}")
find_package(Qt${qt_gui_cpp_USE_QT_MAJOR_VERSION}
  REQUIRED
  COMPONENTS
    Concurrent
    Core
    Gui
    OpenGL
    Widgets
)

if(QT_VERSION_MAJOR EQUAL 6)
  find_package(Qt6
    REQUIRED
    COMPONENTS
      OpenGLWidgets
  )
endif()

find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(mapviz_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(swri_math_util REQUIRED)
find_package(swri_transform_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenCV QUIET CONFIG COMPONENTS core highgui imgcodecs imgproc videoio)
if(NOT OpenCV_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(OpenCV REQUIRED IMPORTED_TARGET opencv4)
endif()
foreach(component IN ITEMS core highgui imgcodecs imgproc videoio)
  mapviz_ensure_opencv_component(${component})
endforeach()

find_package(yaml-cpp QUIET CONFIG)
if(NOT TARGET yaml-cpp)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(YamlCpp REQUIRED IMPORTED_TARGET yaml-cpp)
  mapviz_ensure_interface_alias(yaml-cpp PkgConfig::YamlCpp)
endif()

# Need to include the current dir to include the results of building Qt UI files
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

# Qt UI files
set(UI_FILES
  ui/configitem.ui
  ui/mapviz.ui
  ui/pluginselect.ui
)
# Headers containing QObject definitions (core library)
set(HEADER_FILES
  include/${PROJECT_NAME}/color_button.hpp
  include/${PROJECT_NAME}/config_item.hpp
  include/${PROJECT_NAME}/map_canvas.hpp
  include/${PROJECT_NAME}/${PROJECT_NAME}.hpp
  include/${PROJECT_NAME}/${PROJECT_NAME}_plugin.hpp
  include/${PROJECT_NAME}/select_frame_dialog.hpp
  include/${PROJECT_NAME}/select_service_dialog.hpp
  include/${PROJECT_NAME}/video_writer.hpp
  include/${PROJECT_NAME}/widgets.hpp
)
# Source files for mapviz core library
set(SRC_FILES
  src/${PROJECT_NAME}.cpp
  src/color_button.cpp
  src/config_item.cpp
  src/${PROJECT_NAME}_application.cpp
  src/map_canvas.cpp
  src/select_frame_dialog.cpp
  src/select_service_dialog.cpp
  src/video_writer.cpp
  src/resources/icons.qrc
)

set(MAPVIZ_QT_LIBRARIES
  Qt${QT_VERSION_MAJOR}::Concurrent
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::Gui
  Qt${QT_VERSION_MAJOR}::OpenGL
  Qt${QT_VERSION_MAJOR}::Widgets
)

if(QT_VERSION_MAJOR EQUAL 6)
  list(APPEND MAPVIZ_QT_LIBRARIES Qt${QT_VERSION_MAJOR}::OpenGLWidgets)
endif()

set(MAPVIZ_QT_EXPORT_DEPENDENCIES
  Qt${QT_VERSION_MAJOR}Concurrent
  Qt${QT_VERSION_MAJOR}Core
  Qt${QT_VERSION_MAJOR}Gui
  Qt${QT_VERSION_MAJOR}OpenGL
  Qt${QT_VERSION_MAJOR}Widgets)

if(QT_VERSION_MAJOR EQUAL 6)
  list(APPEND MAPVIZ_QT_EXPORT_DEPENDENCIES Qt${QT_VERSION_MAJOR}OpenGLWidgets)
endif()

### Build mapviz core library ###
add_library(${PROJECT_NAME}_core SHARED
  ${HEADER_FILES}
  ${SRC_FILES}
  ${UI_FILES}
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${MAPVIZ_AUTOGEN_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${MAPVIZ_AUTOGEN_UI_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME}_core PUBLIC
  ${geometry_msgs_TARGETS}
  opencv_core
  opencv_highgui
  ${mapviz_interfaces_TARGETS}
  pluginlib::pluginlib
  ${MAPVIZ_QT_LIBRARIES}
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
  ${swri_transform_util_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  yaml-cpp
)
target_link_libraries(${PROJECT_NAME}_core PRIVATE
  image_transport::image_transport
  opencv_imgcodecs
  opencv_imgproc
  opencv_videoio
  ${swri_math_util_TARGETS}
)
target_compile_definitions(${PROJECT_NAME}_core PRIVATE
  WFlags=WindowFlags
  __STDC_FORMAT_MACROS
)

set_target_properties(${PROJECT_NAME}_core PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
  AUTORCC ON
  AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/ui"
)

### Build rqt plugin (thin wrapper, loaded dynamically by rqt) ###
add_library(rqt_${PROJECT_NAME} SHARED
  include/${PROJECT_NAME}/rqt_${PROJECT_NAME}.hpp
  src/rqt_${PROJECT_NAME}.cpp
)
target_link_libraries(rqt_${PROJECT_NAME} PUBLIC
  ${PROJECT_NAME}_core
  rqt_gui_cpp::rqt_gui_cpp
)
set_target_properties(rqt_${PROJECT_NAME} PROPERTIES
  AUTOMOC ON
)

### Build mapviz as a standalone executable ###
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE
  ${PROJECT_NAME}_core
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  __STDC_FORMAT_MACROS
)

### Install mapviz ###
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

install(FILES
  ${MAPVIZ_AUTOGEN_UI_INCLUDE_DIR}/ui_configitem.h
  ${MAPVIZ_AUTOGEN_UI_INCLUDE_DIR}/ui_mapviz.h
  ${MAPVIZ_AUTOGEN_UI_INCLUDE_DIR}/ui_pluginselect.h
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(TARGETS rqt_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME} 
)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  image_transport
  pluginlib
  ${MAPVIZ_QT_EXPORT_DEPENDENCIES}
  mapviz_interfaces
  rclcpp
  std_srvs
  swri_transform_util
  tf2
  tf2_geometry_msgs
  tf2_ros
)
ament_package(CONFIG_EXTRAS cmake/mapviz-extras.cmake)
