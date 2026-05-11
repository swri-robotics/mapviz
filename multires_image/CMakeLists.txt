cmake_minimum_required(VERSION 3.10...3.17)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(multires_image
  LANGUAGES CXX)

set(MULTIRES_WIDGET_AUTOGEN_INCLUDE_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/multires_widget_autogen/include)
set(MULTIRES_PLUGIN_AUTOGEN_INCLUDE_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_plugin_autogen/include)

find_package(ament_cmake REQUIRED)
find_package(mapviz REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp  REQUIRED)
find_package(tf2 REQUIRED)
find_package(swri_transform_util  REQUIRED)
find_package(swri_math_util REQUIRED)

find_package(Qt5 QUIET COMPONENTS Core Gui OpenGL Widgets)
if(Qt5_FOUND)
  set(MULTIRES_IMAGE_QT_VERSION_MAJOR 5)
else()
  find_package(Qt6
    REQUIRED
    COMPONENTS
      Core
      Gui
      OpenGL
      OpenGLWidgets
      Widgets)
  set(MULTIRES_IMAGE_QT_VERSION_MAJOR 6)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(MULTIRES_IMAGE_QT_LIBRARIES
  Qt${MULTIRES_IMAGE_QT_VERSION_MAJOR}::Core
  Qt${MULTIRES_IMAGE_QT_VERSION_MAJOR}::Gui
  Qt${MULTIRES_IMAGE_QT_VERSION_MAJOR}::OpenGL
  Qt${MULTIRES_IMAGE_QT_VERSION_MAJOR}::Widgets
)

if(MULTIRES_IMAGE_QT_VERSION_MAJOR EQUAL 6)
  list(APPEND MULTIRES_IMAGE_QT_LIBRARIES Qt6::OpenGLWidgets)
  set(MULTIRES_IMAGE_QT_EXPORT_DEPENDENCIES
    Qt6Core
    Qt6Gui
    Qt6OpenGL
    Qt6OpenGLWidgets
    Qt6Widgets)
else()
  set(MULTIRES_IMAGE_QT_EXPORT_DEPENDENCIES
    Qt5Core
    Qt5Gui
    Qt5OpenGL
    Qt5Widgets)
endif()

# Build libtile_cache
set(TILE_SRC_FILES
  src/tile.cpp
  src/tile_cache.cpp
  src/tile_set.cpp
  src/tile_set_layer.cpp
)
set(TILE_HEADER_FILES
  include/${PROJECT_NAME}/tile_cache.hpp
)

add_library(${PROJECT_NAME} 
  ${TILE_SRC_FILES}
  ${TILE_HEADER_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
  AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/src"
  QT_MAJOR_VERSION ${MULTIRES_IMAGE_QT_VERSION_MAJOR})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${mapviz_TARGETS}
  pluginlib::pluginlib
  ${MULTIRES_IMAGE_QT_LIBRARIES}
  rclcpp::rclcpp
  ${swri_math_util_TARGETS}
  ${swri_transform_util_TARGETS}
  tf2::tf2
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  WFlags=WindowFlags
)

# Build libmultires_widget
set(UI_FILES
  src/QGLMap.ui
  src/multires_config.ui
)
set(WIDGET_SRC_FILES
  src/tile_view.cpp
  src/QGLMap.cpp
)
set(WIDGET_HEADER_FILES
  include/${PROJECT_NAME}/QGLMap.hpp
)

add_library(multires_widget
  ${WIDGET_SRC_FILES}
  ${WIDGET_HEADER_FILES}
  src/QGLMap.ui
)
set_target_properties(multires_widget PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
  AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/src"
  QT_MAJOR_VERSION ${MULTIRES_IMAGE_QT_VERSION_MAJOR})
target_include_directories(multires_widget
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${MULTIRES_WIDGET_AUTOGEN_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(multires_widget
  ${PROJECT_NAME}
  ${MULTIRES_IMAGE_QT_LIBRARIES}
)
target_compile_definitions(multires_widget PRIVATE
  WFlags=WindowFlags
)

# Build nodes
set(VIEW_NODE_SRC_FILES
  src/nodes/multires_view_node.cpp
)
set(VIEW_NODE_HEADER_FILES
  include/${PROJECT_NAME}/multires_view_node.hpp
)
add_executable(multires_view_node
  ${VIEW_NODE_SRC_FILES}
  ${VIEW_NODE_HEADER_FILES})
set_target_properties(multires_view_node PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
  AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/src"
  QT_MAJOR_VERSION ${MULTIRES_IMAGE_QT_VERSION_MAJOR})
target_link_libraries(multires_view_node PUBLIC
  multires_widget
  rclcpp::rclcpp
)
target_compile_definitions(multires_view_node PRIVATE
  WFlags=WindowFlags
)

# Build mapviz plugin
set(MAPVIZ_SRC_FILES
  src/${PROJECT_NAME}_plugin.cpp
  src/multires_view.cpp
)
set(MAPVIZ_HEADER_FILES
  include/${PROJECT_NAME}/${PROJECT_NAME}_plugin.hpp
)

add_library(${PROJECT_NAME}_plugin SHARED
  ${MAPVIZ_SRC_FILES}
  ${MAPVIZ_HEADER_FILES}
  src/multires_config.ui
)
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
  AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/src"
  QT_MAJOR_VERSION ${MULTIRES_IMAGE_QT_VERSION_MAJOR})

target_include_directories(${PROJECT_NAME}_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${MULTIRES_PLUGIN_AUTOGEN_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME}_plugin PUBLIC
  ${PROJECT_NAME}
  multires_widget
  ${mapviz_TARGETS}
)
target_compile_definitions(${PROJECT_NAME}_plugin PRIVATE
  WFlags=WindowFlags
)

### Install ${PROJECT_NAME} plugin ###
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

install(FILES
  ${MULTIRES_WIDGET_AUTOGEN_INCLUDE_DIR}/ui_QGLMap.h
  ${MULTIRES_PLUGIN_AUTOGEN_INCLUDE_DIR}/ui_multires_config.h
  DESTINATION include/${PROJECT_NAME}
)

# Install executables/nodes
install(TARGETS multires_view_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install and export libraries
install(TARGETS ${PROJECT_NAME}
  multires_widget
  ${PROJECT_NAME}_plugin
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(PROGRAMS nodes/mapviz_tile_loader
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN ".py" PATTERN ".launch"
)

pluginlib_export_plugin_description_file(mapviz mapviz_plugins.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  mapviz
  pluginlib
  ${MULTIRES_IMAGE_QT_EXPORT_DEPENDENCIES}
  rclcpp
  swri_math_util
  swri_transform_util
  tf2
)
ament_export_include_directories(
  include/${PROJECT_NAME}
)
ament_package()
