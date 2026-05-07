cmake_minimum_required(VERSION 3.10...3.17)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(tile_map
  LANGUAGES CXX)

function(tile_map_ensure_interface_alias alias_target dependency_target)
  if(TARGET ${alias_target})
    return()
  endif()

  add_library(${alias_target} INTERFACE IMPORTED)
  set_target_properties(${alias_target} PROPERTIES
    INTERFACE_LINK_LIBRARIES ${dependency_target})
endfunction()

# ROS packages
find_package(ament_cmake REQUIRED)
find_package(mapviz REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swri_math_util REQUIRED)
find_package(swri_transform_util REQUIRED)
find_package(tf2 REQUIRED)

find_package(Qt5 QUIET COMPONENTS Core Gui Network OpenGL Widgets)
if(Qt5_FOUND)
  set(TILE_MAP_QT_VERSION_MAJOR 5)
else()
  find_package(Qt6 REQUIRED COMPONENTS Core Gui Network OpenGL Widgets)
  set(TILE_MAP_QT_VERSION_MAJOR 6)
  find_package(Qt6 REQUIRED COMPONENTS OpenGLWidgets)
endif()

find_package(jsoncpp QUIET CONFIG)
if(NOT TARGET jsoncpp_lib)
  if(TARGET JsonCpp::JsonCpp)
    tile_map_ensure_interface_alias(jsoncpp_lib JsonCpp::JsonCpp)
  else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)
    tile_map_ensure_interface_alias(jsoncpp_lib PkgConfig::JSONCPP)
  endif()
endif()

find_package(yaml-cpp QUIET CONFIG)
if(NOT TARGET yaml-cpp)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(YamlCpp REQUIRED IMPORTED_TARGET yaml-cpp)
  tile_map_ensure_interface_alias(yaml-cpp PkgConfig::YamlCpp)
endif()

# Need to include the current dir to include the results of building Qt UI files
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(TILE_SRC_FILES
  src/image_cache.cpp
  src/texture_cache.cpp
  src/bing_source.cpp
  src/stadia_source.cpp
  src/tile_source.cpp
  src/wmts_source.cpp
  src/${PROJECT_NAME}_view.cpp
)
set(QT_HEADERS
  include/${PROJECT_NAME}/image_cache.hpp
  include/${PROJECT_NAME}/tile_source.hpp
  include/${PROJECT_NAME}/wmts_source.hpp
  include/${PROJECT_NAME}/bing_source.hpp
  include/${PROJECT_NAME}/stadia_source.hpp
)

set(TILE_MAP_QT_LIBRARIES
  Qt${TILE_MAP_QT_VERSION_MAJOR}::Network
  Qt${TILE_MAP_QT_VERSION_MAJOR}::Core
  Qt${TILE_MAP_QT_VERSION_MAJOR}::Gui
  Qt${TILE_MAP_QT_VERSION_MAJOR}::OpenGL
  Qt${TILE_MAP_QT_VERSION_MAJOR}::Widgets
)

if(TILE_MAP_QT_VERSION_MAJOR EQUAL 6)
  list(APPEND TILE_MAP_QT_LIBRARIES Qt6::OpenGLWidgets)
  set(TILE_MAP_QT_EXPORT_DEPENDENCIES
    Qt6Network
    Qt6Core
    Qt6Gui
    Qt6OpenGL
    Qt6OpenGLWidgets
    Qt6Widgets)
else()
  set(TILE_MAP_QT_EXPORT_DEPENDENCIES
    Qt5Network
    Qt5Core
    Qt5Gui
    Qt5OpenGL
    Qt5Widgets)
endif()

add_library(${PROJECT_NAME} ${TILE_SRC_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
  AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/src"
  QT_MAJOR_VERSION ${TILE_MAP_QT_VERSION_MAJOR})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
    
target_link_libraries(${PROJECT_NAME} PUBLIC
  jsoncpp_lib
  ${mapviz_TARGETS}
  ${TILE_MAP_QT_LIBRARIES}
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${swri_transform_util_TARGETS}
  tf2::tf2
  yaml-cpp)

set(PLUGIN_SRC_FILES
  src/${PROJECT_NAME}_plugin.cpp)
set(PLUGIN_UI_FILES
  src/${PROJECT_NAME}_config.ui)

list(APPEND PLUGIN_SRC_FILES ${PLUGIN_UI_FILES} include/${PROJECT_NAME}/${PROJECT_NAME}_plugin.hpp)

add_library(${PROJECT_NAME}_plugin SHARED ${PLUGIN_SRC_FILES})
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES
  AUTOMOC ON
  AUTOUIC ON
  AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/src"
  QT_MAJOR_VERSION ${TILE_MAP_QT_VERSION_MAJOR})
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

pluginlib_export_plugin_description_file(mapviz mapviz_plugins.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  mapviz
  pluginlib
  rclcpp
  swri_transform_util
  tf2
  ${TILE_MAP_QT_EXPORT_DEPENDENCIES}
)
ament_package(CONFIG_EXTRAS cmake/tile_map-extras.cmake)
