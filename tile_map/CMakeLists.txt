cmake_minimum_required(VERSION 2.8.3)
project(tile_map)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  pluginlib 
  tf 
  mapviz
  swri_transform_util
  swri_math_util)

find_package(Qt4 REQUIRED QtCore QtGui QtNetwork QtOpenGl)

find_package(PkgConfig REQUIRED)

find_package(GLU REQUIRED)

pkg_check_modules(JSONCPP REQUIRED jsoncpp)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Qt4
  CATKIN_DEPENDS 
    roscpp 
    pluginlib 
    tf 
    mapviz
    swri_transform_util
    swri_math_util
)

# Fix conflict between Boost signals used by tf and QT signals used by mapviz
add_definitions(-DQT_NO_KEYWORDS)

include(${QT_USE_FILE})
include_directories(include ${catkin_INCLUDE_DIRS} ${QT_INCLUDE_DIR} ${JSONCPP_INCLUDE_DIRS})
add_definitions(${QT_DEFINITIONS})

set(QT_USE_QTCORE TRUE)
set(QT_USE_QTGUI TRUE)
set(QT_USE_QTOPENGL TRUE)
set(QT_USE_NETWORK TRUE)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

file (GLOB TILE_SRC_FILES 
  src/image_cache.cpp
  src/texture_cache.cpp
  src/bing_source.cpp
  src/tile_source.cpp
  src/wmts_source.cpp
  src/tile_map_view.cpp)
QT4_WRAP_CPP(TILE_SRC_FILES
  include/tile_map/image_cache.h
  include/tile_map/tile_source.h
  include/tile_map/wmts_source.h
  include/tile_map/bing_source.h)
add_library(${PROJECT_NAME} ${TILE_SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARIES} ${GLU_LIBRARY} ${JSONCPP_LIBRARIES} ${catkin_LIBRARIES})

file (GLOB PLUGIN_SRC_FILES
  src/tile_map_plugin.cpp)
file (GLOB PLUGIN_UI_FILES
  src/tile_map_config.ui)
QT4_WRAP_UI(PLUGIN_SRC_FILES ${PLUGIN_UI_FILES})
QT4_WRAP_CPP(PLUGIN_SRC_FILES include/tile_map/tile_map_plugin.h)

add_library(${PROJECT_NAME}_plugin ${PLUGIN_SRC_FILES})
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES mapviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

