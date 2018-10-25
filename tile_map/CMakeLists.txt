cmake_minimum_required(VERSION 2.8.3)
project(tile_map)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  pluginlib 
  tf 
  mapviz
  swri_transform_util
  swri_math_util)

### QT ###
if("$ENV{ROS_DISTRO}" STRLESS "kinetic")
  find_package(Qt4 COMPONENTS REQUIRED QtCore QtGui QtOpenGL QtNetwork)
  set(Qt_FOUND TRUE)
  set(Qt_INCLUDE_DIRS "${QT_INCLUDE_DIR};${QT_QTGUI_INCLUDE_DIR};${GT_QTOPENGL_INCLUDE_DIR};${QT_QTNETWORK_LIBRARY}")
  set(Qt_LIBRARIES "${QT_QTCORE_LIBRARY};${QT_QTGUI_LIBRARY};${QT_QTOPENGL_LIBRARY};${QT_QTNETWORK_LIBRARY}")
  set(Qt_LIBS
      Qt4::QtCore
      Qt4::QtGui
      Qt4::QtOpenGL
      Qt4::QtNetwork
      )
else()
  find_package(Qt5Network REQUIRED)
  find_package(Qt5Core REQUIRED)
  find_package(Qt5Gui REQUIRED)
  find_package(Qt5OpenGL REQUIRED)
  find_package(Qt5Widgets REQUIRED)
  set(Qt_FOUND TRUE)
  set(Qt_INCLUDE_DIRS "${Qt5Network_INCLUDE_DIRS};${Qt5Core_INCLUDE_DIRS};${Qt5Gui_INCLUDE_DIRS};${Qt5OpenGL_INCLUDE_DIRS};${Qt5Widgets_INCLUDE_DIRS}")
  set(Qt_LIBRARIES "${Qt5Network_LIBRARIES};${Qt5Core_LIBRARIES};${Qt5Gui_LIBRARIES};${Qt5OpenGL_LIBRARIES};${Qt5Widgets_LIBRARIES}")
  set(Qt_LIBS
      Qt5::Network
      Qt5::Core
      Qt5::Gui
      Qt5::OpenGL
      Qt5::Widgets
      )
  add_definitions(-DWFlags=WindowFlags)
endif()
#find_package(Qt4 REQUIRED QtCore QtGui QtNetwork QtOpenGl)

find_package(PkgConfig REQUIRED)

find_package(GLU REQUIRED)

pkg_check_modules(JSONCPP REQUIRED jsoncpp)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Qt
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

#include(${QT_USE_FILE})
include_directories(include ${catkin_INCLUDE_DIRS} ${Qt_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})
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
if("$ENV{ROS_DISTRO}" STRLESS "kinetic")
QT4_WRAP_CPP(TILE_SRC_FILES
  include/tile_map/image_cache.h
  include/tile_map/tile_source.h
  include/tile_map/wmts_source.h
  include/tile_map/bing_source.h)
else()
QT5_WRAP_CPP(TILE_SRC_FILES
  include/tile_map/image_cache.h
  include/tile_map/tile_source.h
  include/tile_map/wmts_source.h
  include/tile_map/bing_source.h)
endif()
add_library(${PROJECT_NAME} ${TILE_SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${Qt_LIBRARIES} ${GLU_LIBRARY} ${JSONCPP_LIBRARIES} ${catkin_LIBRARIES})

file (GLOB PLUGIN_SRC_FILES
  src/tile_map_plugin.cpp)
file (GLOB PLUGIN_UI_FILES
  src/tile_map_config.ui)
if("$ENV{ROS_DISTRO}" STRLESS "kinetic")
QT4_WRAP_UI(PLUGIN_SRC_FILES ${PLUGIN_UI_FILES})
QT4_WRAP_CPP(PLUGIN_SRC_FILES include/tile_map/tile_map_plugin.h)
else()
QT5_WRAP_UI(PLUGIN_SRC_FILES ${PLUGIN_UI_FILES})
QT5_WRAP_CPP(PLUGIN_SRC_FILES include/tile_map/tile_map_plugin.h)
endif()

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

