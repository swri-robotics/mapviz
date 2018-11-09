cmake_minimum_required(VERSION 2.8.3)
project(multires_image)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  mapviz
  roscpp 
  tf 
  swri_transform_util 
  swri_math_util
  swri_yaml_util
)

find_package(Boost REQUIRED thread)
find_package(GLU REQUIRED)

if("$ENV{ROS_DISTRO}" STRLESS "kinetic")
  find_package(OpenCV 2 REQUIRED)
else()
  find_package(OpenCV 3.1 COMPONENTS core imgproc REQUIRED)
endif()

### QT ###
if("$ENV{ROS_DISTRO}" STRLESS "kinetic")
  find_package(Qt4 COMPONENTS REQUIRED QtCore QtGui QtOpenGL)
  set(Qt_FOUND TRUE)
  set(Qt_INCLUDE_DIRS "${QT_INCLUDE_DIR};${QT_QTCORE_INCLUDE_DIR};${QT_QTGUI_INCLUDE_DIR};${QT_QTOPENGL_INCLUDE_DIR}")
  set(Qt_LIBRARIES
      Qt4::QtCore
      Qt4::QtGui
      Qt4::QtOpenGL
      )
else()
  find_package(Qt5Core REQUIRED)
  find_package(Qt5Gui REQUIRED)
  find_package(Qt5OpenGL REQUIRED)
  find_package(Qt5Widgets REQUIRED)
  set(Qt_FOUND TRUE)
  set(Qt_INCLUDE_DIRS "${Qt5Core_INCLUDE_DIRS};${Qt5Gui_INCLUDE_DIRS};${Qt5OpenGL_INCLUDE_DIRS};${Qt5Widgets_INCLUDE_DIRS}")
  set(Qt_LIBRARIES
      Qt5::Core
      Qt5::Gui
      Qt5::OpenGL
      Qt5::Widgets
      )
  add_definitions(-DWFlags=WindowFlags)
endif()
set(QT_USE_QTOPENGL TRUE)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} multires_widget
  DEPENDS Qt
  CATKIN_DEPENDS
    mapviz
    pluginlib
    roscpp
    swri_transform_util
    swri_yaml_util
    tf
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
  ${Qt_INCLUDE_DIRS}
)
add_definitions(${QT_DEFINITIONS})

set(QT_USE_QTOPENGL TRUE)

# Build libtile_cache
set(TILE_SRC_FILES 
   src/tile.cpp
   src/tile_cache.cpp
   src/tile_set.cpp
   src/tile_set_layer.cpp)
if("$ENV{ROS_DISTRO}" STRLESS "kinetic")
  QT4_WRAP_CPP(TILE_SRC_FILES include/multires_image/tile_cache.h)
else()
  QT5_WRAP_CPP(TILE_SRC_FILES include/multires_image/tile_cache.h)
endif()

add_library(${PROJECT_NAME} 
  ${TILE_SRC_FILES}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Qt_LIBRARIES}
  ${GLU_LIBRARY}
)

# Build libmultires_widget
set(UI_FILES
  src/QGLMap.ui
  src/multires_config.ui
)
set(WIDGET_SRC_FILES src/tile_view.cpp src/QGLMap.cpp)

if("$ENV{ROS_DISTRO}" STRLESS "kinetic")
  QT4_WRAP_UI(WIDGET_SRC_FILES ${UI_FILES})
  QT4_WRAP_CPP(WIDGET_SRC_FILES include/multires_image/QGLMap.h)
else()
  QT5_WRAP_UI(WIDGET_SRC_FILES ${UI_FILES})
  QT5_WRAP_CPP(WIDGET_SRC_FILES include/multires_image/QGLMap.h)
endif()
add_library(multires_widget ${WIDGET_SRC_FILES})
target_link_libraries(multires_widget ${PROJECT_NAME})

# Build nodes
set(VIEW_NODE_SRC_FILES src/nodes/multires_view_node.cpp)
if("$ENV{ROS_DISTRO}" STRLESS "kinetic")
  QT4_WRAP_CPP(VIEW_NODE_SRC_FILES include/multires_image/multires_view_node.h)
else()
  QT5_WRAP_CPP(VIEW_NODE_SRC_FILES include/multires_image/multires_view_node.h)
endif()
add_executable(multires_view_node ${VIEW_NODE_SRC_FILES})
target_link_libraries(multires_view_node multires_widget)

# Build mapviz plugin
set(MAPVIZ_SRC_FILES 
  src/multires_image_plugin.cpp
  src/multires_view.cpp)
if("$ENV{ROS_DISTRO}" STRLESS "kinetic")
  QT4_WRAP_UI(MAPVIZ_SRC_FILES src/multires_config.ui)
  QT4_WRAP_CPP(MAPVIZ_SRC_FILES include/multires_image/multires_image_plugin.h)
else()
  QT5_WRAP_UI(MAPVIZ_SRC_FILES src/multires_config.ui)
  QT5_WRAP_CPP(MAPVIZ_SRC_FILES include/multires_image/multires_image_plugin.h)
endif()

add_library(${PROJECT_NAME}_plugin ${MAPVIZ_SRC_FILES})
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} multires_view_node multires_widget ${PROJECT_NAME}_plugin
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/mapviz_tile_loader
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES mapviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
