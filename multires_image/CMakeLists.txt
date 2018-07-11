cmake_minimum_required(VERSION 2.8.3)
project(multires_image)

find_package(catkin REQUIRED COMPONENTS
  mapviz
  roscpp 
  tf 
  swri_transform_util 
  swri_math_util
  swri_yaml_util)

find_package(Boost REQUIRED thread)
find_package(GLU REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
set(QT_USE_QTOPENGL TRUE)
find_package(OpenCV 2 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} multires_widget
  DEPENDS Qt4
  CATKIN_DEPENDS
    mapviz
    pluginlib
    roscpp
    swri_transform_util
    swri_yaml_util
    tf
)

include(${QT_USE_FILE})
include_directories(include ${catkin_INCLUDE_DIRS} ${QT_INCLUDE_DIR} ${QT_QTOPENGL_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR})
add_definitions(${QT_DEFINITIONS})

set(QT_USE_QTOPENGL TRUE)

# Build libtile_cache
file (GLOB TILE_SRC_FILES 
   src/string_util.cpp
   src/tile.cpp
   src/tile_cache.cpp
   src/tile_set.cpp
   src/tile_set_layer.cpp)
QT4_WRAP_CPP(TILE_SRC_FILES include/multires_image/tile_cache.h)
add_library(${PROJECT_NAME} ${TILE_SRC_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARIES} ${GLU_LIBRARY} ${OpenCV_LIBRARIES})

# Build libmultires_widget
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(src)
file (GLOB UI_FILES src/*.ui)
file (GLOB WIDGET_SRC_FILES src/tile_view.cpp src/QGLMap.cpp)
QT4_WRAP_UI(WIDGET_SRC_FILES ${UI_FILES})
QT4_WRAP_CPP(WIDGET_SRC_FILES include/multires_image/QGLMap.h)
add_library(multires_widget ${WIDGET_SRC_FILES})
target_link_libraries(multires_widget ${PROJECT_NAME})

# Build nodes
include_directories(src)
file (GLOB VIEW_NODE_SRC_FILES src/nodes/multires_view_node.cpp)
QT4_WRAP_CPP(VIEW_NODE_SRC_FILES include/multires_image/multires_view_node.h)
add_executable(multires_view_node ${VIEW_NODE_SRC_FILES})
target_link_libraries(multires_view_node multires_widget)

# Build mapviz plugin
file (GLOB MAPVIZ_SRC_FILES 
  src/multires_image_plugin.cpp
  src/multires_view.cpp)
QT4_WRAP_UI(MAPVIZ_SRC_FILES src/multires_config.ui)
QT4_WRAP_CPP(MAPVIZ_SRC_FILES include/multires_image/multires_image_plugin.h)
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
