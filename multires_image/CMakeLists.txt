### SET CMAKE POLICIES ###
cmake_minimum_required(VERSION 3.10...3.17)

if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(${CMAKE_VERSION} VERSION_EQUAL "3.11.0" OR ${CMAKE_VERSION} VERSION_GREATER "3.11.0")
  cmake_policy(SET CMP0072 NEW)
endif()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
### END CMAKE POLICIES ###

project(multires_image
  LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(mapviz REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp  REQUIRED)
find_package(tf2 REQUIRED)
find_package(swri_transform_util  REQUIRED)
find_package(swri_math_util REQUIRED)

### Boost ###
find_package(Boost REQUIRED filesystem thread)

### OpenGL ###
find_package(OpenGL REQUIRED)

### OpenCV ###
# multires_image doesn't directly use OpenCV, but it uses swri_transform_util, and
# we need to include OpenCV to make one of its headers compile.
find_package(OpenCV COMPONENTS core imgproc REQUIRED)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
add_definitions(-DWFlags=WindowFlags)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Build libtile_cache
set(TILE_SRC_FILES 
   src/tile.cpp
   src/tile_cache.cpp
   src/tile_set.cpp
   src/tile_set_layer.cpp)

qt5_wrap_cpp(TILE_SRC_FILES include/${PROJECT_NAME}/tile_cache.h)

add_library(${PROJECT_NAME} 
  ${TILE_SRC_FILES}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  OpenGL::GL
  Qt5::Core
  Qt5::Gui
  Qt5::OpenGL
  Qt5::Widgets
)
ament_target_dependencies(${PROJECT_NAME}
  ament_cmake
  cv_bridge
  gps_msgs
  mapviz
  pluginlib
  rclcpp 
  tf2
  swri_transform_util 
  swri_math_util
)

# Build libmultires_widget
set(UI_FILES
  src/QGLMap.ui
  src/multires_config.ui
)
set(WIDGET_SRC_FILES src/tile_view.cpp src/QGLMap.cpp)

qt5_wrap_ui(WIDGET_SRC_FILES ${UI_FILES})
qt5_wrap_cpp(WIDGET_SRC_FILES include/${PROJECT_NAME}/QGLMap.h)

add_library(multires_widget
  ${WIDGET_SRC_FILES}
)
target_link_libraries(multires_widget
  ${PROJECT_NAME}
  OpenGL::GL
  Qt5::Core
  Qt5::Gui
  Qt5::OpenGL
  Qt5::Widgets
)

# Build nodes
set(VIEW_NODE_SRC_FILES src/nodes/multires_view_node.cpp)
qt5_wrap_cpp(VIEW_NODE_SRC_FILES include/${PROJECT_NAME}/multires_view_node.h)
add_executable(multires_view_node ${VIEW_NODE_SRC_FILES})
target_link_libraries(multires_view_node
  multires_widget
  Boost::filesystem
  Boost::thread
)
ament_target_dependencies(multires_view_node
  ament_cmake
  rclcpp
  swri_transform_util
)

# Build mapviz plugin
set(MAPVIZ_SRC_FILES 
  src/${PROJECT_NAME}_plugin.cpp
  src/multires_view.cpp)
qt5_wrap_ui(MAPVIZ_SRC_FILES src/multires_config.ui)
qt5_wrap_cpp(MAPVIZ_SRC_FILES include/${PROJECT_NAME}/${PROJECT_NAME}_plugin.h)

add_library(${PROJECT_NAME}_plugin SHARED
  ${MAPVIZ_SRC_FILES}
)
target_compile_definitions(${PROJECT_NAME}_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(${PROJECT_NAME}_plugin
  ${PROJECT_NAME}
)

### Install ${PROJECT_NAME} plugin ###
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  multires_view_node
  multires_widget
  ${PROJECT_NAME}_plugin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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

ament_export_dependencies(${RUNTIME_DEPS} Qt)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
