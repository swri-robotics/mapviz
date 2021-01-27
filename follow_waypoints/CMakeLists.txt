cmake_minimum_required(VERSION 2.8.3)
project(follow_waypoints_mapviz_plugin)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  mapviz
  move_base_msgs
  pluginlib
  roscpp
  std_msgs
)

# Qt Libaries (for mapviz plugin)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
# Setting these variable so catkin_package can export Qt as a dependency
set(Qt_FOUND TRUE)
set(Qt_INCLUDE_DIRS "${Qt5Core_INCLUDE_DIRS};${Qt5Gui_INCLUDE_DIRS};${Qt5OpenGL_INCLUDE_DIRS};${Qt5Widgets_INCLUDE_DIRS}")
set(Qt_LIBRARIES
    Qt5::Core
    Qt5::Gui
    Qt5::OpenGL
    Qt5::Widgets
    )
add_definitions(-DWFlags=WindowFlags)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_plugin ${PROJECT_NAME}_widget
  DEPENDS Qt
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    mapviz
    move_base_msgs
    pluginlib
    roscpp
    std_msgs
)


# Need to include the current dir to include the results of building Qt UI files
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Build libfollow_waypoints_widget
set(UI_FILES
  src/follow_waypoints_config.ui
)
set(SRC_FILES
  src/follow_waypoints_mapviz_plugin.cpp
)
set(HEADER_FILES
    include/follow_waypoints_mapviz_plugin.h
)

qt5_wrap_ui(UI_SRC_FILES ${UI_FILES})
qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})

add_library(${PROJECT_NAME}
    ${MOC_FILES}
    ${SRC_FILES}
    ${HEADER_FILES}
    ${UI_SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Qt_LIBRARIES}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS "-std=c++11 -D__STDC_FORMAT_MACROS"
)

### Install the plugins ###
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES mapviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
