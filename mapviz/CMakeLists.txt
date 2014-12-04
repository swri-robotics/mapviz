cmake_minimum_required(VERSION 2.4.6)
project(mapviz)

find_package(catkin REQUIRED COMPONENTS roscpp pluginlib tf transform_util yaml_util build_tools)

find_package(Boost REQUIRED system filesystem)
find_package(OpenCV REQUIRED)
find_package(Qt4 REQUIRED)

# Require OpenGL
find_package(OpenGL REQUIRED)
find_package(GLU REQUIRED)
find_package(GLUT REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GLEW glew)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS libqt4
  CATKIN_DEPENDS roscpp pluginlib tf transform_util yaml_util
)

execute_process(
  COMMAND rospack find build_tools
  ERROR_VARIABLE __null
  OUTPUT_VARIABLE build_tools_PACKAGE_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE)
include(${build_tools_PACKAGE_PATH}/scripts/build_common.cmake)

# Fix conflict between Boost signals used by tf and QT signals used by mapviz
add_definitions(-DQT_NO_KEYWORDS)

include(${QT_USE_FILE})
include_directories(${catkin_INCLUDE_DIRS} ${QT_INCLUDE_DIR} ${QT_QTOPENGL_INCLUDE_DIR} ${OpenGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR})
add_definitions(${QT_DEFINITIONS})

set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTCORE TRUE)
set(QT_USE_QTGUI TRUE)

# Build mapviz node
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(src)
file (GLOB UI_FILES src/*.ui)
file (GLOB SRC_FILES src/mapviz.cpp src/config_item.cpp src/map_canvas.cpp)
QT4_ADD_RESOURCES(RCC_SRCS src/resources/icons.qrc)
QT4_WRAP_UI(SRC_FILES ${UI_FILES})
QT4_WRAP_CPP(SRC_FILES include/mapviz/mapviz.h include/mapviz/config_item.h include/mapviz/widgets.h include/mapviz/map_canvas.h)

add_executable(${PROJECT_NAME} ${SRC_FILES} ${RCC_SRCS})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${QT_QTOPENGL_LIBRARIES} ${OpenGL_LIBRARY} ${GLUT_LIBRARY} ${GLEW_LIBRARIES} ${GLU_LIBRARY} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
