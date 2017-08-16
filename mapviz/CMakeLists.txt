cmake_minimum_required(VERSION 2.8.3)
project(mapviz)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  marti_common_msgs
  message_generation
  rosapi
  roscpp
  #rqt_gui
  rqt_gui_cpp
  pluginlib
  swri_transform_util
  swri_yaml_util
  tf
)

find_package(Boost REQUIRED system filesystem)
find_package(OpenCV 3.1 COMPONENTS core imgcodecs imgproc videoio REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
set(QT_LIBRARIES
  ${Qt5Core_LIBRARIES}
  ${Qt5Gui_LIBRARIES}
  ${Qt5OpenGL_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)
include_directories(
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5OpenGL_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)
add_definitions(
  ${Qt5Core_DEFINITIONS}
  ${Qt5Gui_DEFINITIONS}
  ${Qt5OpenGL_DEFINITIONS}
  ${Qt5Widgets_DEFINITIONS}
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

# Require OpenGL
find_package(OpenGL REQUIRED)
find_package(GLU REQUIRED)
find_package(GLUT REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GLEW glew)

add_service_files(FILES
  AddMapvizDisplay.srv
)

generate_messages(DEPENDENCIES
  marti_common_msgs)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Qt5Core Qt5Gui Qt5OpenGL Qt5Widgets
  CATKIN_DEPENDS
    marti_common_msgs
    message_runtime
    roscpp
    #rqt_gui
    rqt_gui_cpp
    pluginlib
    swri_transform_util
    swri_yaml_util
    tf
)

# Fix conflict between Boost signals used by tf and QT signals used by mapviz
add_definitions(-DQT_NO_KEYWORDS)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR})

# Build mapviz node
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(src)
file (GLOB UI_FILES
  src/configitem.ui
  src/mapviz.ui
  src/pluginselect.ui
  src/rqt_mapviz.ui
)
file (GLOB HEADER_FILES
  include/mapviz/color_button.h
  include/mapviz/config_item.h
  include/mapviz/map_canvas.h
  include/mapviz/mapviz.h
  include/mapviz/mapviz_application.h
  include/mapviz/mapviz_plugin.h
  include/mapviz/rqt_mapviz.h
  include/mapviz/select_frame_dialog.h
  include/mapviz/select_service_dialog.h
  include/mapviz/select_topic_dialog.h
  include/mapviz/video_writer.h
  include/mapviz/widgets.h
)
file (GLOB SRC_FILES
  src/mapviz.cpp
  src/color_button.cpp
  src/config_item.cpp
  src/mapviz_application.cpp
  src/map_canvas.cpp
  src/rqt_mapviz.cpp
  src/select_frame_dialog.cpp
  src/select_service_dialog.cpp
  src/select_topic_dialog.cpp
  src/video_writer.cpp
)
qt5_add_resources(RCC_SRCS src/resources/icons.qrc)
qt5_wrap_ui(SRC_FILES ${UI_FILES})
qt5_wrap_cpp(SRC_FILES ${HEADER_FILES})

add_library(rqt_${PROJECT_NAME} ${SRC_FILES} ${RCC_SRCS})
link_directories(${OpenCV_LIBRARY_DIRS})
target_link_libraries(rqt_${PROJECT_NAME} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARIES} ${OpenGL_LIBRARY} ${GLUT_LIBRARY} ${GLEW_LIBRARIES} ${GLU_LIBRARY} ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(rqt_${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
set_target_properties(rqt_${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11 -D__STDC_FORMAT_MACROS")
add_dependencies(rqt_${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

add_executable(${PROJECT_NAME} ${SRC_FILES} ${RCC_SRCS} src/mapviz_main.cpp)
link_directories(${OpenCV_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${QT_QTOPENGL_LIBRARIES} ${OpenGL_LIBRARY} ${GLUT_LIBRARY} ${GLEW_LIBRARIES} ${GLU_LIBRARY} ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11 -D__STDC_FORMAT_MACROS")
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} rqt_${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
