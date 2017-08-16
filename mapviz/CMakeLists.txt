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
find_package(OpenCV 2 REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
set(QT_USE_QTOPENGL TRUE)

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
  DEPENDS libqt4
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

include(${QT_USE_FILE})
include_directories(include ${catkin_INCLUDE_DIRS} ${QT_INCLUDE_DIR} ${QT_QTOPENGL_INCLUDE_DIR} ${OpenGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR})
add_definitions(${QT_DEFINITIONS})

set(QT_USE_QTOPENGL TRUE)
set(QT_USE_QTCORE TRUE)
set(QT_USE_QTGUI TRUE)

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
QT4_ADD_RESOURCES(RCC_SRCS src/resources/icons.qrc)
QT4_WRAP_UI(SRC_FILES ${UI_FILES})
QT4_WRAP_CPP(SRC_FILES ${HEADER_FILES})

add_library(rqt_${PROJECT_NAME} ${SRC_FILES} ${RCC_SRCS})
target_link_libraries(rqt_${PROJECT_NAME} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARIES} ${OpenGL_LIBRARY} ${GLUT_LIBRARY} ${GLEW_LIBRARIES} ${GLU_LIBRARY} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(rqt_${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
set_target_properties(rqt_${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11 -D__STDC_FORMAT_MACROS")
add_dependencies(rqt_${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

add_executable(${PROJECT_NAME} ${SRC_FILES} ${RCC_SRCS} src/mapviz_main.cpp)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${QT_QTOPENGL_LIBRARIES} ${OpenGL_LIBRARY} ${GLUT_LIBRARY} ${GLEW_LIBRARIES} ${GLU_LIBRARY} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
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
