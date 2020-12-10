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

project(tile_map
  LANGUAGES CXX)

# ROS packages
find_package(ament_cmake REQUIRED)
find_package(mapviz REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swri_math_util REQUIRED)
find_package(swri_transform_util REQUIRED)
find_package(tf2 REQUIRED)

# Qt 5
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
# Still useful?
# add_definitions(-DWFlags=WindowFlags)

### OpenGL ###
find_package(OpenGL REQUIRED)

### GLEW ###
find_package(GLEW REQUIRED)

### PkgConfig ###
find_package(PkgConfig REQUIRED)

### yaml-cpp ###
pkg_check_modules(YamlCpp yaml-cpp)

### Use PkgConfig to find jsoncpp ###
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

# Fix conflict between Boost signals used by tf and QT signals used by mapviz
add_definitions(-DQT_NO_KEYWORDS)

# Need to include the current dir to include the results of building Qt UI files
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(TILE_SRC_FILES
  src/image_cache.cpp
  src/texture_cache.cpp
  src/bing_source.cpp
  src/tile_source.cpp
  src/wmts_source.cpp
  src/${PROJECT_NAME}_view.cpp
)
set(QT_HEADERS
  include/${PROJECT_NAME}/image_cache.h
  include/${PROJECT_NAME}/tile_source.h
  include/${PROJECT_NAME}/wmts_source.h
  include/${PROJECT_NAME}/bing_source.h
)
qt5_wrap_cpp(TILE_SRC_FILES ${QT_HEADERS})
add_library(${PROJECT_NAME} ${TILE_SRC_FILES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include)
target_include_directories(${PROJECT_NAME} SYSTEM
  PUBLIC
    GLEW::GLEW
    ${JSONCPP_INCLUDE_DIRS}
    ${YamlCpp_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  ament_cmake
  mapviz
  pluginlib
  rclcpp
  swri_math_util
  swri_transform_util
  tf2
)
target_link_libraries(${PROJECT_NAME}
  ${GLU_LIBRARY}
  ${JSONCPP_LIBRARIES}
  ${YamlCpp_LIBRARIES}
  Qt5::Network
  Qt5::Core
  Qt5::Gui
  Qt5::OpenGL
  Qt5::Widgets
)

set(PLUGIN_SRC_FILES
  src/${PROJECT_NAME}_plugin.cpp)
set(PLUGIN_UI_FILES
  src/${PROJECT_NAME}_config.ui)

qt5_wrap_ui(PLUGIN_SRC_FILES ${PLUGIN_UI_FILES})
qt5_wrap_cpp(PLUGIN_SRC_FILES include/${PROJECT_NAME}/${PROJECT_NAME}_plugin.h)

add_library(${PROJECT_NAME}_plugin SHARED ${PLUGIN_SRC_FILES})
target_compile_definitions(${PROJECT_NAME}_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

pluginlib_export_plugin_description_file(mapviz mapviz_plugins.xml)

ament_export_dependencies(${RUNTIME_DEPS}
  Qt5::Network
  Qt5::Core
  Qt5::Gui
  Qt5::OpenGL
  Qt5::Widgets
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
