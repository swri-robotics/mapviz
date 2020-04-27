cmake_minimum_required(VERSION 3.10)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

project(mapviz_interfaces)

find_package(ament_cmake REQUIRED)
find_package(marti_common_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/AddMapvizDisplay.srv
  DEPENDENCIES marti_common_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
