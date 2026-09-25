cmake_minimum_required(VERSION 3.14.4)

project(mapviz_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(marti_common_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/AddMapvizDisplay.srv
  DEPENDENCIES marti_common_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # TODO(#938): Enable the remaining linters once the code complies with them.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
