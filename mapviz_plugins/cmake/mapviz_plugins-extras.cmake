include(CMakeFindDependencyMacro)

function(_mapviz_plugins_ensure_interface_alias alias_target dependency_target)
  if(TARGET ${alias_target})
    return()
  endif()

  add_library(${alias_target} INTERFACE IMPORTED)
  set_target_properties(${alias_target} PROPERTIES
    INTERFACE_LINK_LIBRARIES ${dependency_target})
endfunction()

function(_mapviz_plugins_ensure_opencv_component component)
  if(TARGET opencv_${component})
    return()
  endif()

  if(TARGET OpenCV::opencv_${component})
    _mapviz_plugins_ensure_interface_alias(opencv_${component} OpenCV::opencv_${component})
    return()
  endif()

  if(TARGET PkgConfig::OpenCV)
    _mapviz_plugins_ensure_interface_alias(opencv_${component} PkgConfig::OpenCV)
    return()
  endif()

  message(FATAL_ERROR "mapviz_plugins could not resolve OpenCV component target opencv_${component}")
endfunction()

if(NOT TARGET opencv_core)
  find_package(OpenCV QUIET CONFIG COMPONENTS core imgproc highgui)
  if(NOT OpenCV_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(OpenCV REQUIRED IMPORTED_TARGET opencv4)
  endif()
endif()

foreach(component IN ITEMS core imgproc highgui)
  _mapviz_plugins_ensure_opencv_component(${component})
endforeach()