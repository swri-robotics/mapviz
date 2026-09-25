# Copyright (c) 2026, Southwest Research Institute® (SwRI®)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

include(CMakeFindDependencyMacro)

function(_tile_map_ensure_interface_alias alias_target dependency_target)
  if(TARGET ${alias_target})
    return()
  endif()

  add_library(${alias_target} INTERFACE IMPORTED)
  set_target_properties(${alias_target} PROPERTIES
    INTERFACE_LINK_LIBRARIES ${dependency_target})
endfunction()

if(NOT TARGET jsoncpp_lib)
  find_package(jsoncpp QUIET CONFIG)
  if(TARGET JsonCpp::JsonCpp)
    _tile_map_ensure_interface_alias(jsoncpp_lib JsonCpp::JsonCpp)
  elseif(NOT TARGET jsoncpp_lib)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)
    _tile_map_ensure_interface_alias(jsoncpp_lib PkgConfig::JSONCPP)
  endif()
endif()

if(NOT TARGET yaml-cpp)
  find_package(yaml-cpp QUIET CONFIG)
  if(NOT TARGET yaml-cpp)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(YamlCpp REQUIRED IMPORTED_TARGET yaml-cpp)
    _tile_map_ensure_interface_alias(yaml-cpp PkgConfig::YamlCpp)
  endif()
endif()
