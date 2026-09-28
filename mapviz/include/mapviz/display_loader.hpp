// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef MAPVIZ__DISPLAY_LOADER_HPP_
#define MAPVIZ__DISPLAY_LOADER_HPP_

#include <yaml-cpp/yaml.h>

#include <functional>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>

namespace mapviz
{
/// One entry of a config file's "displays" list.
struct DisplaySpec
{
  std::string type;
  std::string name;
  bool visible = true;
  bool collapsed = false;
  YAML::Node config;
};

/// Creates and configures one display.  May throw to report that it failed.
using LoadDisplayFunction = std::function<void (const DisplaySpec & display)>;

/**
 * Loads every entry of a config file's "displays" list with @p load_display.
 *
 * Each display is loaded on its own: whatever one of them throws is logged
 * and recorded, and the rest are still loaded.  Missing settings fall back to
 * what a new display gets (visible, not collapsed, named after its type, and
 * an empty config); only a display with no type fails, since there is nothing
 * to create.
 *
 * @param[in] displays     The "displays" node of a config file.
 * @param[in] load_display Creates and configures one display.
 * @param[in] logger       Where failures are logged, with their details.
 * @return One entry per display that failed, suitable for showing the user.
 */
std::vector<std::string> LoadDisplays(
  const YAML::Node & displays,
  const LoadDisplayFunction & load_display,
  const rclcpp::Logger & logger);
}  // namespace mapviz

#endif  // MAPVIZ__DISPLAY_LOADER_HPP_
