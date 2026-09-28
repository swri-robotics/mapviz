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

#include <mapviz/display_loader.hpp>

#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

#include <pluginlib/exceptions.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/logging.hpp>

namespace mapviz
{
namespace
{
/// Reads one entry of the "displays" list, filling in defaults for anything
/// missing.  Throws if the entry has no type.
DisplaySpec ReadDisplay(const YAML::Node & entry)
{
  if (!entry.IsMap() || !entry["type"]) {
    throw std::invalid_argument("it has no type");
  }

  DisplaySpec display;
  display.type = entry["type"].as<std::string>();
  display.name = entry["name"] ? entry["name"].as<std::string>() : display.type;
  display.config = entry["config"] ? entry["config"] : YAML::Node(YAML::NodeType::Map);
  if (display.config.IsMap()) {
    if (display.config["visible"]) {
      display.visible = display.config["visible"].as<bool>();
    }
    if (display.config["collapsed"]) {
      display.collapsed = display.config["collapsed"].as<bool>();
    }
  }
  return display;
}
}  // namespace

std::vector<std::string> LoadDisplays(
  const YAML::Node & displays,
  const LoadDisplayFunction & load_display,
  const rclcpp::Logger & logger)
{
  std::vector<std::string> failures;
  if (!displays.IsSequence()) {
    RCLCPP_ERROR(logger, "The config's displays are not a list; no displays were loaded.");
    failures.emplace_back("displays (not a list)");
    return failures;
  }

  for (size_t i = 0; i < displays.size(); i++) {
    // Until the entry has been read, all there is to identify it by is its
    // position in the list.
    std::string label = "display " + std::to_string(i + 1);
    std::string what;
    try {
      const DisplaySpec display = ReadDisplay(displays[i]);
      label = display.type + " (" + display.name + ")";
      load_display(display);
      continue;
    } catch (const pluginlib::PluginlibException & e) {
      what = std::string("could not be created: ") + e.what();
    } catch (const YAML::Exception & e) {
      what = std::string("failed with a YAML error: ") + e.what();
    } catch (const rclcpp::exceptions::RCLError & e) {
      what = std::string("failed with an RCL error: ") + e.what();
    } catch (const std::exception & e) {
      what = std::string("failed: ") + e.what();
    } catch (...) {
      what = "failed with an unknown error";
    }
    RCLCPP_ERROR(logger, "%s %s", label.c_str(), what.c_str());
    failures.push_back(label);
  }
  return failures;
}
}  // namespace mapviz
