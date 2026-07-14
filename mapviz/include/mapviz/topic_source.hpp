// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef MAPVIZ__TOPIC_SOURCE_HPP_
#define MAPVIZ__TOPIC_SOURCE_HPP_

#include <functional>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>

namespace mapviz
{
/**
 * A narrow, read-only view of the ROS graph for the topic/service selection
 * dialogs.  ROS graph queries are thread-safe, so unlike a raw node handle
 * this grants nothing that can register callbacks, create entities, or
 * otherwise require NodeUnsafe()'s caveats.  Obtain one from
 * MapvizPlugin::TopicSource().
 */
struct TopicSource
{
  /// Name -> datatypes, as returned by get_*_names_and_types().
  using NamesAndTypes = std::map<std::string, std::vector<std::string>>;

  /// Returns every known topic and its datatypes.
  std::function<NamesAndTypes()> topics;
  /// Returns every known service and its datatypes.
  std::function<NamesAndTypes()> services;
  /// Logger for the dialog's diagnostics.
  rclcpp::Logger logger;
};
}  // namespace mapviz

#endif  // MAPVIZ__TOPIC_SOURCE_HPP_
