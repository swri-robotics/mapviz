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

#ifndef MAPVIZ_PLUGINS__TOPIC_TYPE_WATCHER_HPP_
#define MAPVIZ_PLUGINS__TOPIC_TYPE_WATCHER_HPP_

#include <QTimer>

#include <functional>
#include <string>
#include <vector>

#include <mapviz/topic_source.hpp>

namespace mapviz_plugins
{
/**
 * Finds out which message type a topic carries, for displays that accept
 * several types on one topic.
 *
 * ROS 2 cannot subscribe to "whatever type the topic has", and some middleware
 * (Fast DDS) refuses one node subscribing to the same topic under several
 * types, so a display has to subscribe with the one type the topic is
 * advertised with.  If nothing advertises the topic yet, the watcher checks
 * again every second until something does.
 *
 * Must be used from the GUI thread.
 */
class TopicTypeWatcher
{
public:
  using TypeCallback = std::function<void (const std::string & type)>;

  /// @param supported_types Types the display can show, in order of
  ///        preference, e.g. "std_msgs/msg/Float64".
  explicit TopicTypeWatcher(std::vector<std::string> supported_types);

  /**
   * Calls @p on_type with the first supported type that @p topic is
   * advertised with: straight away if the topic is already known, otherwise
   * once it appears.  Replaces any earlier watch.
   *
   * @return True if @p on_type was called before returning.
   */
  bool Watch(const std::string & topic, mapviz::TopicSource source, TypeCallback on_type);

  /// Cancels the current watch, if any.
  void Stop();

private:
  /// Returns true once the topic's type is known and @p on_type_ was called.
  bool Check();

  std::vector<std::string> supported_types_;
  std::string topic_;
  mapviz::TopicSource source_;
  TypeCallback on_type_;
  QTimer timer_;
  bool reported_unsupported_;
};
}  // namespace mapviz_plugins

#endif  // MAPVIZ_PLUGINS__TOPIC_TYPE_WATCHER_HPP_
