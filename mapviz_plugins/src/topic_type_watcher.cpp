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

#include <mapviz_plugins/topic_type_watcher.hpp>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/logging.hpp>

namespace mapviz_plugins
{
TopicTypeWatcher::TopicTypeWatcher(std::vector<std::string> supported_types)
: supported_types_(std::move(supported_types)),
  source_{nullptr, nullptr, rclcpp::get_logger("mapviz")},
  reported_unsupported_(false)
{
  timer_.setInterval(1000);
  QObject::connect(&timer_, &QTimer::timeout, [this]() {Check();});
}

bool TopicTypeWatcher::Watch(
  const std::string & topic,
  mapviz::TopicSource source,
  TypeCallback on_type)
{
  Stop();
  if (topic.empty()) {
    return false;
  }

  topic_ = topic;
  source_ = std::move(source);
  on_type_ = std::move(on_type);
  reported_unsupported_ = false;

  if (Check()) {
    return true;
  }
  timer_.start();
  return false;
}

void TopicTypeWatcher::Stop()
{
  timer_.stop();
  topic_.clear();
  on_type_ = nullptr;
}

bool TopicTypeWatcher::Check()
{
  if (topic_.empty() || !on_type_) {
    return true;
  }

  const mapviz::TopicSource::NamesAndTypes topics = source_.topics();
  const auto found = topics.find(topic_);
  if (found == topics.end()) {
    return false;
  }

  for (const std::string & supported : supported_types_) {
    const std::vector<std::string> & advertised = found->second;
    if (std::find(advertised.begin(), advertised.end(), supported) != advertised.end()) {
      // Stop before calling out, since on_type_ may start a new watch.
      timer_.stop();
      TypeCallback on_type = on_type_;
      on_type(supported);
      return true;
    }
  }

  // Keep watching: a publisher with a supported type may still appear.
  if (!reported_unsupported_) {
    reported_unsupported_ = true;
    RCLCPP_ERROR(
      source_.logger,
      "Topic %s has type %s, which this display does not support.",
      topic_.c_str(), found->second.empty() ? "" : found->second.front().c_str());
  }
  return false;
}
}  // namespace mapviz_plugins
