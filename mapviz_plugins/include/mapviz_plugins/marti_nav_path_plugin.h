// *****************************************************************************
//
// Copyright (c) 2021, Southwest Research Institute® (SwRI®)
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

#ifndef MAPVIZ_PLUGINS_MARTI_NAV_PATH_PLUGIN_H_
#define MAPVIZ_PLUGINS_MARTI_NAV_PATH_PLUGIN_H_

#include <string>

#include <boost/circular_buffer.hpp>

#include <mapviz/mapviz_plugin.h>

#include <QObject>
#include <QGLWidget>
#include <QColor>

#include <ros/subscriber.h>

#include <mapviz/mapviz_plugin.h>

#include <marti_nav_msgs/Path.h>

#include <topic_tools/shape_shifter.h>

#include "ui_marti_nav_path_config.h"

namespace mapviz_plugins
{
class MartiNavPathPlugin : public mapviz::MapvizPlugin
{
  Q_OBJECT

  Ui::marti_nav_path_config ui_;
  QWidget* config_widget_;
  std::string topic_;

  ros::Subscriber subscriber_;

  boost::circular_buffer<marti_nav_msgs::Path> items_;

public:

  MartiNavPathPlugin();

  bool Initialize(QGLWidget* canvas);
  void Shutdown();
  void Draw(double x, double y, double scale);
  void Transform();

  void LoadConfig(const YAML::Node& node, const std::string& path);
  void SaveConfig(YAML::Emitter& emitter, const std::string& path);

  QWidget* GetConfigWidget(QWidget* parent);

protected:

  void PrintInfo(const std::string& message);
  void PrintWarning(const std::string& message);
  void PrintError(const std::string& message);

  void setColorForDirection(const bool in_reverse);

  // Callbacks and helpers for them
  void messageCallback(const topic_tools::ShapeShifter::ConstPtr& msg);
  void handlePath(const marti_nav_msgs::Path& path);
  void handlePathPoint(const marti_nav_msgs::PathPoint& pt);

protected Q_SLOTS:  // NOLINT - Qt SLOTS

  void selectTopic();
  void topicEdited();
  void historyChanged();
};  // class MartiNavPathPlugin
}  // namespace mapviz_plugins
#endif  // MAPVIZ_PLUGINS_MARTI_NAV_PATH_PLUGIN_H_
