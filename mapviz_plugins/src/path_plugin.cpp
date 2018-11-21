// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
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

#include <mapviz_plugins/path_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QDialog>
#include <QGLWidget>

// ROS libraries
#include <ros/master.h>

#include <mapviz/select_topic_dialog.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::PathPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  PathPlugin::PathPlugin() : config_widget_(new QWidget())
  {
    ui_.setupUi(config_widget_);
    ui_.path_color->setColor(Qt::green);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
    connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
    connect(ui_.path_color, SIGNAL(colorEdited(const QColor&)), this,
            SLOT(SetColor(const QColor&)));

  }

  PathPlugin::~PathPlugin()
  {
  }

  void PathPlugin::SelectTopic()
  {
    ros::master::TopicInfo topic =
        mapviz::SelectTopicDialog::selectTopic("nav_msgs/Path");

    if (!topic.name.empty())
    {
      ui_.topic->setText(QString::fromStdString(topic.name));
      TopicEdited();
    }
  }

  void PathPlugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();
    if (topic != topic_)
    {
      initialized_ = false;
      ClearPoints();
      has_message_ = false;
      PrintWarning("No messages received.");

      path_sub_.shutdown();

      topic_ = topic;
      if (!topic.empty())
      {
        path_sub_ = node_.subscribe(topic_, 1, &PathPlugin::pathCallback, this);

        ROS_INFO("Subscribing to %s", topic_.c_str());
      }
    }
  }

  void PathPlugin::pathCallback(const nav_msgs::PathConstPtr& path)
  {
    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }

    ClearPoints();

    for (unsigned int i = 0; i < path->poses.size(); i++)
    {
      StampedPoint stamped_point;
      stamped_point.stamp = path->header.stamp;
      stamped_point.source_frame = path->header.frame_id;
      stamped_point.point = tf::Point(path->poses[i].pose.position.x,
                                      path->poses[i].pose.position.y, 0);

      pushPoint( std::move(stamped_point) );
    }
  }

  void PathPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void PathPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void PathPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  QWidget* PathPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool PathPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;
    DrawIcon();
    return true;
  }

  void PathPlugin::Draw(double x, double y, double scale)
  {
    bool lines;
    bool points;
    QColor old_color = ui_.path_color->color();
    QColor color = old_color.dark(200);
    SetDrawStyle( LINES );
    lines = DrawPoints(scale);
    SetColor(color);
    SetDrawStyle( POINTS );
    points = DrawPoints(scale);
    SetColor(old_color);
    if (lines && points)
    {
      PrintInfo("OK");
    }
  }

  void PathPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (swri_yaml_util::FindValue(node, "topic"))
    {
      std::string topic;
      node["topic"] >> topic;
      ui_.topic->setText(topic.c_str());
      TopicEdited();
    }

    if (swri_yaml_util::FindValue(node, "color"))
    {
      std::string color;
      node["color"] >> color;
      QColor qcolor(color.c_str());
      SetColor(qcolor);
      ui_.path_color->setColor(qcolor);
    }
  }

  void PathPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "topic" << YAML::Value << topic;

    std::string color = ui_.path_color->color().name().toStdString();
    emitter << YAML::Key << "color" << YAML::Value << color;
  }
}
