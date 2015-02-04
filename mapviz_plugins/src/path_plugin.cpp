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

#include <yaml_util/yaml_util.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(mapviz_plugins, path, mapviz_plugins::PathPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  PathPlugin::PathPlugin() :
    config_widget_(new QWidget()),
    transformed_(false),
    color_(Qt::green),
    line_width_(2)
  {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
  }

  PathPlugin::~PathPlugin()
  {
  }

  void PathPlugin::SelectTopic()
  {
    QDialog dialog;
    Ui::topicselect ui;
    ui.setupUi(&dialog);

    std::vector<ros::master::TopicInfo> topics;
    ros::master::getTopics(topics);

    for (unsigned int i = 0; i < topics.size(); i++)
    {
      if (topics[i].datatype == "nav_msgs/Path")
      {
        ui.displaylist->addItem(topics[i].name.c_str());
      }
    }
    ui.displaylist->setCurrentRow(0);

    dialog.exec();

    if (dialog.result() == QDialog::Accepted && ui.displaylist->selectedItems().count() == 1)
    {
      ui_.topic->setText(ui.displaylist->selectedItems().first()->text());
      TopicEdited();
    }
  }

  void PathPlugin::TopicEdited()
  {
    if (ui_.topic->text().toStdString() != topic_)
    {
      initialized_ = false;
      points_.clear();
      transformed_points_.clear();
      has_message_ = false;
      topic_ = ui_.topic->text().toStdString();
      PrintWarning("No messages received.");

      path_sub_.shutdown();
      path_sub_ = node_.subscribe(topic_, 1, &PathPlugin::pathCallback, this);

      ROS_INFO("Subscribing to %s", topic_.c_str());
    }
  }

  void PathPlugin::pathCallback(const nav_msgs::PathConstPtr path)
  {
    ROS_INFO("Got path message");
    if (!has_message_)
    {
      source_frame_ = path->header.frame_id;
      initialized_ = true;
      has_message_ = true;
    }

    transformed_ = false;

    stamp_ = path->header.stamp;
    points_.clear();
    transformed_points_.clear();

    for (unsigned int i = 0; i < path->poses.size(); i++)
    {
      tf::Point point(
        path->poses[i].pose.position.x,
        path->poses[i].pose.position.y,
        0);

      points_.push_back(point);
      transformed_points_.push_back(point);
    }

    canvas_->update();
  }

  void PathPlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_ERROR("Error: %s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void PathPlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_INFO("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void PathPlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_WARN("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  QWidget* PathPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool PathPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;

    return true;
  }

  void PathPlugin::Draw(double x, double y, double scale)
  {
    if (transformed_ && has_message_)
    {
      glLineWidth(line_width_);
      glColor4f(color_.redF(), color_.greenF(), color_.blueF(), color_.alphaF());
      glBegin(GL_LINE_STRIP);

        std::list<tf::Point>::iterator transformed_it = transformed_points_.begin();
        for (; transformed_it != transformed_points_.end(); ++transformed_it)
        {
          glVertex2f(transformed_it->getX(), transformed_it->getY());
        }

      glEnd();

      glPointSize(line_width_*4);

      QColor dark = color_.darker(200);

      glColor4f(dark.redF(), dark.greenF(), dark.blueF(), dark.alphaF());
      glBegin(GL_POINTS);

        transformed_it = transformed_points_.begin();
        for (; transformed_it != transformed_points_.end(); ++transformed_it)
        {
          glVertex2f(transformed_it->getX(), transformed_it->getY());
        }

      glEnd();

      PrintInfo("OK");
    }
  }

  void PathPlugin::Transform()
  {
    transformed_ = false;

    transform_util::Transform transform;
    if (GetTransform(stamp_, transform))
    {
      std::list<tf::Point>::iterator points_it = points_.begin();
      std::list<tf::Point>::iterator transformed_it = transformed_points_.begin();
      for (; points_it != points_.end() && transformed_it != transformed_points_.end(); ++points_it)
      {
        (*transformed_it) = transform * (*points_it);

        ++transformed_it;
      }

      transformed_ = true;
    }
    else
    {
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }
  }

  void PathPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    std::string topic;
    node["topic"] >> topic;
    ui_.topic->setText(topic.c_str());

    TopicEdited();
  }

  void PathPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "topic" << YAML::Value << topic;
  }
}

