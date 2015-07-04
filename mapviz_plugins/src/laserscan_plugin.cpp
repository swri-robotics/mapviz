// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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

#include <mapviz_plugins/laserscan_plugin.h>

// C++ standard libraries
#include <cmath>
#include <cstdio>
#include <vector>

// Boost libraries
#include <boost/algorithm/string.hpp>

// QT libraries
#include <QDialog>
#include <QGLWidget>

// ROS libraries
#include <ros/master.h>

#include <math_util/constants.h>
#include <yaml_util/yaml_util.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
    mapviz_plugins,
    laserscan,
    mapviz_plugins::LaserScanPlugin,
    mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  LaserScanPlugin::LaserScanPlugin() :
    config_widget_(new QWidget())
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

  LaserScanPlugin::~LaserScanPlugin()
  {
  }

  void LaserScanPlugin::SelectTopic()
  {
    QDialog dialog;
    Ui::topicselect ui;
    ui.setupUi(&dialog);

    std::vector<ros::master::TopicInfo> topics;
    ros::master::getTopics(topics);

    for (unsigned int i = 0; i < topics.size(); i++)
    {
      if (topics[i].datatype == "sensor_msgs/LaserScan")
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

  void LaserScanPlugin::TopicEdited()
  {
    if (ui_.topic->text().toStdString() != topic_)
    {
      initialized_ = false;
      scan_.points.clear();
      has_message_ = false;
      topic_ = boost::trim_copy(ui_.topic->text().toStdString());
      PrintWarning("No messages received.");

      laserscan_sub_.shutdown();
      laserscan_sub_ = node_.subscribe(topic_, 100, &LaserScanPlugin::laserScanCallback, this);

      ROS_INFO("Subscribing to %s", topic_.c_str());
    }
  }

  void LaserScanPlugin::laserScanCallback(const sensor_msgs::LaserScanConstPtr msg)
  {
    if (!has_message_)
    {
      source_frame_ = msg->header.frame_id;
      initialized_ = true;
      has_message_ = true;
    }

    Scan& scan = scan_;
    scan.stamp = msg->header.stamp;
    scan.color = QColor::fromRgbF(1.0f, 0.0f, 0.0f, 1.0f);
    scan.transformed = true;

    scan.points.clear();

    transform_util::Transform transform;
    if (!GetTransform(msg->header.stamp, transform))
    {
      scan.transformed = false;
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }
    double angle, x, y;
    for (unsigned int i = 0; i < msg->ranges.size(); i++)
    {
      // Discard the point if it's out of range
      if( msg->ranges[i] > msg->range_max || msg->ranges[i] < msg->range_min)
      {
        continue;
      }
      StampedPoint point;
      angle  = msg->angle_min + msg->angle_increment * i;
      x = cos(angle) * msg->ranges[i];
      y = sin(angle) * msg->ranges[i];
      point.point = tf::Point(x, y, 0.0f);
      point.transformed_point = transform * point.point;
      //TODO(evenator): Color options (or use intensity?)
      point.color = scan.color;
      scan.points.push_back(point);
    }

    canvas_->update();
  }

  void LaserScanPlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_ERROR("Error: %s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void LaserScanPlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_INFO("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void LaserScanPlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_WARN("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  QWidget* LaserScanPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool LaserScanPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;

    return true;
  }

  void LaserScanPlugin::Draw(double x, double y, double scale)
  {
    ros::Time now = ros::Time::now();

    std::map<std::string, std::map<int, Scan> >::iterator nsIter;
    Scan& scan = scan_;

    if (scan.transformed)
    {
      glPointSize(3);
      glBegin(GL_POINTS);

      std::list<StampedPoint>::iterator point_it = scan.points.begin();
      for (; point_it != scan.points.end(); ++point_it)
      {
        glColor4f(
            scan.color.redF(),
            scan.color.greenF(),
            scan.color.blueF(),
            scan.color.alphaF());
        glVertex2f(
            point_it->transformed_point.getX(),
            point_it->transformed_point.getY());
      }

      glEnd();

      PrintInfo("OK");
    }
  }

  void LaserScanPlugin::Transform()
  {
    std::map<std::string, std::map<int, Scan> >::iterator nsIter;
    Scan& scan = scan_;

    transform_util::Transform transform;
    if (GetTransform(scan.stamp, transform))
    {
      scan.transformed = true;

      std::list<StampedPoint>::iterator point_it = scan.points.begin();
      for (; point_it != scan.points.end(); ++point_it)
      {
        point_it->transformed_point = transform * point_it->point;
      }
    }
    else
    {
      scan.transformed = false;
    }
  }

  void LaserScanPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    std::string topic;
    node["topic"] >> topic;
    ui_.topic->setText(boost::trim_copy(topic).c_str());

    TopicEdited();
  }

  void LaserScanPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "topic" << YAML::Value << boost::trim_copy(ui_.topic->text().toStdString());
  }
}

