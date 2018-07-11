// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-58058A
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Steve Dellenback <sdellenback@swri.org> (210) 522-3914
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <mapviz_plugins/navsat_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QDialog>
#include <QGLWidget>
#include <QPalette>

#include <opencv2/core/core.hpp>

// ROS libraries
#include <ros/master.h>

#include <swri_image_util/geometry_util.h>
#include <swri_transform_util/transform_util.h>

#include <mapviz/select_topic_dialog.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::NavSatPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  NavSatPlugin::NavSatPlugin() : config_widget_(new QWidget())
  {
    ui_.setupUi(config_widget_);

    ui_.color->setColor(Qt::green);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this,
                     SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this,
                     SLOT(TopicEdited()));
    QObject::connect(ui_.positiontolerance, SIGNAL(valueChanged(double)), this,
                     SLOT(PositionToleranceChanged(double)));
    QObject::connect(ui_.buffersize, SIGNAL(valueChanged(int)), this,
                     SLOT(BufferSizeChanged(int)));
    QObject::connect(ui_.drawstyle, SIGNAL(activated(QString)), this,
                     SLOT(SetDrawStyle(QString)));
    QObject::connect(ui_.color, SIGNAL(colorEdited(const QColor&)), this,
                     SLOT(SetColor(const QColor&)));
  }

  NavSatPlugin::~NavSatPlugin()
  {
  }

  void NavSatPlugin::SelectTopic()
  {
    ros::master::TopicInfo topic =
        mapviz::SelectTopicDialog::selectTopic("sensor_msgs/NavSatFix");

    if (!topic.name.empty())
    {
      ui_.topic->setText(QString::fromStdString(topic.name));
      TopicEdited();
    }
  }

  void NavSatPlugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();
    if (topic != topic_)
    {
      initialized_ = false;
      points_.clear();
      has_message_ = false;
      PrintWarning("No messages received.");

      navsat_sub_.shutdown();
      topic_ = topic;
      if (!topic.empty())
      {
        navsat_sub_ = node_.subscribe(topic_, 1, &NavSatPlugin::NavSatFixCallback, this);

        ROS_INFO("Subscribing to %s", topic_.c_str());
      }
    }
  }

  void NavSatPlugin::NavSatFixCallback(
      const sensor_msgs::NavSatFixConstPtr navsat)
  {
    if (!local_xy_util_.Initialized())
    {
      return;
    }
    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }

    StampedPoint stamped_point;
    stamped_point.stamp = navsat->header.stamp;

    double x;
    double y;
    local_xy_util_.ToLocalXy(navsat->latitude, navsat->longitude, x, y);

    stamped_point.point = tf::Point(x, y, navsat->altitude);

    stamped_point.orientation = tf::createQuaternionFromYaw(0.0);

    stamped_point.source_frame = local_xy_util_.Frame();

    if (points_.empty() ||
        stamped_point.point.distance(points_.back().point) >=
            position_tolerance_)
    {
      points_.push_back(stamped_point);
    }

    if (buffer_size_ > 0)
    {
      while (static_cast<int>(points_.size()) > buffer_size_)
      {
        points_.pop_front();
      }
    }

    cur_point_ = stamped_point;
  }

  void NavSatPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void NavSatPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void NavSatPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  QWidget* NavSatPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool NavSatPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;
    SetColor(ui_.color->color());
    return true;
  }

  void NavSatPlugin::Draw(double x, double y, double scale)
  {
    if (DrawPoints(scale))
    {
      PrintInfo("OK");
    }
  }

  void NavSatPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["topic"])
    {
      std::string topic;
      node["topic"] >> topic;
      ui_.topic->setText(topic.c_str());
    }

    if (node["color"])
    {
      std::string color;
      node["color"] >> color;
      SetColor(QColor(color.c_str()));
      ui_.color->setColor(color_);
    }

    if (node["draw_style"])
    {
      std::string draw_style;
      node["draw_style"] >> draw_style;

      if (draw_style == "lines")
      {
        draw_style_ = LINES;
        ui_.drawstyle->setCurrentIndex(0);
      }
      else if (draw_style == "points")
      {
        draw_style_ = POINTS;
        ui_.drawstyle->setCurrentIndex(1);
      }
    }

    if (node["position_tolerance"])
    {
      node["position_tolerance"] >> position_tolerance_;
      ui_.positiontolerance->setValue(position_tolerance_);
    }

    if (node["buffer_size"])
    {
      node["buffer_size"] >> buffer_size_;
      ui_.buffersize->setValue(buffer_size_);
    }

    TopicEdited();
  }

  void NavSatPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "topic" << YAML::Value << topic;

    std::string color = ui_.color->color().name().toStdString();
    emitter << YAML::Key << "color" << YAML::Value << color;

    std::string draw_style = ui_.drawstyle->currentText().toStdString();
    emitter << YAML::Key << "draw_style" << YAML::Value << draw_style;

    emitter << YAML::Key << "position_tolerance" <<
               YAML::Value << position_tolerance_;

    emitter << YAML::Key << "buffer_size" << YAML::Value << buffer_size_;
  }
}
