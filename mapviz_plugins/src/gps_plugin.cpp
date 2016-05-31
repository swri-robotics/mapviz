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

#include <mapviz_plugins/gps_plugin.h>

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
PLUGINLIB_DECLARE_CLASS(mapviz_plugins, gps, mapviz_plugins::GpsPlugin,
                        mapviz::MapvizPlugin);

namespace mapviz_plugins
{
  GpsPlugin::GpsPlugin() : config_widget_(new QWidget())
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
    connect(ui_.color, SIGNAL(colorEdited(const QColor&)), this,
            SLOT(DrawIcon()));
  }

  GpsPlugin::~GpsPlugin()
  {
  }

  void GpsPlugin::SelectTopic()
  {
    ros::master::TopicInfo topic =
        mapviz::SelectTopicDialog::selectTopic("gps_common/GPSFix");

    if (!topic.name.empty())
    {
      ui_.topic->setText(QString::fromStdString(topic.name));
      TopicEdited();
    }
  }

  void GpsPlugin::TopicEdited()
  {
    if (ui_.topic->text().toStdString() != topic_)
    {
      initialized_ = false;
      points_.clear();
      has_message_ = false;
      topic_ = ui_.topic->text().toStdString();
      PrintWarning("No messages received.");

      gps_sub_.shutdown();
      gps_sub_ = node_.subscribe(topic_, 1, &GpsPlugin::GPSFixCallback, this);

      ROS_INFO("Subscribing to %s", topic_.c_str());
    }
  }

  void GpsPlugin::GPSFixCallback(const gps_common::GPSFixConstPtr gps)
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
    stamped_point.stamp = gps->header.stamp;
    stamped_point.source_frame = local_xy_util_.Frame();
    double x;
    double y;
    local_xy_util_.ToLocalXy(gps->latitude, gps->longitude, x, y);

    stamped_point.point = tf::Point(x, y, gps->altitude);
    lap_checked_ = ui_.show_laps->isChecked();
    // The GPS "track" is in degrees, but createQuaternionFromYaw expects
    // radians.
    // Furthermore, the track rotates in the opposite direction and is also
    // offset by 90 degrees, so all of that has to be compensated for.
    stamped_point.orientation =
        tf::createQuaternionFromYaw((-gps->track * (M_PI / 180.0)) + M_PI_2);

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

  void GpsPlugin::PositionToleranceChanged(double value)
  {
    position_tolerance_ = value;
  }

  void GpsPlugin::BufferSizeChanged(int value)
  {
    buffer_size_ = value;

    if (buffer_size_ > 0)
    {
      while (static_cast<int>(points_.size()) > buffer_size_)
      {
        points_.pop_front();
      }
    }
  }

  void GpsPlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_ERROR("Error: %s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void GpsPlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_INFO("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void GpsPlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_WARN("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  QWidget* GpsPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool GpsPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;
    DrawIcon();

    return true;
  }

  void GpsPlugin::Draw(double x, double y, double scale)
  {
    color_ = ui_.color->color();
    if (DrawPoints())
    {
      PrintInfo("OK");
    }
  }

  void GpsPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
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
      ui_.color->setColor(QColor(color.c_str()));
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
      else if (draw_style == "arrows")
      {
        draw_style_ = ARROWS;
        ui_.drawstyle->setCurrentIndex(2);
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

    if (swri_yaml_util::FindValue(node, "show_laps"))
    {
      bool show_laps = false;
      node["show_laps"] >> show_laps;
      ui_.show_laps->setChecked(show_laps);
    }

    TopicEdited();
  }

  void GpsPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "topic" << YAML::Value << topic;

    emitter << YAML::Key << "color" << YAML::Value
            << ui_.color->color().name().toStdString();

    std::string draw_style = ui_.drawstyle->currentText().toStdString();
    emitter << YAML::Key << "draw_style" << YAML::Value << draw_style;

    emitter << YAML::Key << "position_tolerance" << YAML::Value
            << position_tolerance_;

    if (!lap_checked_)
    {
      emitter << YAML::Key << "buffer_size" << YAML::Value << buffer_size_;
    }
    else
    {
      emitter << YAML::Key << "buffer_size" << YAML::Value << buffer_holder_;
    }

    bool show_laps = ui_.show_laps->isChecked();
    emitter << YAML::Key << "show_laps" << YAML::Value << show_laps;
  }
}
