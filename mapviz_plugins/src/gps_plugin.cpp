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
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::GpsPlugin, mapviz::MapvizPlugin)

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
    QObject::connect(ui_.static_arrow_sizes, SIGNAL(clicked(bool)),
                     this, SLOT(SetStaticArrowSizes(bool)));
    QObject::connect(ui_.arrow_size, SIGNAL(valueChanged(int)),
                     this, SLOT(SetArrowSize(int)));
    QObject::connect(ui_.color, SIGNAL(colorEdited(const QColor&)), this,
            SLOT(SetColor(const QColor&)));
    QObject::connect(ui_.show_laps, SIGNAL(toggled(bool)), this,
            SLOT(LapToggled(bool)));
    QObject::connect(ui_.buttonResetBuffer, SIGNAL(pressed()), this,
                     SLOT(ClearPoints()));
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
    std::string topic = ui_.topic->text().trimmed().toStdString();
    if (topic != topic_)
    {
      initialized_ = false;
      ClearPoints();
      has_message_ = false;
      PrintWarning("No messages received.");

      gps_sub_.shutdown();

      topic_ = topic;
      if (!topic.empty())
      {
        gps_sub_ = node_.subscribe(topic_, 1, &GpsPlugin::GPSFixCallback, this);

        ROS_INFO("Subscribing to %s", topic_.c_str());
      }
    }
  }

  void GpsPlugin::GPSFixCallback(const gps_common::GPSFixConstPtr& gps)
  {  
    if (!tf_manager_->LocalXyUtil()->Initialized())
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
    stamped_point.source_frame = tf_manager_->LocalXyUtil()->Frame();
    double x;
    double y;
    tf_manager_->LocalXyUtil()->ToLocalXy(gps->latitude, gps->longitude, x, y);

    stamped_point.point = tf::Point(x, y, gps->altitude);

    // The GPS "track" is in degrees, but createQuaternionFromYaw expects
    // radians.
    // Furthermore, the track rotates in the opposite direction and is also
    // offset by 90 degrees, so all of that has to be compensated for.
    stamped_point.orientation =
        tf::createQuaternionFromYaw((-gps->track * (M_PI / 180.0)) + M_PI_2);

    pushPoint( std::move( stamped_point) );
  }

  void GpsPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void GpsPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void GpsPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  QWidget* GpsPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool GpsPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;
    SetColor(ui_.color->color());

    return true;
  }

  void GpsPlugin::Draw(double x, double y, double scale)
  {
    if (DrawPoints(scale))
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
      QColor qcolor(color.c_str());
      SetColor(qcolor);
      ui_.color->setColor(qcolor);
    }

    if (node["draw_style"])
    {
      std::string draw_style;
      node["draw_style"] >> draw_style;

      if (draw_style == "lines")
      {
        ui_.drawstyle->setCurrentIndex(0);
        SetDrawStyle( LINES );
      }
      else if (draw_style == "points")
      {
        ui_.drawstyle->setCurrentIndex(1);
        SetDrawStyle( POINTS );
      }
      else if (draw_style == "arrows")
      {
        ui_.drawstyle->setCurrentIndex(2);
        SetDrawStyle( ARROWS );
      }
    }

    if (node["position_tolerance"])
    {
      double position_tolerance;
      node["position_tolerance"] >> position_tolerance;
      ui_.positiontolerance->setValue(position_tolerance);
      PositionToleranceChanged(position_tolerance);
    }

    if (node["buffer_size"])
    {
      double buffer_size;
      node["buffer_size"] >> buffer_size;
      ui_.buffersize->setValue(buffer_size);
      BufferSizeChanged(buffer_size);
    }

    if (node["show_laps"])
    {
      bool show_laps = false;
      node["show_laps"] >> show_laps;
      ui_.show_laps->setChecked(show_laps);
      LapToggled(show_laps);
    }

    if (node["static_arrow_sizes"])
    {
      bool static_arrow_sizes = node["static_arrow_sizes"].as<bool>();
      ui_.static_arrow_sizes->setChecked(static_arrow_sizes);
      SetStaticArrowSizes(static_arrow_sizes);
    }

    if (node["arrow_size"])
    {
      int arrow_size = node["arrow_size"].as<int>();
      ui_.arrow_size->setValue(arrow_size);
      SetArrowSize(arrow_size);
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

    emitter << YAML::Key << "position_tolerance" <<
               YAML::Value << positionTolerance();

    emitter << YAML::Key << "buffer_size" << YAML::Value << bufferSize();

    bool show_laps = ui_.show_laps->isChecked();
    emitter << YAML::Key << "show_laps" << YAML::Value << show_laps;

    emitter << YAML::Key << "static_arrow_sizes" << YAML::Value << ui_.static_arrow_sizes->isChecked();

    emitter << YAML::Key << "arrow_size" << YAML::Value << ui_.arrow_size->value();
  }
}
