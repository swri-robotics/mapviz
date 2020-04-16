/**
 * Copyright 2019 Hatchbed L.L.C.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 **/ 

#include <mapviz_plugins/pose_plugin.h>

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
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::PosePlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  PosePlugin::PosePlugin() : config_widget_(new QWidget())
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

  PosePlugin::~PosePlugin()
  {
  }

  void PosePlugin::SelectTopic()
  {
    ros::master::TopicInfo topic =
        mapviz::SelectTopicDialog::selectTopic("geometry_msgs/PoseStamped");

    if (!topic.name.empty())
    {
      ui_.topic->setText(QString::fromStdString(topic.name));
      TopicEdited();
    }
  }

  void PosePlugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();
    if (topic != topic_)
    {
      initialized_ = false;
      ClearPoints();
      has_message_ = false;
      PrintWarning("No messages received.");

      pose_sub_.shutdown();

      topic_ = topic;
      if (!topic.empty())
      {
        pose_sub_ = node_.subscribe(topic_, 10, &PosePlugin::PoseCallback, this);

        ROS_INFO("Subscribing to %s", topic_.c_str());
      }
    }
  }

  void PosePlugin::PoseCallback(const geometry_msgs::PoseStampedConstPtr& pose)
  {  
    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }

    StampedPoint stamped_point;
    stamped_point.stamp = pose->header.stamp;
    stamped_point.source_frame = pose->header.frame_id;

    stamped_point.point = tf::Point(pose->pose.position.x,
                                    pose->pose.position.y,
                                    pose->pose.position.z);

    stamped_point.orientation = tf::Quaternion(
        pose->pose.orientation.x,
        pose->pose.orientation.y,
        pose->pose.orientation.z,
        pose->pose.orientation.w);

    pushPoint( std::move( stamped_point) );
  }

  void PosePlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void PosePlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void PosePlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  QWidget* PosePlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool PosePlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;
    SetColor(ui_.color->color());

    return true;
  }

  void PosePlugin::Draw(double x, double y, double scale)
  {
    if (DrawPoints(scale))
    {
      PrintInfo("OK");
    }
  }

  void PosePlugin::LoadConfig(const YAML::Node& node, const std::string& path)
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

  void PosePlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
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
