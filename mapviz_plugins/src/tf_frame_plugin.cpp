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

#include <mapviz_plugins/tf_frame_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <algorithm>
#include <vector>

// QT libraries
#include <QGLWidget>
#include <QPalette>

// ROS libraries
#include <ros/master.h>

#include <mapviz/select_frame_dialog.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(mapviz_plugins,
                        tf_frame,
                        mapviz_plugins::TfFramePlugin,
                        mapviz::MapvizPlugin);

namespace mapviz_plugins
{
  TfFramePlugin::TfFramePlugin() : config_widget_(new QWidget())
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

    QObject::connect(ui_.selectframe, SIGNAL(clicked()), this,
                     SLOT(SelectFrame()));
    QObject::connect(ui_.frame, SIGNAL(editingFinished()), this,
                     SLOT(FrameEdited()));
    QObject::connect(ui_.positiontolerance, SIGNAL(valueChanged(double)), this,
                     SLOT(PositionToleranceChanged(double)));
    QObject::connect(ui_.buffersize, SIGNAL(valueChanged(int)), this,
                     SLOT(BufferSizeChanged(int)));
    QObject::connect(ui_.drawstyle, SIGNAL(activated(QString)), this,
                     SLOT(SetDrawStyle(QString)));
    connect(ui_.color, SIGNAL(colorEdited(const QColor&)), this,
            SLOT(DrawIcon()));
  }

  TfFramePlugin::~TfFramePlugin()
  {
  }

  void TfFramePlugin::SelectFrame()
  {
    std::string frame = mapviz::SelectFrameDialog::selectFrame(tf_);
    if (!frame.empty())
    {
      ui_.frame->setText(QString::fromStdString(frame));
      FrameEdited();
    }
  }

  void TfFramePlugin::FrameEdited()
  {
    source_frame_ = ui_.frame->text().toStdString();
    PrintWarning("Waiting for transform.");

    ROS_INFO("Setting target frame to to %s", source_frame_.c_str());

    initialized_ = true;
  }

  void TfFramePlugin::PositionToleranceChanged(double value)
  {
    position_tolerance_ = value;
  }

  void TfFramePlugin::BufferSizeChanged(int value)
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

  void TfFramePlugin::TimerCallback(const ros::TimerEvent& event)
  {
    swri_transform_util::Transform transform;
    if (GetTransform(ros::Time(), transform))
    {
      StampedPoint stamped_point;
      stamped_point.point = transform.GetOrigin();
      stamped_point.orientation = transform.GetOrientation();
      stamped_point.source_frame = target_frame_;
      stamped_point.stamp = transform.GetStamp();
      stamped_point.transformed = false;

      double distance = std::sqrt(
          std::pow(stamped_point.point.x() - points_.back().point.x(), 2) +
          std::pow(stamped_point.point.y() - points_.back().point.y(), 2));

      if (points_.empty() || distance >= position_tolerance_)
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
  }

  void TfFramePlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_ERROR("Error: %s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void TfFramePlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_INFO("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void TfFramePlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_WARN("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  QWidget* TfFramePlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool TfFramePlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;

    timer_ = node_.createTimer(ros::Duration(0.1),
                               &TfFramePlugin::TimerCallback, this);

    DrawIcon();

    return true;
  }

  void TfFramePlugin::Draw(double x, double y, double scale)
  {
    color_ = ui_.color->color();
    if (DrawPoints())
    {
      PrintInfo("OK");
    }
  }
  void TfFramePlugin::LoadConfig(const YAML::Node& node,
                                 const std::string& path)
  {
    if (node["frame"])
    {
      node["frame"] >> source_frame_;
      ui_.frame->setText(source_frame_.c_str());
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

    FrameEdited();
  }

  void TfFramePlugin::SaveConfig(YAML::Emitter& emitter,
                                 const std::string& path)
  {
    emitter << YAML::Key << "frame" << YAML::Value
            << ui_.frame->text().toStdString();
    emitter << YAML::Key << "color" << YAML::Value
            << ui_.color->color().name().toStdString();

    std::string draw_style = ui_.drawstyle->currentText().toStdString();
    emitter << YAML::Key << "draw_style" << YAML::Value << draw_style;

    emitter << YAML::Key << "position_tolerance" <<
               YAML::Value << position_tolerance_;

    emitter << YAML::Key << "buffer_size" << YAML::Value << buffer_size_;
  }
}
