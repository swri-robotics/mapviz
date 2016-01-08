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
PLUGINLIB_DECLARE_CLASS(
    mapviz_plugins,
    tf_frame,
    mapviz_plugins::TfFramePlugin,
    mapviz::MapvizPlugin);

namespace mapviz_plugins
{
  TfFramePlugin::TfFramePlugin() :
    config_widget_(new QWidget()),
    draw_style_(LINES)
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

    QObject::connect(ui_.selectframe, SIGNAL(clicked()), this, SLOT(SelectFrame()));
    QObject::connect(ui_.frame, SIGNAL(editingFinished()), this, SLOT(FrameEdited()));
    QObject::connect(ui_.positiontolerance, SIGNAL(valueChanged(double)), this, SLOT(PositionToleranceChanged(double)));
    QObject::connect(ui_.buffersize, SIGNAL(valueChanged(int)), this, SLOT(BufferSizeChanged(int)));
    QObject::connect(ui_.drawstyle, SIGNAL(activated(QString)), this, SLOT(SetDrawStyle(QString)));
    connect(ui_.color, SIGNAL(colorEdited(const QColor &)),
            this, SLOT(DrawIcon()));
  }

  TfFramePlugin::~TfFramePlugin()
  {
  }

  void TfFramePlugin::DrawIcon()
  {
    if (icon_)
    {
      QPixmap icon(16, 16);
      icon.fill(Qt::transparent);
      
      QPainter painter(&icon);
      painter.setRenderHint(QPainter::Antialiasing, true);
      
      QPen pen(ui_.color->color());
      
      if (draw_style_ == POINTS)
      {
        pen.setWidth(7);
        pen.setCapStyle(Qt::RoundCap);
        painter.setPen(pen);
        painter.drawPoint(8, 8);
      }
      else if (draw_style_ == LINES)
      {
        pen.setWidth(3);
        pen.setCapStyle(Qt::FlatCap);
        painter.setPen(pen);
        painter.drawLine(1, 14, 14, 1);
      }
      else if (draw_style_ == ARROWS)
      {
        pen.setWidth(2);
        pen.setCapStyle(Qt::SquareCap);
        painter.setPen(pen);
        painter.drawLine(2, 13, 13, 2);
        painter.drawLine(13, 2, 13, 8);
        painter.drawLine(13, 2, 7, 2);
      }
      
      icon_->SetPixmap(icon);
    }
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

    canvas_->update();
  }
  
  void TfFramePlugin::SetDrawStyle(QString style)
  {
    if (style == "lines")
    {
      draw_style_ = LINES;
    }
    else if (style == "points")
    {
      draw_style_ = POINTS;
    }
    else if (style == "arrows")
    {
      draw_style_ = ARROWS;
    }

    DrawIcon();
    canvas_->update();
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

    canvas_->update();
  }
  
  void TfFramePlugin::TimerCallback(const ros::TimerEvent& event)
  {
    swri_transform_util::Transform transform;
    if (GetTransform(ros::Time(), transform))
    {
      StampedPoint stamped_point;
      stamped_point.point = transform.GetOrigin();
      stamped_point.orientation = transform.GetOrientation();
      stamped_point.frame = target_frame_;
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
      
      if (canvas_)
        canvas_->update();
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

    connect(ui_.color, SIGNAL(colorEdited(const QColor &)),
            canvas_, SLOT(update()));          

    timer_ = node_.createTimer(ros::Duration(0.1), &TfFramePlugin::TimerCallback, this);

    DrawIcon();

    return true;
  }

  void TfFramePlugin::Draw(double x, double y, double scale)
  {
    QColor color = ui_.color->color();
    
    bool transformed = false;

    glColor4f(color.redF(), color.greenF(), color.blueF(), 1.0);

    if (draw_style_ == ARROWS)
    {
      transformed = DrawArrows();
    }
    else
    {
      if (draw_style_ == LINES)
      {
        glLineWidth(3);
        glBegin(GL_LINE_STRIP);
      }
      else
      {
        glPointSize(6);
        glBegin(GL_POINTS);
      }

      std::list<StampedPoint>::iterator it = points_.begin();
      for (; it != points_.end(); ++it)
      {
        if (it->transformed)
        {
          glVertex2f(it->transformed_point.getX(), it->transformed_point.getY());

          transformed = true;
        }
      }

      if (cur_point_.transformed)
      {
        glVertex2f(
          cur_point_.transformed_point.getX(),
          cur_point_.transformed_point.getY());

        transformed = true;
      }

      glEnd();
    }

    if (transformed)
    {
      PrintInfo("OK");
    }
  }
  
  bool TfFramePlugin::DrawArrows()
  {
    bool transformed = false;
    glLineWidth(2);
    glBegin(GL_LINES);

    std::list<StampedPoint>::iterator it = points_.begin();
    for (; it != points_.end(); ++it)
    {
      if (it->transformed)
      {
        glVertex2f(it->transformed_point.getX(), it->transformed_point.getY());
        glVertex2f(it->transformed_arrow_point.getX(), it->transformed_arrow_point.getY());

        glVertex2f(it->transformed_arrow_point.getX(), it->transformed_arrow_point.getY());
        glVertex2f(it->transformed_arrow_left.getX(), it->transformed_arrow_left.getY());

        glVertex2f(it->transformed_arrow_point.getX(), it->transformed_arrow_point.getY());
        glVertex2f(it->transformed_arrow_right.getX(), it->transformed_arrow_right.getY());

        transformed = true;
      }
    }

    if (cur_point_.transformed)
    {
      glVertex2f(
        cur_point_.transformed_point.getX(),
        cur_point_.transformed_point.getY());
      glVertex2f(
        cur_point_.transformed_arrow_point.getX(),
        cur_point_.transformed_arrow_point.getY());

      glVertex2f(
        cur_point_.transformed_arrow_point.getX(),
        cur_point_.transformed_arrow_point.getY());
      glVertex2f(
        cur_point_.transformed_arrow_left.getX(),
        cur_point_.transformed_arrow_left.getY());

      glVertex2f(
        cur_point_.transformed_arrow_point.getX(),
        cur_point_.transformed_arrow_point.getY());
      glVertex2f(
        cur_point_.transformed_arrow_right.getX(),
        cur_point_.transformed_arrow_right.getY());

      transformed = true;
    }


    glEnd();

    return transformed;
  }

  bool TfFramePlugin::TransformPoint(StampedPoint& point)
  {
    swri_transform_util::Transform transform;
    if (GetTransform(point.frame, point.stamp, transform))
    {
      point.transformed_point = transform * point.point;

      tf::Transform orientation(tf::Transform(transform.GetOrientation()) * point.orientation);
      point.transformed_arrow_point = point.transformed_point + orientation * tf::Point(1.0, 0.0, 0.0);
      point.transformed_arrow_left = point.transformed_point + orientation * tf::Point(0.75, -0.2, 0.0);
      point.transformed_arrow_right = point.transformed_point + orientation * tf::Point(0.75, 0.2, 0.0);

      point.transformed = true;
      return true;
    }

     point.transformed = false;
     return false;
  }

  void TfFramePlugin::Transform()
  {
    bool transformed = false;

    std::list<StampedPoint>::iterator points_it = points_.begin();
    for (; points_it != points_.end(); ++points_it)
    {
      transformed = transformed | TransformPoint(*points_it);
    }

    transformed = transformed | TransformPoint(cur_point_);

    if (!points_.empty() && !transformed)
    {
      PrintError("No transform error.");
    }
  }

  void TfFramePlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    node["frame"] >> source_frame_;
    ui_.frame->setText(source_frame_.c_str());

    std::string color;
    node["color"] >> color;
    ui_.color->setColor(QColor(color.c_str()));

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

    node["position_tolerance"] >> position_tolerance_;
    ui_.positiontolerance->setValue(position_tolerance_);

    node["buffer_size"] >> buffer_size_;
    ui_.buffersize->setValue(buffer_size_);

    FrameEdited();
  }

  void TfFramePlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "frame" << YAML::Value << ui_.frame->text().toStdString();    
    emitter << YAML::Key << "color" << YAML::Value << ui_.color->color().name().toStdString();

    std::string draw_style = ui_.drawstyle->currentText().toStdString();
    emitter << YAML::Key << "draw_style" << YAML::Value << draw_style;

    emitter << YAML::Key << "position_tolerance" << YAML::Value << position_tolerance_;

    emitter << YAML::Key << "buffer_size" << YAML::Value << buffer_size_;
  }
}

