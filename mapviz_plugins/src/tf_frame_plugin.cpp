// *****************************************************************************
//
// Copyright (C) 2014 All Right Reserved, Southwest Research Institute® (SwRI®)
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

// C++ standard libraries
#include <cstdio>
#include <algorithm>
#include <vector>

// QT libraries
#include <QColorDialog>
#include <QDialog>
#include <QGLWidget>
#include <QPalette>

// ROS libraries
#include <ros/master.h>

#include <mapviz_plugins/tf_frame_plugin.h>

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
    color_(Qt::green),
    draw_style_(LINES)
  {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    // Initialize color selector color
    ui_.selectcolor->setStyleSheet("background: " + color_.name() + ";");

    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.selectcolor, SIGNAL(clicked()), this, SLOT(SelectColor()));
    QObject::connect(ui_.selectframe, SIGNAL(clicked()), this, SLOT(SelectFrame()));
    QObject::connect(ui_.frame, SIGNAL(editingFinished()), this, SLOT(FrameEdited()));
    QObject::connect(ui_.positiontolerance, SIGNAL(valueChanged(double)), this, SLOT(PositionToleranceChanged(double)));
    QObject::connect(ui_.buffersize, SIGNAL(valueChanged(int)), this, SLOT(BufferSizeChanged(int)));
    QObject::connect(ui_.drawstyle, SIGNAL(activated(QString)), this, SLOT(SetDrawStyle(QString)));
  }

  TfFramePlugin::~TfFramePlugin()
  {
  }

  void TfFramePlugin::SelectFrame()
  {
    QDialog dialog;
    Ui::topicselect ui;
    ui.setupUi(&dialog);

    std::vector<std::string> frames;
    tf_->getFrameStrings(frames);

    for (unsigned int i = 0; i < frames.size(); i++)
    {
      ui.displaylist->addItem(frames[i].c_str());
    }
    ui.displaylist->setCurrentRow(0);

    dialog.exec();

    if (dialog.result() == QDialog::Accepted && ui.displaylist->selectedItems().count() == 1)
    {
      ui_.frame->setText(ui.displaylist->selectedItems().first()->text());
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

    canvas_->update();
  }
  
  void TfFramePlugin::SelectColor()
  {
    QColorDialog dialog(color_, config_widget_);
    dialog.exec();

    if (dialog.result() == QDialog::Accepted)
    {
      color_ = dialog.selectedColor();
      ui_.selectcolor->setStyleSheet("background: " + color_.name() + ";");
      canvas_->update();
    }
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
    transform_util::Transform transform;
    if (GetTransform(ros::Time(), transform))
    {
      StampedPoint stamped_point;
      stamped_point.point = transform.GetOrigin();
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

    timer_ = node_.createTimer(ros::Duration(0.1), &TfFramePlugin::TimerCallback, this);

    return true;
  }

  void TfFramePlugin::Draw(double x, double y, double scale)
  {
    glColor4f(color_.redF(), color_.greenF(), color_.blueF(), 0.5);

    bool transformed = false;

    glColor4f(color_.redF(), color_.greenF(), color_.blueF(), 1.0);

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

    if (transformed)
    {
      PrintInfo("OK");
    }
  }

  bool TfFramePlugin::TransformPoint(StampedPoint& point)
  {
    transform_util::Transform transform;
    if (GetTransform(point.frame, point.stamp, transform))
    {
      point.transformed_point = transform * point.point;

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
    color_ = QColor(color.c_str());
    ui_.selectcolor->setStyleSheet("background: " + color_.name() + ";");

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
    
    std::string color = color_.name().toStdString();
    emitter << YAML::Key << "color" << YAML::Value << color;

    std::string draw_style = ui_.drawstyle->currentText().toStdString();
    emitter << YAML::Key << "draw_style" << YAML::Value << draw_style;

    emitter << YAML::Key << "position_tolerance" << YAML::Value << position_tolerance_;

    emitter << YAML::Key << "buffer_size" << YAML::Value << buffer_size_;
  }
}

