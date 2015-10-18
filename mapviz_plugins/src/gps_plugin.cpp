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
#include <QColorDialog>
#include <QGLWidget>
#include <QPalette>

#include <opencv2/core/core.hpp>

// ROS libraries
#include <ros/master.h>

#include <swri_image_util/geometry_util.h>
#include <swri_transform_util/transform_util.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
    mapviz_plugins,
    gps,
    mapviz_plugins::GpsPlugin,
    mapviz::MapvizPlugin);

namespace mapviz_plugins
{
  GpsPlugin::GpsPlugin() :
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
    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
    QObject::connect(ui_.positiontolerance, SIGNAL(valueChanged(double)), this, SLOT(PositionToleranceChanged(double)));
    QObject::connect(ui_.buffersize, SIGNAL(valueChanged(int)), this, SLOT(BufferSizeChanged(int)));
    QObject::connect(ui_.drawstyle, SIGNAL(activated(QString)), this, SLOT(SetDrawStyle(QString)));
  }

  GpsPlugin::~GpsPlugin()
  {
  }

  void GpsPlugin::DrawIcon()
  {
    if (icon_)
    {
      QPixmap icon(16, 16);
      icon.fill(Qt::transparent);

      QPainter painter(&icon);
      painter.setRenderHint(QPainter::Antialiasing, true);

      QPen pen(color_);

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

      icon_->SetPixmap(icon);
    }
  }

  void GpsPlugin::SetDrawStyle(QString style)
  {
    if (style == "lines")
    {
      draw_style_ = LINES;
    }
    else if (style == "points")
    {
      draw_style_ = POINTS;
    }

    DrawIcon();
    canvas_->update();
  }

  void GpsPlugin::SelectTopic()
  {
    QDialog dialog;
    Ui::topicselect ui;
    ui.setupUi(&dialog);

    std::vector<ros::master::TopicInfo> topics;
    ros::master::getTopics(topics);

    for (unsigned int i = 0; i < topics.size(); i++)
    {
      if (topics[i].datatype == "sensor_msgs/NavSatFix")
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

  void GpsPlugin::SelectColor()
  {
    QColorDialog dialog(color_, config_widget_);
    dialog.exec();

    if (dialog.result() == QDialog::Accepted)
    {
      color_ = dialog.selectedColor();
      ui_.selectcolor->setStyleSheet("background: " + color_.name() + ";");
      DrawIcon();
      canvas_->update();
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
      gps_sub_ = node_.subscribe(topic_, 1, &GpsPlugin::NavSatFixCallback, this);

      ROS_INFO("Subscribing to %s", topic_.c_str());
    }
  }

  void GpsPlugin::NavSatFixCallback(const sensor_msgs::NavSatFixConstPtr gps)
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

    double x;
    double y;
    local_xy_util_.ToLocalXy(gps->latitude, gps->longitude, x, y);

    stamped_point.point = tf::Point(x, y, gps->altitude);

    stamped_point.orientation = tf::createQuaternionFromYaw(0.0);

    if (points_.empty() || stamped_point.point.distance(points_.back().point) >= position_tolerance_)
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

    canvas_->update();
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

    canvas_->update();
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

      swri_transform_util::Transform tf;
    }

    glEnd();

    if (transformed)
    {
      PrintInfo("OK");
    }
  }

  bool GpsPlugin::TransformPoint(StampedPoint& point)
  {
    swri_transform_util::Transform transform;
    if (GetTransform(point.stamp, transform))
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

  void GpsPlugin::Transform()
  {
    if (source_frame_.empty())
    {
      if (!local_xy_util_.Initialized())
      {
        return;
      }
      source_frame_ = local_xy_util_.Frame();
    }
    bool transformed = false;

    std::list<StampedPoint>::iterator points_it = points_.begin();
    for (; points_it != points_.end(); ++points_it)
    {
      transformed = transformed | TransformPoint(*points_it);
    }

    transformed = transformed | TransformPoint(cur_point_);

    if (!points_.empty() && !transformed)
    {
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }
  }

  void GpsPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    std::string topic;
    node["topic"] >> topic;
    ui_.topic->setText(topic.c_str());

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

    TopicEdited();
  }

  void GpsPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "topic" << YAML::Value << topic;

    std::string color = color_.name().toStdString();
    emitter << YAML::Key << "color" << YAML::Value << color;

    std::string draw_style = ui_.drawstyle->currentText().toStdString();
    emitter << YAML::Key << "draw_style" << YAML::Value << draw_style;

    emitter << YAML::Key << "position_tolerance" << YAML::Value << position_tolerance_;

    emitter << YAML::Key << "buffer_size" << YAML::Value << buffer_size_;
  }
}

