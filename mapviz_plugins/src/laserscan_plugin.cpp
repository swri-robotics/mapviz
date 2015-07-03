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

#include <mapviz_plugins/laserscan_plugin.h>

// C++ standard libraries
#include <cmath>
#include <cstdio>
#include <vector>

// Boost libraries
#include <boost/algorithm/string.hpp>

// QT libraries
#include <QColorDialog>
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
    config_widget_(new QWidget()),
    min_color_(Qt::white),
    max_color_(Qt::white),
    min_intensity_(0.0),
    max_intensity_(100.0),
    point_size_(3),
    buffer_size_(1)
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
    
    // Initialize color selector colors
    ui_.selectMinColor->setStyleSheet("background: " + min_color_.name() + ";");
    ui_.selectMaxColor->setStyleSheet("background: " + max_color_.name() + ";");

    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
    QObject::connect(ui_.selectMinColor, SIGNAL(clicked()), this, SLOT(SelectMinColor()));
    QObject::connect(ui_.selectMaxColor, SIGNAL(clicked()), this, SLOT(SelectMaxColor()));
    QObject::connect(ui_.minIntensity, SIGNAL(valueChanged(double)), this, SLOT(MinIntensityChanged(double)));
    QObject::connect(ui_.maxIntensity, SIGNAL(valueChanged(double)), this, SLOT(MaxIntensityChanged(double)));
    QObject::connect(ui_.bufferSize, SIGNAL(valueChanged(int)), this, SLOT(BufferSizeChanged(int)));
    QObject::connect(ui_.pointSize, SIGNAL(valueChanged(int)), this, SLOT(PointSizeChanged(int)));
  }

  LaserScanPlugin::~LaserScanPlugin()
  {
  }

  QColor LaserScanPlugin::InterpolateColors(
    const QColor& c1, 
    const QColor& c2, 
    double weight) const
  {
    QColor color;
    
    double r = c1.redF() * weight + c2.redF() * (1.0 - weight);
    double g = c1.greenF() * weight + c2.greenF() * (1.0 - weight);
    double b = c1.blueF() * weight + c2.blueF() * (1.0 - weight);
    
    color.setRgbF(r, g, b);
    
    return color;
  }

  void LaserScanPlugin::UpdateColors()
  {
    for (size_t i = 0; i < scans_.size(); i++)
    {
      for (size_t j = 0; j < scans_[i].points.size(); j++)
      {
        StampedPoint& point = scans_[i].points[j];
        double weight = std::min(1.0, std::max(
          0.0, (point.intensity - min_intensity_) / max_intensity_));
        point.color = InterpolateColors(min_color_, max_color_, weight);
      }
    }
  }

  void LaserScanPlugin::DrawIcon()
  {
    if (icon_)
    {
      QPixmap icon(16, 16);
      icon.fill(Qt::transparent);

      QPainter painter(&icon);
      painter.setRenderHint(QPainter::Antialiasing, true);

      QPen pen;
      pen.setWidth(4);
      pen.setCapStyle(Qt::RoundCap);
      
      pen.setColor(InterpolateColors(min_color_, max_color_, 0.2));
      painter.setPen(pen);
      painter.drawPoint(2, 13);
      
      pen.setColor(InterpolateColors(min_color_, max_color_, 0.6));
      painter.setPen(pen);
      painter.drawPoint(4, 6);
      
      pen.setColor(InterpolateColors(min_color_, max_color_, 0.4));
      painter.setPen(pen);
      painter.drawPoint(12, 9);
      
      pen.setColor(InterpolateColors(min_color_, max_color_, 0.8));
      painter.setPen(pen);
      painter.drawPoint(13, 2);

      icon_->SetPixmap(icon);
    }
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
      scans_.clear();
      has_message_ = false;
      topic_ = boost::trim_copy(ui_.topic->text().toStdString());
      PrintWarning("No messages received.");

      laserscan_sub_.shutdown();
      laserscan_sub_ = node_.subscribe(topic_, 100, &LaserScanPlugin::laserScanCallback, this);

      ROS_INFO("Subscribing to %s", topic_.c_str());
    }
  }
  
  void LaserScanPlugin::SelectMinColor()
  {
    QColorDialog dialog(min_color_, config_widget_);
    dialog.exec();

    if (dialog.result() == QDialog::Accepted)
    {
      min_color_ = dialog.selectedColor();
      ui_.selectMinColor->setStyleSheet("background: " + min_color_.name() + ";");
      DrawIcon();
      UpdateColors();
      canvas_->update();
    }
  }
  
  void LaserScanPlugin::SelectMaxColor()
  {
    QColorDialog dialog(max_color_, config_widget_);
    dialog.exec();

    if (dialog.result() == QDialog::Accepted)
    {
      max_color_ = dialog.selectedColor();
      ui_.selectMaxColor->setStyleSheet("background: " + max_color_.name() + ";");
      DrawIcon();
      UpdateColors();
      canvas_->update();
    }
  }

  void LaserScanPlugin::MinIntensityChanged(double value)
  {
    min_intensity_ = value;
    UpdateColors();
    canvas_->update();
  }

  void LaserScanPlugin::MaxIntensityChanged(double value)
  {
    max_intensity_ = value;
    UpdateColors();
    canvas_->update();
  }

  void LaserScanPlugin::BufferSizeChanged(int value)
  {
    buffer_size_ = value;

    if (buffer_size_ > 0)
    {
      while (static_cast<int>(scans_.size()) > buffer_size_)
      {
        scans_.pop_front();
      }
    }

    canvas_->update();
  }

  void LaserScanPlugin::PointSizeChanged(int value)
  {
    point_size_ = value;

    canvas_->update();
  }

  void LaserScanPlugin::laserScanCallback(const sensor_msgs::LaserScanConstPtr msg)
  {
    if (!has_message_)
    {
      source_frame_ = msg->header.frame_id;
      initialized_ = true;
      has_message_ = true;
    }

    Scan scan;
    scan.stamp = msg->header.stamp;
    scan.transformed = true;
    scan.points.reserve(msg->ranges.size());
    scan.has_intensity = msg->ranges.size() == msg->intensities.size();

    double angle, x, y;
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
      // Discard the point if it's out of range
      if( msg->ranges[i] > msg->range_max || msg->ranges[i] < msg->range_min)
      {
        continue;
      }
      StampedPoint point;
      angle = msg->angle_min + msg->angle_increment * i;
      x = cos(angle) * msg->ranges[i];
      y = sin(angle) * msg->ranges[i];
      point.point = tf::Point(x, y, 0.0f);
      
      if (scan.has_intensity)
      {
        point.intensity = msg->intensities[i];
        
        double weight = std::min(1.0, std::max(
          0.0, (point.intensity - min_intensity_) / max_intensity_));
        point.color = InterpolateColors(min_color_, max_color_, weight);
      }
      else
      {
        point.color = min_color_;
      }
      
      scan.points.push_back(point);
    }

    transform_util::Transform transform;
    if (GetTransform(msg->header.stamp, transform))
    {
      for (size_t i = 0; i < scan.points.size(); i++)
      {
        scan.points[i].transformed_point = transform * scan.points[i].point;
      }
    }
    else
    {
      scan.transformed = false;
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }
    
    scans_.push_back(scan);
    
    if (buffer_size_ > 0)
    {
      while (static_cast<int>(scans_.size()) > buffer_size_)
      {
        scans_.pop_front();
      }
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

    DrawIcon();

    return true;
  }

  void LaserScanPlugin::Draw(double x, double y, double scale)
  {
    ros::Time now = ros::Time::now();

    bool success = false;
    for (size_t i = 0; i < scans_.size(); i++)
    {
      const Scan& scan = scans_[i];
      if (scan.transformed)
      {
        glPointSize(point_size_);
        glBegin(GL_POINTS);

        for (size_t j = 0; j < scan.points.size(); j++)
        {
          const StampedPoint& point = scan.points[j];
          glColor3f(
              point.color.redF(),
              point.color.greenF(),
              point.color.blueF());
          glVertex2f(
              point.transformed_point.getX(),
              point.transformed_point.getY());
        }

        glEnd();

        success = true;
      }
    }
    
    if (success)
    {
      PrintInfo("OK");
    }
  }

  void LaserScanPlugin::Transform()
  {
    for (size_t i = 0; i < scans_.size(); i++)
    {
      Scan& scan = scans_[i];
      
      transform_util::Transform transform;
      if (GetTransform(scan.stamp, transform))
      {
        scan.transformed = true;

        for (size_t j = 0; j < scan.points.size(); j++)
        {
          StampedPoint& point = scan.points[j];
          point.transformed_point = transform * point.point;
        }
      }
      else
      {
        scan.transformed = false;
      }
    }
  }

  void LaserScanPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    std::string topic;
    node["topic"] >> topic;
    ui_.topic->setText(boost::trim_copy(topic).c_str());

    std::string min_color;
    node["min_color"] >> min_color;
    min_color_ = QColor(min_color.c_str());
    ui_.selectMinColor->setStyleSheet("background: " + min_color_.name() + ";");

    std::string max_color;
    node["max_color"] >> max_color;
    max_color_ = QColor(max_color.c_str());
    ui_.selectMaxColor->setStyleSheet("background: " + max_color_.name() + ";");

    node["min_intensity"] >> min_intensity_;
    ui_.minIntensity->setValue(min_intensity_);

    node["max_intensity"] >> max_intensity_;
    ui_.maxIntensity->setValue(max_intensity_);

    node["point_size"] >> point_size_;
    ui_.pointSize->setValue(point_size_);

    node["buffer_size"] >> buffer_size_;
    ui_.bufferSize->setValue(buffer_size_);

    TopicEdited();
  }

  void LaserScanPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "topic" << YAML::Value << boost::trim_copy(ui_.topic->text().toStdString());
    std::string min_color = min_color_.name().toStdString();
    emitter << YAML::Key << "min_color" << YAML::Value << min_color;
    std::string max_color = max_color_.name().toStdString();
    emitter << YAML::Key << "max_color" << YAML::Value << max_color;
    emitter << YAML::Key << "min_intensity" << YAML::Value << min_intensity_;
    emitter << YAML::Key << "max_intensity" << YAML::Value << max_intensity_;
    emitter << YAML::Key << "point_size" << YAML::Value << point_size_;
    emitter << YAML::Key << "buffer_size" << YAML::Value << buffer_size_;
  }
}

