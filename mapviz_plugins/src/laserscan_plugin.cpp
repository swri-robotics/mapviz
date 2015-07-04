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
#include <algorithm>
#include <vector>

// Boost libraries
#include <boost/algorithm/string.hpp>

// QT libraries
#include <QColorDialog>
#include <QDialog>
#include <QGLWidget>

// OpenGL
#include <GL/glew.h>

// QT Autogenerated
#include "ui_topic_select.h"

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
          topic_(""),
          min_color_(Qt::white),
          max_color_(Qt::black),
          alpha_(1.0),
          min_value_(0.0),
          max_value_(100.0),
          point_size_(3)
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

    // Set color transformer choices
    ui_.color_transformer->addItem(QString("Flat Color"), QVariant(0));
    ui_.color_transformer->addItem(QString("Intensity"), QVariant(1));
    ui_.color_transformer->addItem(QString("Range"), QVariant(2));
    ui_.color_transformer->addItem(QString("X Axis"), QVariant(3));
    ui_.color_transformer->addItem(QString("Y Axis"), QVariant(4));
    ui_.color_transformer->addItem(QString("Z Axis"), QVariant(5));

    QObject::connect(ui_.selecttopic,
        SIGNAL(clicked()),
        this,
        SLOT(SelectTopic()));
    QObject::connect(ui_.topic,
        SIGNAL(editingFinished()),
        this,
        SLOT(TopicEdited()));
    QObject::connect(ui_.alpha,
        SIGNAL(editingFinished()),
        this,
        SLOT(AlphaEdited()));
    QObject::connect(ui_.color_transformer,
        SIGNAL(currentIndexChanged(int)),
        this,
        SLOT(ColorTransformerChanged(int)));
    QObject::connect(ui_.selectMaxColor,
        SIGNAL(clicked()),
        this,
        SLOT(SelectMaxColor()));
    QObject::connect(ui_.selectMinColor,
        SIGNAL(clicked()),
        this,
        SLOT(SelectMinColor()));
    QObject::connect(ui_.minValue,
        SIGNAL(valueChanged(double)),
        this,
        SLOT(MinValueChanged(double)));
    QObject::connect(ui_.maxValue,
        SIGNAL(valueChanged(double)),
        this,
        SLOT(MaxValueChanged(double)));
    QObject::connect(ui_.bufferSize,
        SIGNAL(valueChanged(int)),
        this,
        SLOT(BufferSizeChanged(int)));
    QObject::connect(ui_.pointSize,
        SIGNAL(valueChanged(int)),
        this,
        SLOT(PointSizeChanged(int)));
    QObject::connect(ui_.use_rainbow,
        SIGNAL(stateChanged(int)),
        this,
        SLOT(UseRainbowChanged(int)));
    PrintInfo("Constructed LaserScanPlugin");
  }

  LaserScanPlugin::~LaserScanPlugin()
  {
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

      pen.setColor(min_color_);
      painter.setPen(pen);
      painter.drawPoint(2, 13);

      pen.setColor(min_color_);
      painter.setPen(pen);
      painter.drawPoint(4, 6);

      pen.setColor(min_color_);
      painter.setPen(pen);
      painter.drawPoint(12, 9);

      pen.setColor(min_color_);
      painter.setPen(pen);
      painter.drawPoint(13, 2);

      icon_->SetPixmap(icon);
    }
  }

  QColor LaserScanPlugin::CalculateColor(const StampedPoint& point,
      bool has_intensity)
  {
    double val;
    unsigned int color_transformer = ui_.color_transformer->currentIndex();
    if (color_transformer == COLOR_RANGE)
    {
      val = point.range;
    }
    else if (color_transformer == COLOR_INTENSITY && has_intensity)
    {
      val = point.intensity;
    }
    else if (color_transformer == COLOR_X)
    {
      val = point.point.x();
    }
    else if (color_transformer == COLOR_Y)
    {
      val = point.point.y();
    }
    else if (color_transformer == COLOR_Z)
    {
      val = point.point.z();
    }
    else  // No intensity or  (color_transformer == COLOR_FLAT)
    {
      return min_color_;
    }
    if (max_value_ > min_value_)
      val = (val - min_value_) / (max_value_ - min_value_);
    val = std::max(0.0, std::min(val, 1.0));
    if (ui_.use_rainbow->isChecked())
    {
      // Hue Interpolation
      int hue = val * 255;
      return QColor::fromHsl(hue, 255, 127, 255);
    }
    else
    {
      // RGB Interpolation
      int red, green, blue;
      red = val * max_color_.red() + ((1.0 - val) * min_color_.red());
      green = val * max_color_.green() + ((1.0 - val) * min_color_.green());
      blue = val * max_color_.blue() + ((1.0 - val) * min_color_.blue());
      return QColor(red, green, blue, 255);
    }
  }

  void LaserScanPlugin::UpdateColors()
  {
    std::list<Scan>::iterator scan_it = scans_.begin();
    for (; scan_it != scans_.end(); ++scan_it)
    {
      std::list<StampedPoint>::iterator point_it = scan_it->points.begin();
      for (; point_it != scan_it->points.end(); point_it++)
      {
        point_it->color = CalculateColor(*point_it, scan_it->has_intensity);
      }
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

    if (dialog.result() == QDialog::Accepted &&
        ui.displaylist->selectedItems().count() == 1)
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
      laserscan_sub_ = node_.subscribe(topic_,
          100,
          &LaserScanPlugin::laserScanCallback,
          this);

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

  void LaserScanPlugin::MinValueChanged(double value)
  {
    min_value_ = value;
    UpdateColors();
    canvas_->update();
  }

  void LaserScanPlugin::MaxValueChanged(double value)
  {
    max_value_ = value;
    UpdateColors();
    canvas_->update();
  }

  void LaserScanPlugin::BufferSizeChanged(int value)
  {
    buffer_size_ = value;

    if (buffer_size_ > 0)
    {
      while (scans_.size() > buffer_size_)
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

  void LaserScanPlugin::laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
  {
    if (!has_message_)
    {
      source_frame_ = msg->header.frame_id;
      initialized_ = true;
      has_message_ = true;
    }

    Scan scan;
    scan.stamp = msg->header.stamp;
    scan.color = QColor::fromRgbF(1.0f, 0.0f, 0.0f, 1.0f);
    scan.transformed = true;
    scan.has_intensity = !msg->intensities.empty();

    scan.points.clear();

    transform_util::Transform transform;
    if (!GetTransform(msg->header.stamp, transform))
    {
      scan.transformed = false;
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }
    double angle, x, y;
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
      // Discard the point if it's out of range
      if (msg->ranges[i] > msg->range_max || msg->ranges[i] < msg->range_min)
      {
        continue;
      }
      StampedPoint point;
      angle = msg->angle_min + msg->angle_increment * i;
      x = cos(angle) * msg->ranges[i];
      y = sin(angle) * msg->ranges[i];
      point.point = tf::Point(x, y, 0.0f);
      point.range = msg->ranges[i];
      if (i < msg->intensities.size())
        point.intensity = msg->intensities[i];
      if (scan.transformed)
      {
        point.transformed_point = transform * point.point;
      }
      point.color = CalculateColor(point, scan.has_intensity);
      scan.points.push_back(point);
    }
    scans_.push_back(scan);

    // If there are more items in the scan buffer than buffer_size_, remove them
    if (buffer_size_ > 0)
    {
      while (scans_.size() > buffer_size_)
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

    glPointSize(point_size_);
    glBegin(GL_POINTS);

    std::list<Scan>::const_iterator scan_it = scans_.begin();
    while (scan_it != scans_.end())
    {
      /* This code was for implementing time-based scan decay (rather than
       * queue-based). It is disabled for now. The iterator
       * will have to be changed from a const iterator to a non-const iterator
       * if this is re-implemented. (evenator)
      // Remove expired scan
      if ((now - scan_it->stamp).toSec() > decay_time_)
      {
        // Special case: If decay time is zero, don't erase
        // the last scan
        if (decay_time_ > 0.0f || scans_.size() > 1)
        {
          scan_it = scans_.erase(scan_it);
          continue;
        }
      }
      */
      if (scan_it->transformed)
      {
        std::list<StampedPoint>::const_iterator point_it = scan_it->points.begin();
        for (; point_it != scan_it->points.end(); ++point_it)
        {
          glColor4f(
              point_it->color.redF(),
              point_it->color.greenF(),
              point_it->color.blueF(),
              alpha_);
          glVertex2f(
              point_it->transformed_point.getX(),
              point_it->transformed_point.getY());
        }
      }
      ++scan_it;
    }

    glEnd();

    PrintInfo("OK");
  }

  void LaserScanPlugin::UseRainbowChanged(int check_state)
  {
    if (check_state == Qt::Checked)
    {
      ui_.selectMaxColor->setVisible(false);
      ui_.selectMinColor->setVisible(false);
      ui_.maxColorLabel->setVisible(false);
      ui_.minColorLabel->setVisible(false);
    }
    else
    {
      ui_.selectMaxColor->setVisible(true);
      ui_.selectMinColor->setVisible(true);
      ui_.maxColorLabel->setVisible(true);
      ui_.minColorLabel->setVisible(true);
    }
    UpdateColors();
    canvas_->update();
  }

  void LaserScanPlugin::Transform()
  {
    std::list<Scan>::iterator scan_it = scans_.begin();
    for (; scan_it != scans_.end(); ++scan_it)
    {
      Scan& scan = *scan_it;

      transform_util::Transform transform;
      if (GetTransform(scan.stamp, transform, false))
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
    // Z color is based on transformed color, so it is dependent on the
    // transform
    if (ui_.color_transformer->currentIndex() == COLOR_Z)
    {
      UpdateColors();
      canvas_->update();
    }
  }

  void LaserScanPlugin::LoadConfig(const YAML::Node& node,
      const std::string& path)
  {
    std::string topic;
    node["topic"] >> topic;
    ui_.topic->setText(boost::trim_copy(topic).c_str());

    TopicEdited();

    node["size"] >> point_size_;
    ui_.pointSize->setValue(point_size_);

    node["buffer_size"] >> buffer_size_;
    ui_.bufferSize->setValue(buffer_size_);

    std::string color_transformer;
    node["color_transformer"] >> color_transformer;
    if (color_transformer == "Intensity")
      ui_.color_transformer->setCurrentIndex(COLOR_INTENSITY);
    else if (color_transformer == "Range")
      ui_.color_transformer->setCurrentIndex(COLOR_RANGE);
    else if (color_transformer == "X Axis")
      ui_.color_transformer->setCurrentIndex(COLOR_X);
    else if (color_transformer == "Y Axis")
      ui_.color_transformer->setCurrentIndex(COLOR_Y);
    else if (color_transformer == "Z Axis")
      ui_.color_transformer->setCurrentIndex(COLOR_Z);
    else
      ui_.color_transformer->setCurrentIndex(COLOR_FLAT);

    std::string min_color_str;
    node["min_color"] >> min_color_str;
    min_color_ = QColor(min_color_str.c_str());
    ui_.selectMinColor->setStyleSheet("background: " + min_color_.name() + ";");

    std::string max_color_str;
    node["max_color"] >> max_color_str;
    max_color_ = QColor(max_color_str.c_str());
    ui_.selectMaxColor->setStyleSheet("background: " + max_color_.name() + ";");

    node["value_min"] >> min_value_;
    ui_.minValue->setValue(min_value_);
    node["value_max"] >> max_value_;
    ui_.maxValue->setValue(max_value_);
    node["alpha"] >> alpha_;
    ui_.alpha->setValue(alpha_);
    AlphaEdited();
    bool use_rainbow;
    node["use_rainbow"] >> use_rainbow;
    ui_.use_rainbow->setChecked(use_rainbow);
    // UseRainbowChanged must be called *before* ColorTransformerChanged
    UseRainbowChanged(ui_.use_rainbow->checkState());
    // ColorTransformerChanged will also update colors of all points
    ColorTransformerChanged(ui_.color_transformer->currentIndex());
  }

  void LaserScanPlugin::ColorTransformerChanged(int index)
  {
    ROS_DEBUG("Color transformer changed to %d", index);
    switch (index)
    {
      case COLOR_FLAT:
        ui_.selectMaxColor->setVisible(false);
        ui_.maxColorLabel->setVisible(false);
        ui_.minColorLabel->setVisible(false);
        ui_.minValueLabel->setVisible(false);
        ui_.maxValueLabel->setVisible(false);
        ui_.minValue->setVisible(false);
        ui_.maxValue->setVisible(false);
        ui_.use_rainbow->setVisible(false);
        break;
      case COLOR_INTENSITY:  // Intensity
      case COLOR_RANGE:  // Range
      case COLOR_X:  // X Axis
      case COLOR_Y:  // Y Axis
      case COLOR_Z:  // Z axis
      default:
        ui_.selectMinColor->setVisible(!ui_.use_rainbow->isChecked());
        ui_.selectMaxColor->setVisible(!ui_.use_rainbow->isChecked());
        ui_.maxColorLabel->setVisible(!ui_.use_rainbow->isChecked());
        ui_.minColorLabel->setVisible(!ui_.use_rainbow->isChecked());
        ui_.minValueLabel->setVisible(true);
        ui_.maxValueLabel->setVisible(true);
        ui_.minValue->setVisible(true);
        ui_.maxValue->setVisible(true);
        ui_.use_rainbow->setVisible(true);
        break;
    }
    UpdateColors();
    canvas_->update();
  }

  /**
   * Coerces alpha to [0.0, 1.0] and stores it in alpha_
   */
  void LaserScanPlugin::AlphaEdited()
  {
    alpha_ = std::max(0.0f, std::min(ui_.alpha->text().toFloat(), 1.0f));
    ui_.alpha->setValue(alpha_);
  }

  void LaserScanPlugin::SaveConfig(YAML::Emitter& emitter,
      const std::string& path)
  {
    emitter << YAML::Key << "topic" <<
               YAML::Value << boost::trim_copy(ui_.topic->text().toStdString());
    emitter << YAML::Key << "size" <<
               YAML::Value << ui_.pointSize->value();
    emitter << YAML::Key << "buffer_size" <<
               YAML::Value << ui_.bufferSize->value();
    emitter << YAML::Key << "alpha" <<
               YAML::Value << alpha_;
    emitter << YAML::Key << "color_transformer" <<
               YAML::Value << ui_.color_transformer->currentText().toStdString();
    emitter << YAML::Key << "min_color" <<
               YAML::Value << min_color_.name().toStdString();
    emitter << YAML::Key << "max_color" <<
               YAML::Value << max_color_.name().toStdString();
    emitter << YAML::Key << "value_min" <<
               YAML::Value << ui_.minValue->text().toDouble();
    emitter << YAML::Key << "value_max" <<
               YAML::Value << ui_.maxValue->text().toDouble();
    emitter << YAML::Key << "use_rainbow" <<
               YAML::Value << ui_.use_rainbow->isChecked();
  }
}

