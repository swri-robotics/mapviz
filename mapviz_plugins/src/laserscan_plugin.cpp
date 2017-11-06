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
#include <swri_transform_util/transform.h>
#include <swri_yaml_util/yaml_util.h>

#include <mapviz/select_topic_dialog.h>

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
    ui_.min_color->setColor(Qt::white);
    ui_.max_color->setColor(Qt::black);
    
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
        SIGNAL(valueChanged(double)),
        this,
        SLOT(AlphaEdited(double)));
    QObject::connect(ui_.color_transformer,
        SIGNAL(currentIndexChanged(int)),
        this,
        SLOT(ColorTransformerChanged(int)));
    QObject::connect(ui_.max_color,
        SIGNAL(colorEdited(const QColor &)),
        this,
        SLOT(UpdateColors()));
    QObject::connect(ui_.min_color,
        SIGNAL(colorEdited(const QColor &)),
        this,
        SLOT(UpdateColors()));
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

    QObject::connect(ui_.max_color,
        SIGNAL(colorEdited(const QColor &)),
        this,
        SLOT(DrawIcon()));
    QObject::connect(ui_.min_color,
        SIGNAL(colorEdited(const QColor &)),
        this,
        SLOT(DrawIcon()));

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

      pen.setColor(ui_.min_color->color());
      painter.setPen(pen);
      painter.drawPoint(2, 13);

      pen.setColor(ui_.min_color->color());
      painter.setPen(pen);
      painter.drawPoint(4, 6);

      pen.setColor(ui_.max_color->color());
      painter.setPen(pen);
      painter.drawPoint(12, 9);

      pen.setColor(ui_.max_color->color());
      painter.setPen(pen);
      painter.drawPoint(13, 2);

      icon_->SetPixmap(icon);
    }
  }

  QColor LaserScanPlugin::CalculateColor(const StampedPoint& point,
      bool has_intensity)
  {
    double val;
    int color_transformer = ui_.color_transformer->currentIndex();
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
      val = point.transformed_point.z();
    }
    else  // No intensity or  (color_transformer == COLOR_FLAT)
    {
      return ui_.min_color->color();
    }
    if (max_value_ > min_value_)
      val = (val - min_value_) / (max_value_ - min_value_);
    val = std::max(0.0, std::min(val, 1.0));
    if (ui_.use_rainbow->isChecked())
    {
      // Hue Interpolation
      int hue = static_cast<int>(val * 255);
      return QColor::fromHsl(hue, 255, 127, 255);
    }
    else
    {
      const QColor min_color = ui_.min_color->color();
      const QColor max_color = ui_.max_color->color();
      // RGB Interpolation
      int red, green, blue;
      red = static_cast<int>(val * max_color.red()   + ((1.0 - val) * min_color.red()));
      green = static_cast<int>(val * max_color.green() + ((1.0 - val) * min_color.green()));
      blue = static_cast<int>(val * max_color.blue()  + ((1.0 - val) * min_color.blue()));
      return QColor(red, green, blue, 255);
    }
  }

  void LaserScanPlugin::UpdateColors()
  {
    std::deque<Scan>::iterator scan_it = scans_.begin();
    for (; scan_it != scans_.end(); ++scan_it)
    {
      std::vector<StampedPoint>::iterator point_it = scan_it->points.begin();
      for (; point_it != scan_it->points.end(); point_it++)
      {
        point_it->color = CalculateColor(*point_it, scan_it->has_intensity);
      }
    }
  }

  void LaserScanPlugin::SelectTopic()
  {
    ros::master::TopicInfo topic = mapviz::SelectTopicDialog::selectTopic(
      "sensor_msgs/LaserScan");

    if (!topic.name.empty())
    {
      ui_.topic->setText(QString::fromStdString(topic.name));
      TopicEdited();
    }
  }

  void LaserScanPlugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();
    if (topic != topic_)
    {
      initialized_ = false;
      scans_.clear();
      has_message_ = false;
      PrintWarning("No messages received.");

      laserscan_sub_.shutdown();

      topic_ = topic;
      if (!topic.empty())
      {
        laserscan_sub_ = node_.subscribe(topic_,
                                         100,
                                         &LaserScanPlugin::laserScanCallback,
                                         this);

        ROS_INFO("Subscribing to %s", topic_.c_str());
      }
    }
  }

  void LaserScanPlugin::MinValueChanged(double value)
  {
    min_value_ = value;
    UpdateColors();
  }

  void LaserScanPlugin::MaxValueChanged(double value)
  {
    max_value_ = value;
    UpdateColors();
  }

  void LaserScanPlugin::BufferSizeChanged(int value)
  {
    buffer_size_ = static_cast<size_t>(value);

    if (buffer_size_ > 0)
    {
      while (scans_.size() > buffer_size_)
      {
        scans_.pop_front();
      }
    }
  }

  void LaserScanPlugin::PointSizeChanged(int value)
  {
    point_size_ = static_cast<size_t>(value);
  }

  void LaserScanPlugin::updatePreComputedTriginometic(const sensor_msgs::LaserScanConstPtr& msg)
  {
      static size_t prev_size      = 0;
      static float  prev_angle_min = msg->angle_min;
      static float  prev_increment = msg->angle_increment;

      if( msg->ranges.size() != prev_size ||
          msg->angle_min !=  prev_angle_min  ||
          msg->angle_increment != prev_increment   )
      {
          prev_size = msg->ranges.size();
          prev_angle_min = msg->angle_min;
          prev_increment = msg->angle_increment;
          precomputed_cos_.resize( prev_size );
          precomputed_sin_.resize( prev_size );

          for (size_t i = 0; i < prev_size; i++)
          {
              double angle = msg->angle_min + msg->angle_increment * i;
              precomputed_cos_[i] = cos(angle);
              precomputed_sin_[i] = sin(angle);
          }
      }
  }

  void LaserScanPlugin::laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
  {
    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }

    // Note that unlike some plugins, this one does not store nor rely on the
    // source_frame_ member variable.  This one can potentially store many
    // messages with different source frames, so we need to store and transform
    // them individually.

    Scan scan;
    scan.stamp = msg->header.stamp;
    scan.color = QColor::fromRgbF(1.0f, 0.0f, 0.0f, 1.0f);
    scan.source_frame_ = msg->header.frame_id;
    scan.transformed = true;
    scan.has_intensity = !msg->intensities.empty();

    scan.points.reserve( msg->ranges.size() );

    swri_transform_util::Transform transform;
    if (!GetTransform(scan.source_frame_, msg->header.stamp, transform))
    {
      scan.transformed = false;
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }
    double x, y;

    updatePreComputedTriginometic(msg);

    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
      // Discard the point if it's out of range
      if (msg->ranges[i] > msg->range_max || msg->ranges[i] < msg->range_min)
      {
        continue;
      }
      StampedPoint point;
      x = precomputed_cos_[i] * msg->ranges[i];
      y = precomputed_sin_[i] * msg->ranges[i];
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
  }

  void LaserScanPlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
    {
      return;
    }

    ROS_ERROR("Error: %s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void LaserScanPlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
    {
      return;
    }

    ROS_INFO("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void LaserScanPlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
    {
      return;
    }

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
    glPointSize(point_size_);
    glBegin(GL_POINTS);

    std::deque<Scan>::const_iterator scan_it = scans_.begin();
    while (scan_it != scans_.end())
    {
      if (scan_it->transformed)
      {
        std::vector<StampedPoint>::const_iterator point_it = scan_it->points.begin();
        for (; point_it != scan_it->points.end(); ++point_it)
        {
          glColor4d(
              point_it->color.redF(),
              point_it->color.greenF(),
              point_it->color.blueF(),
              alpha_);
          glVertex2d(
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
      ui_.max_color->setVisible(false);
      ui_.min_color->setVisible(false);
      ui_.maxColorLabel->setVisible(false);
      ui_.minColorLabel->setVisible(false);
    }
    else
    {
      ui_.max_color->setVisible(true);
      ui_.min_color->setVisible(true);
      ui_.maxColorLabel->setVisible(true);
      ui_.minColorLabel->setVisible(true);
    }
    UpdateColors();
  }

  void LaserScanPlugin::Transform()
  {
    std::deque<Scan>::iterator scan_it = scans_.begin();
    for (; scan_it != scans_.end(); ++scan_it)
    {
      Scan& scan = *scan_it;

      swri_transform_util::Transform transform;

      bool was_using_latest_transforms = this->use_latest_transforms_;
      this->use_latest_transforms_ = false;
      if (GetTransform(scan.source_frame_, scan.stamp, transform))
      {
        scan.transformed = true;
        std::vector<StampedPoint>::iterator point_it = scan.points.begin();
        for (; point_it != scan.points.end(); ++point_it)
        {
          point_it->transformed_point = transform * point_it->point;
        }
      }
      else
      {
        scan.transformed = false;
      }
      this->use_latest_transforms_ = was_using_latest_transforms;
    }
    // Z color is based on transformed color, so it is dependent on the
    // transform
    if (ui_.color_transformer->currentIndex() == COLOR_Z)
    {
      UpdateColors();
    }
  }

  void LaserScanPlugin::LoadConfig(const YAML::Node& node,
      const std::string& path)
  {
    if (node["topic"])
    {
      std::string topic;
      node["topic"] >> topic;
      ui_.topic->setText(boost::trim_copy(topic).c_str());
      TopicEdited();
    }

    if (node["size"])
    {
      node["size"] >> point_size_;
      ui_.pointSize->setValue(static_cast<int>(point_size_));
    }

    if (node["buffer_size"])
    {
      node["buffer_size"] >> buffer_size_;
      ui_.bufferSize->setValue(static_cast<int>(buffer_size_));
    }

    if (node["color_transformer"])
    {
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
    }

    if (node["min_color"])
    {
      std::string min_color_str;
      node["min_color"] >> min_color_str;
      ui_.min_color->setColor(QColor(min_color_str.c_str()));
    }

    if (node["max_color"])
    {
      std::string max_color_str;
      node["max_color"] >> max_color_str;
      ui_.max_color->setColor(QColor(max_color_str.c_str()));
    }

    if (node["value_min"])
    {
      node["value_min"] >> min_value_;
      ui_.minValue->setValue(min_value_);
    }

    if (node["max_value"])
    {
      node["value_max"] >> max_value_;
      ui_.maxValue->setValue(max_value_);
    }

    if (node["alpha"])
    {
      node["alpha"] >> alpha_;
      ui_.alpha->setValue(alpha_);
    }

    if (node["use_rainbow"])
    {
      bool use_rainbow;
      node["use_rainbow"] >> use_rainbow;
      ui_.use_rainbow->setChecked(use_rainbow);
    }

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
        ui_.min_color->setVisible(true);
        ui_.max_color->setVisible(false);
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
        ui_.min_color->setVisible(!ui_.use_rainbow->isChecked());
        ui_.max_color->setVisible(!ui_.use_rainbow->isChecked());
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
  }

  /**
   * Coerces alpha to [0.0, 1.0] and stores it in alpha_
   */
  void LaserScanPlugin::AlphaEdited(double val)
  {
    alpha_ = std::max(0.0f, std::min((float)val, 1.0f));
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
               YAML::Value << ui_.min_color->color().name().toStdString();
    emitter << YAML::Key << "max_color" <<
               YAML::Value << ui_.max_color->color().name().toStdString();
    emitter << YAML::Key << "value_min" <<
               YAML::Value << ui_.minValue->text().toDouble();
    emitter << YAML::Key << "value_max" <<
               YAML::Value << ui_.maxValue->text().toDouble();
    emitter << YAML::Key << "use_rainbow" <<
               YAML::Value << ui_.use_rainbow->isChecked();
  }
}

