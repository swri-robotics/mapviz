// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
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

#include <mapviz_plugins/speedometer_plugin.hpp>
#include <mapviz_plugins/topic_select.hpp>

// QT libraries
#include <QFontMetrics>
#include <QPainter>
#include <QPalette>
#include <QPen>

// ROS libraries
#include <rclcpp/rclcpp.hpp>

// Declare plugin
#include <pluginlib/class_list_macros.hpp>

// C++ standard libraries
#include <algorithm>
#include <cmath>
#include <string>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::SpeedometerPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  namespace
  {
    /// The dial sweeps from lower left to lower right, the way a vehicle
    /// speedometer does.  Angles are measured the way QPainter::drawArc wants
    /// them: degrees counter-clockwise from three o'clock.
    constexpr int DIAL_START_ANGLE = 210;
    constexpr int DIAL_SWEEP_ANGLE = -240;

    /// Total tick marks, including both ends of the scale.  Only every other
    /// one is labelled; labelling all nine crowds the numbers together at the
    /// top of the dial where the arc is tightest.
    constexpr int TICK_COUNT = 9;
  }  // namespace

  SpeedometerPlugin::SpeedometerPlugin()
  : MapvizPlugin()
  , ui_()
  , config_widget_(new QWidget())
  , topic_("")
  , qos_(rmw_qos_profile_default)
  , speed_(0.0)
  , has_message_(false)
  , max_speed_(40.0)
  {
    ui_.setupUi(config_widget_);

    ui_.color->setColor(Qt::green);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Window, Qt::white);
    config_widget_->setPalette(p);

    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    max_speed_ = ui_.max_speed->value();

    placer_.setRect(QRect(0, 0, 200, 200));
    QObject::connect(this, SIGNAL(VisibleChanged(bool)),
                     &placer_, SLOT(setVisible(bool)));

    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
    QObject::connect(ui_.max_speed, SIGNAL(valueChanged(double)),
                     this, SLOT(MaxSpeedChanged(double)));
  }

  bool SpeedometerPlugin::Initialize(QOpenGLWidget* canvas)
  {
    canvas_ = canvas;
    placer_.setContainer(canvas_);
    initialized_ = true;
    return true;
  }

  void SpeedometerPlugin::Shutdown()
  {
    placer_.setContainer(nullptr);
  }

  QWidget* SpeedometerPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);
    return config_widget_;
  }

  void SpeedometerPlugin::SelectTopic()
  {
    auto [topic, qos] = SelectTopicDialog::selectTopic(
      TopicSource(),
      "nav_msgs/msg/Odometry",
      qos_);

    if (!topic.empty())
    {
      connectCallback(topic, qos);
    }
  }

  void SpeedometerPlugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();
    connectCallback(topic, qos_);
  }

  void SpeedometerPlugin::MaxSpeedChanged(double value)
  {
    max_speed_ = value;
    if (canvas_)
    {
      canvas_->update();
    }
  }

  void SpeedometerPlugin::connectCallback(
    const std::string& topic, const rmw_qos_profile_t& qos)
  {
    ui_.topic->setText(QString::fromStdString(topic));
    if ((topic != topic_) || !qosEqual(qos, qos_))
    {
      has_message_ = false;
      speed_ = 0.0;
      PrintWarning("No messages received.");

      odom_sub_.reset();

      topic_ = topic;
      qos_ = qos;
      if (!topic_.empty())
      {
        // Subscribe() delivers each message to handleOdometry() on the GUI
        // thread, where plugin state may be touched without locking.
        Subscribe<nav_msgs::msg::Odometry>(
          topic_, qos, odom_sub_,
          [this](nav_msgs::msg::Odometry::ConstSharedPtr odometry) {
            handleOdometry(odometry);
          });
        RCLCPP_INFO(Logger(), "Subscribing to %s", topic_.c_str());
      }
    }
  }

  void SpeedometerPlugin::handleOdometry(
    const nav_msgs::msg::Odometry::ConstSharedPtr odometry)
  {
    // Odometry twists are body relative.  Some sources put the whole speed in
    // x, others split it across the axes, so use the magnitude of the vector
    // rather than assuming a convention.
    const double vx = odometry->twist.twist.linear.x;
    const double vy = odometry->twist.twist.linear.y;
    const double vz = odometry->twist.twist.linear.z;
    speed_ = std::sqrt(vx * vx + vy * vy + vz * vz);

    has_message_ = true;
    initialized_ = true;

    if (canvas_)
    {
      canvas_->update();
    }
  }

  void SpeedometerPlugin::PaintDial(QPainter* painter, const QRectF& rect)
  {
    // Keep the dial circular inside whatever rectangle the placer gives us.
    const double diameter = std::min(rect.width(), rect.height());
    const QPointF center(rect.center());
    const double radius = diameter / 2.0;
    if (radius <= 1.0)
    {
      return;
    }

    const QRectF dial(center.x() - radius, center.y() - radius, diameter, diameter);
    const QColor color = ui_.color->color();

    // Backing disc, so the dial stays readable over aerial imagery.
    painter->setPen(Qt::NoPen);
    painter->setBrush(QColor(0, 0, 0, 140));
    painter->drawEllipse(dial);

    // The arc the needle sweeps.  drawArc() counts in sixteenths of a degree.
    QPen arc_pen(color, std::max(1.0, radius * 0.04));
    arc_pen.setCapStyle(Qt::FlatCap);
    painter->setPen(arc_pen);
    painter->setBrush(Qt::NoBrush);
    const QRectF arc_rect = dial.adjusted(radius * 0.12, radius * 0.12,
                                          -radius * 0.12, -radius * 0.12);
    painter->drawArc(arc_rect, DIAL_START_ANGLE * 16, DIAL_SWEEP_ANGLE * 16);

    // Ticks and their labels.
    QFont font = painter->font();
    font.setPointSizeF(std::max(5.0, radius * 0.13));
    painter->setFont(font);
    const QFontMetrics metrics(font);

    for (int i = 0; i < TICK_COUNT; i++)
    {
      const bool is_major = (i % 2) == 0;
      const double fraction = static_cast<double>(i) / (TICK_COUNT - 1);
      const double degrees = DIAL_START_ANGLE + fraction * DIAL_SWEEP_ANGLE;
      const double radians = degrees * M_PI / 180.0;
      // Screen y grows downward while the arc angles grow counter-clockwise,
      // so the y term is negated here and for the needle below.
      const double dx = std::cos(radians);
      const double dy = -std::sin(radians);

      const double outer = radius * 0.88;
      const double inner = radius * (is_major ? 0.72 : 0.80);
      painter->setPen(QPen(color, std::max(1.0, radius * (is_major ? 0.025 : 0.015))));
      painter->drawLine(QPointF(center.x() + dx * inner, center.y() + dy * inner),
                        QPointF(center.x() + dx * outer, center.y() + dy * outer));

      if (!is_major)
      {
        continue;
      }

      const double label_radius = radius * 0.56;
      const QString label = QString::number(fraction * max_speed_, 'g', 3);
      const QRectF label_rect(
        center.x() + dx * label_radius - metrics.horizontalAdvance(label) / 2.0,
        center.y() + dy * label_radius - metrics.height() / 2.0,
        metrics.horizontalAdvance(label),
        metrics.height());
      painter->setPen(QPen(Qt::white));
      painter->drawText(label_rect, Qt::AlignCenter, label);
    }

    // Needle.  A speed beyond the configured maximum pegs it at the end of the
    // scale rather than swinging past the dial.
    const double clamped = max_speed_ > 0.0
      ? std::min(std::max(speed_, 0.0), max_speed_)
      : 0.0;
    const double needle_fraction = max_speed_ > 0.0 ? clamped / max_speed_ : 0.0;
    const double needle_degrees = DIAL_START_ANGLE + needle_fraction * DIAL_SWEEP_ANGLE;
    const double needle_radians = needle_degrees * M_PI / 180.0;

    painter->setPen(QPen(color, std::max(1.5, radius * 0.05), Qt::SolidLine, Qt::RoundCap));
    painter->drawLine(center,
                      QPointF(center.x() + std::cos(needle_radians) * radius * 0.7,
                              center.y() - std::sin(needle_radians) * radius * 0.7));

    painter->setPen(Qt::NoPen);
    painter->setBrush(color);
    painter->drawEllipse(center, radius * 0.07, radius * 0.07);

    // Numeric readout.  This shows the real speed even when the needle is
    // pegged, so an over-range value is still legible.
    QFont readout_font = painter->font();
    readout_font.setPointSizeF(std::max(6.0, radius * 0.2));
    readout_font.setBold(true);
    painter->setFont(readout_font);
    painter->setPen(QPen(Qt::white));

    const QString readout = has_message_
      ? QString::number(speed_, 'f', 2) + " m/s"
      : QString("--");
    const QRectF readout_rect(dial.left(), center.y() + radius * 0.35,
                              diameter, radius * 0.3);
    painter->drawText(readout_rect, Qt::AlignCenter, readout);
  }

  void SpeedometerPlugin::Paint(QPainter* painter, double, double, double)
  {
    painter->save();
    painter->resetTransform();
    painter->setRenderHint(QPainter::Antialiasing, true);

    PaintDial(painter, placer_.rect());

    painter->restore();

    if (has_message_)
    {
      PrintInfo("OK");
    }
    else
    {
      PrintWarning("No messages received.");
    }
  }

  void SpeedometerPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void SpeedometerPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void SpeedometerPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  void SpeedometerPlugin::LoadConfig(const YAML::Node& node, const std::string& /*path*/)
  {
    LoadQosConfig(node, qos_);

    if (node["topic"])
    {
      ui_.topic->setText(node["topic"].as<std::string>().c_str());
    }

    if (node["max_speed"])
    {
      ui_.max_speed->setValue(node["max_speed"].as<double>());
    }

    if (node["color"])
    {
      ui_.color->setColor(QColor(node["color"].as<std::string>().c_str()));
    }

    QRect current = placer_.rect();
    int x = current.x();
    int y = current.y();
    int width = current.width();
    int height = current.height();

    if (node["x"])
    {
      x = node["x"].as<int>();
    }

    if (node["y"])
    {
      y = node["y"].as<int>();
    }

    if (node["width"])
    {
      width = node["width"].as<int>();
    }

    if (node["height"])
    {
      height = node["height"].as<int>();
    }

    placer_.setRect(QRect(x, y, width, height));

    TopicEdited();
  }

  void SpeedometerPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& /*path*/)
  {
    emitter << YAML::Key << "topic" << YAML::Value << ui_.topic->text().toStdString();
    emitter << YAML::Key << "max_speed" << YAML::Value << ui_.max_speed->value();
    emitter << YAML::Key << "color" << YAML::Value << ui_.color->color().name().toStdString();

    const QRect rect = placer_.rect();
    emitter << YAML::Key << "x" << YAML::Value << rect.x();
    emitter << YAML::Key << "y" << YAML::Value << rect.y();
    emitter << YAML::Key << "width" << YAML::Value << rect.width();
    emitter << YAML::Key << "height" << YAML::Value << rect.height();

    SaveQosConfig(emitter, qos_);
  }
}   // namespace mapviz_plugins
