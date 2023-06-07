// *****************************************************************************
//
// Copyright (c) 2016, Southwest Research Institute® (SwRI®)
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

#include <mapviz_plugins/draw_polygon_plugin.h>

// QT libraries
#include <QDateTime>
#include <QDialog>
#include <QGLWidget>
#include <QMouseEvent>
#include <QPalette>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <mapviz/select_frame_dialog.h>

// Declare plugin
#include <pluginlib/class_list_macros.hpp>

// C++ standard libraries
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::DrawPolygonPlugin, mapviz::MapvizPlugin)

namespace stu = swri_transform_util;

namespace mapviz_plugins
{
  DrawPolygonPlugin::DrawPolygonPlugin()
  : MapvizPlugin()
  , ui_()
  , config_widget_(new QWidget())
  , map_canvas_(nullptr)
  , selected_point_(-1)
  , is_mouse_down_(false)
  , mouse_down_time_(0)
  , max_ms_(Q_INT64_C(500))
  , max_distance_(2.0)
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

    QObject::connect(ui_.selectframe, SIGNAL(clicked()), this,
                     SLOT(SelectFrame()));
    QObject::connect(ui_.frame, SIGNAL(editingFinished()), this,
                     SLOT(FrameEdited()));
    QObject::connect(ui_.publish, SIGNAL(clicked()), this,
                     SLOT(PublishPolygon()));
    QObject::connect(ui_.clear, SIGNAL(clicked()), this,
                     SLOT(Clear()));
  }

  DrawPolygonPlugin::~DrawPolygonPlugin()
  {
    if (map_canvas_)
    {
      map_canvas_->removeEventFilter(this);
    }
  }

  void DrawPolygonPlugin::SelectFrame()
  {
    std::string frame = mapviz::SelectFrameDialog::selectFrame(tf_buf_);
    if (!frame.empty())
    {
      ui_.frame->setText(QString::fromStdString(frame));
      FrameEdited();
    }
  }

  void DrawPolygonPlugin::FrameEdited()
  {
    source_frame_ = ui_.frame->text().toStdString();
    PrintWarning("Waiting for transform.");

    RCLCPP_INFO(node_->get_logger(), "Setting target frame to to %s", source_frame_.c_str());

    initialized_ = true;
  }

  void DrawPolygonPlugin::PublishPolygon()
  {
    if (polygon_topic_ != ui_.topic->text().toStdString())
    {
      polygon_topic_ = ui_.topic->text().toStdString();
      rclcpp::QoS qos = rclcpp::QoS(1).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      polygon_pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
          polygon_topic_, qos);
    }

    geometry_msgs::msg::PolygonStamped::UniquePtr polygon =
        std::make_unique<geometry_msgs::msg::PolygonStamped>();
    polygon->header.stamp = node_->get_clock()->now();
    polygon->header.frame_id = ui_.frame->text().toStdString();

    for (const auto& vertex : vertices_)
    {
      geometry_msgs::msg::Point32 point;
      point.x = vertex.x();
      point.y = vertex.y();
      point.z = 0;
      polygon->polygon.points.push_back(point);
    }

    polygon_pub_->publish(*polygon);
  }

  void DrawPolygonPlugin::Clear()
  {
    vertices_.clear();
    transformed_vertices_.clear();
  }

  void DrawPolygonPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message, 1.0);
  }

  void DrawPolygonPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message, 1.0);
  }

  void DrawPolygonPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message, 1.0);
  }

  QWidget* DrawPolygonPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool DrawPolygonPlugin::Initialize(QGLWidget* canvas)
  {
    map_canvas_ = dynamic_cast<mapviz::MapCanvas*>(canvas);
    map_canvas_->installEventFilter(this);

    initialized_ = true;
    return true;
  }

  bool DrawPolygonPlugin::eventFilter(QObject *object, QEvent* event)
  {
    switch (event->type())
    {
      case QEvent::MouseButtonPress:
        return handleMousePress(dynamic_cast<QMouseEvent*>(event));
      case QEvent::MouseButtonRelease:
        return handleMouseRelease(dynamic_cast<QMouseEvent*>(event));
      case QEvent::MouseMove:
        return handleMouseMove(dynamic_cast<QMouseEvent*>(event));
      default:
        return false;
    }
  }

  bool DrawPolygonPlugin::handleMousePress(QMouseEvent* event)
  {
    if(!this->Visible())
    {
      RCLCPP_DEBUG(node_->get_logger(), "Ignoring mouse press, since draw polygon plugin is hidden");
      return false;
    }
    
    selected_point_ = -1;
    int closest_point = 0;
    double closest_distance = std::numeric_limits<double>::max();

    QPointF point = event->localPos();
    stu::Transform transform;
    std::string frame = ui_.frame->text().toStdString();
    if (tf_manager_->GetTransform(target_frame_, frame, transform))
    {
      for (size_t i = 0; i < vertices_.size(); i++)
      {
        tf2::Vector3 vertex = vertices_[i];
        vertex = transform * vertex;

        QPointF transformed = map_canvas_->FixedFrameToMapGlCoord(QPointF(vertex.x(), vertex.y()));

        double distance = QLineF(transformed, point).length();

        if (distance < closest_distance)
        {
          closest_distance = distance;
          closest_point = static_cast<int>(i);
        }
      }
    }

    if (event->button() == Qt::LeftButton)
    {
      if (closest_distance < 15)
      {
        selected_point_ = closest_point;
        return true;
      } else {
        is_mouse_down_ = true;
        mouse_down_pos_ = event->localPos();
        mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();
        return false;
      }
    } else if (event->button() == Qt::RightButton) {
      if (closest_distance < 15)
      {
        vertices_.erase(vertices_.begin() + closest_point);
        transformed_vertices_.resize(vertices_.size());
        return true;
      }
    }

    return false;
  }

  bool DrawPolygonPlugin::handleMouseRelease(QMouseEvent* event)
  {
    std::string frame = ui_.frame->text().toStdString();
    if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < vertices_.size())
    {
      QPointF point = event->localPos();
      stu::Transform transform;
      if (tf_manager_->GetTransform(frame, target_frame_, transform))
      {
        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
        tf2::Vector3 position(transformed.x(), transformed.y(), 0.0);
        position = transform * position;
        vertices_[selected_point_].setX(position.x());
        vertices_[selected_point_].setY(position.y());
      }

      selected_point_ = -1;
      return true;
    } else if (is_mouse_down_) {
      qreal distance = QLineF(mouse_down_pos_, event->localPos()).length();
      qint64 msecsDiff = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

      // Only fire the event if the mouse has moved less than the maximum distance
      // and was held for shorter than the maximum time..  This prevents click
      // events from being fired if the user is dragging the mouse across the map
      // or just holding the cursor in place.
      if (msecsDiff < max_ms_ && distance <= max_distance_)
      {
        QPointF point = event->localPos();

        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
        RCLCPP_INFO(
          node_->get_logger(),
          "mouse point at %f, %f -> %f, %f",
          point.x(),
          point.y(),
          transformed.x(),
          transformed.y());

        stu::Transform transform;
        tf2::Vector3 position(transformed.x(), transformed.y(), 0.0);

        if (tf_manager_->GetTransform(frame, target_frame_, transform))
        {
          position = transform * position;
          vertices_.push_back(position);
          transformed_vertices_.resize(vertices_.size());
          RCLCPP_INFO(
            node_->get_logger(),
            "Adding vertex at %lf, %lf %s",
            position.x(),
            position.y(),
            frame.c_str());
        }
      }
    }
    is_mouse_down_ = false;

    return false;
  }

  bool DrawPolygonPlugin::handleMouseMove(QMouseEvent* event)
  {
    if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < vertices_.size())
    {
      QPointF point = event->localPos();
      stu::Transform transform;
      std::string frame = ui_.frame->text().toStdString();
      if (tf_manager_->GetTransform(frame, target_frame_, transform))
      {
        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
        tf2::Vector3 position(transformed.x(), transformed.y(), 0.0);
        position = transform * position;
        vertices_[selected_point_].setY(position.y());
        vertices_[selected_point_].setX(position.x());
      }

      return true;
    }
    return false;
  }

  void DrawPolygonPlugin::Draw(double x, double y, double scale)
  {
    stu::Transform transform;
    std::string frame = ui_.frame->text().toStdString();
    if (!tf_manager_->GetTransform(target_frame_, frame, transform))
    {
      return;
    }

    // Transform polygon
    for (size_t i = 0; i < vertices_.size(); i++)
    {
      transformed_vertices_[i] = transform * vertices_[i];
    }

    glLineWidth(1);
    const QColor color = ui_.color->color();
    glColor4d(color.redF(), color.greenF(), color.blueF(), 1.0);
    glBegin(GL_LINE_STRIP);

    for (const auto& vertex : transformed_vertices_)
    {
      glVertex2d(vertex.x(), vertex.y());
    }

    glEnd();

    glBegin(GL_LINES);

    glColor4d(color.redF(), color.greenF(), color.blueF(), 0.25);

    if (transformed_vertices_.size() > 2)
    {
      glVertex2d(transformed_vertices_.front().x(), transformed_vertices_.front().y());
      glVertex2d(transformed_vertices_.back().x(), transformed_vertices_.back().y());
    }

    glEnd();

    // Draw vertices
    glPointSize(9);
    glBegin(GL_POINTS);

    for (const auto& vertex : transformed_vertices_)
    {
      glVertex2d(vertex.x(), vertex.y());
    }
    glEnd();



    PrintInfo("OK");
  }

  void DrawPolygonPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["frame"])
    {
      source_frame_ = node["frame"].as<std::string>();
      ui_.frame->setText(source_frame_.c_str());
    }

    if (node["polygon_topic"])
    {
      std::string polygon_topic = node["polygon_topic"].as<std::string>();
      ui_.topic->setText(polygon_topic.c_str());
    }
    if (node["color"])
    {
      std::string color = node["color"].as<std::string>();
      ui_.color->setColor(QColor(color.c_str()));
    }
  }

  void DrawPolygonPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string frame = ui_.frame->text().toStdString();
    emitter << YAML::Key << "frame" << YAML::Value << frame;

    std::string polygon_topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "polygon_topic" << YAML::Value << polygon_topic;

    std::string color = ui_.color->color().name().toStdString();
    emitter << YAML::Key << "color" << YAML::Value << color;
  }
}   // namespace mapviz_plugins
