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

#include <mapviz_plugins/draw_marker_plugin.hpp>

// QT libraries
#include <QDateTime>
#include <QDialog>
#include <QOpenGLWidget>
#include <QMouseEvent>
#include <QPalette>

#include <visualization_msgs/msg/marker.hpp>
#include <mapviz/select_frame_dialog.hpp>
#include <mapviz/qt_mouse_event_compat.hpp>

// Declare plugin
#include <pluginlib/class_list_macros.hpp>

// C++ standard libraries
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::DrawMarkerPlugin, mapviz::MapvizPlugin)

namespace stu = swri_transform_util;

namespace mapviz_plugins
{
  DrawMarkerPlugin::DrawMarkerPlugin()
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
                     SLOT(PublishMarker()));
    QObject::connect(ui_.clear, SIGNAL(clicked()), this,
                     SLOT(Clear()));
  }

  DrawMarkerPlugin::~DrawMarkerPlugin()
  {
    if (map_canvas_)
    {
      map_canvas_->removeEventFilter(this);
    }
  }

  int32_t DrawMarkerPlugin::SelectedMarkerType() const
  {
    switch (ui_.marker_type->currentIndex())
    {
      case SHAPE_POINTS:
        return visualization_msgs::msg::Marker::POINTS;
      case SHAPE_LINE_STRIP:
      case SHAPE_CLOSED_POLYGON:
        return visualization_msgs::msg::Marker::LINE_STRIP;
      case SHAPE_SPHERES:
        return visualization_msgs::msg::Marker::SPHERE_LIST;
      case SHAPE_CUBES:
        return visualization_msgs::msg::Marker::CUBE_LIST;
      default:
        return visualization_msgs::msg::Marker::POINTS;
    }
  }

  bool DrawMarkerPlugin::SelectedShapeIsLine() const
  {
    const int index = ui_.marker_type->currentIndex();
    return index == SHAPE_LINE_STRIP || index == SHAPE_CLOSED_POLYGON;
  }

  void DrawMarkerPlugin::SelectFrame()
  {
    std::string frame = mapviz::SelectFrameDialog::selectFrame(tf_buf_);
    if (!frame.empty())
    {
      ui_.frame->setText(QString::fromStdString(frame));
      FrameEdited();
    }
  }

  void DrawMarkerPlugin::FrameEdited()
  {
    source_frame_ = ui_.frame->text().toStdString();
    PrintWarning("Waiting for transform.");

    RCLCPP_INFO(Logger(), "Setting target frame to %s", source_frame_.c_str());

    initialized_ = true;
  }

  void DrawMarkerPlugin::PublishMarker()
  {
    if (marker_topic_ != ui_.topic->text().toStdString())
    {
      marker_topic_ = ui_.topic->text().toStdString();
      rclcpp::QoS qos = rclcpp::QoS(1).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
      marker_pub_ = Publisher<visualization_msgs::msg::Marker>(marker_topic_, qos);
    }

    visualization_msgs::msg::Marker::UniquePtr marker =
        std::make_unique<visualization_msgs::msg::Marker>();
    marker->header.stamp = Clock()->now();
    marker->header.frame_id = ui_.frame->text().toStdString();
    marker->ns = ui_.marker_ns->text().toStdString();
    marker->id = ui_.marker_id->value();
    marker->type = SelectedMarkerType();

    if (vertices_.empty())
    {
      // Publishing an ADD with no points leaves whatever was published before
      // on screen, so clearing the plugin and publishing removes the marker.
      marker->action = visualization_msgs::msg::Marker::DELETE;
      marker_pub_->publish(*marker);
      PrintInfo("Published DELETE for " + marker->ns + "/" + std::to_string(marker->id));
      return;
    }

    marker->action = visualization_msgs::msg::Marker::ADD;
    // An all-zero quaternion is invalid and consumers warn about it, so set an
    // explicit identity even though the points carry the geometry.
    marker->pose.orientation.w = 1.0;

    const double scale = ui_.scale->value();
    marker->scale.x = scale;
    // For a line strip only scale.x is read, as the line width.  For the point
    // and list types every axis is the marker size.
    marker->scale.y = scale;
    marker->scale.z = scale;

    const QColor color = ui_.color->color();
    marker->color.r = static_cast<float>(color.redF());
    marker->color.g = static_cast<float>(color.greenF());
    marker->color.b = static_cast<float>(color.blueF());
    marker->color.a = static_cast<float>(ui_.alpha->value());

    for (const auto& vertex : vertices_)
    {
      geometry_msgs::msg::Point point;
      point.x = vertex.x();
      point.y = vertex.y();
      point.z = 0.0;
      marker->points.push_back(point);
    }

    // A closed polygon is a line strip that returns to where it started.  The
    // first point is copied out before the push_back rather than passed as a
    // reference into the vector being grown.
    if (ui_.marker_type->currentIndex() == SHAPE_CLOSED_POLYGON && marker->points.size() > 2)
    {
      const geometry_msgs::msg::Point first = marker->points.front();
      marker->points.push_back(first);
    }

    marker_pub_->publish(*marker);
    PrintInfo("Published " + std::to_string(marker->points.size()) + " points");
  }

  void DrawMarkerPlugin::Clear()
  {
    vertices_.clear();
    transformed_vertices_.clear();
  }

  void DrawMarkerPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message, 1.0);
  }

  void DrawMarkerPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message, 1.0);
  }

  void DrawMarkerPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message, 1.0);
  }

  QWidget* DrawMarkerPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool DrawMarkerPlugin::Initialize(QOpenGLWidget* canvas)
  {
    map_canvas_ = dynamic_cast<mapviz::MapCanvas*>(canvas);
    map_canvas_->installEventFilter(this);
    canvas->makeCurrent();
    initializeOpenGLFunctions();
    canvas->doneCurrent();

    initialized_ = true;
    return true;
  }

  bool DrawMarkerPlugin::eventFilter(QObject* /*object*/, QEvent* event)
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

  bool DrawMarkerPlugin::handleMousePress(QMouseEvent* event)
  {
    if (!this->Visible())
    {
      RCLCPP_DEBUG(Logger(), "Ignoring mouse press, since draw marker plugin is hidden");
      return false;
    }

    selected_point_ = -1;
    int closest_point = 0;
    double closest_distance = std::numeric_limits<double>::max();

    QPointF point = mapviz::MouseEventPosition(event);
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
        mouse_down_pos_ = mapviz::MouseEventPosition(event);
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

  bool DrawMarkerPlugin::handleMouseRelease(QMouseEvent* event)
  {
    std::string frame = ui_.frame->text().toStdString();
    if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < vertices_.size())
    {
      QPointF point = mapviz::MouseEventPosition(event);
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
      const QPointF point = mapviz::MouseEventPosition(event);
      qreal distance = QLineF(mouse_down_pos_, point).length();
      qint64 msecsDiff = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

      // Only fire the event if the mouse has moved less than the maximum distance
      // and was held for shorter than the maximum time..  This prevents click
      // events from being fired if the user is dragging the mouse across the map
      // or just holding the cursor in place.
      if (msecsDiff < max_ms_ && distance <= max_distance_)
      {
        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);

        stu::Transform transform;
        tf2::Vector3 position(transformed.x(), transformed.y(), 0.0);

        if (tf_manager_->GetTransform(frame, target_frame_, transform))
        {
          position = transform * position;
          vertices_.push_back(position);
          transformed_vertices_.resize(vertices_.size());
          RCLCPP_INFO(
            Logger(),
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

  bool DrawMarkerPlugin::handleMouseMove(QMouseEvent* event)
  {
    if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < vertices_.size())
    {
      QPointF point = mapviz::MouseEventPosition(event);
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

  void DrawMarkerPlugin::Draw(double /*x*/, double /*y*/, double /*scale*/)
  {
    stu::Transform transform;
    std::string frame = ui_.frame->text().toStdString();
    if (!tf_manager_->GetTransform(target_frame_, frame, transform))
    {
      PrintError("No transform between " + frame + " and " + target_frame_);
      return;
    }

    for (size_t i = 0; i < vertices_.size(); i++)
    {
      transformed_vertices_[i] = transform * vertices_[i];
    }

    const QColor color = ui_.color->color();
    const double alpha = ui_.alpha->value();

    // Preview the shape the way it will be published, so that switching the
    // type shows what the marker will look like before anything is sent.
    if (SelectedShapeIsLine() && transformed_vertices_.size() > 1)
    {
      glLineWidth(2);
      glColor4d(color.redF(), color.greenF(), color.blueF(), alpha);
      glBegin(GL_LINE_STRIP);
      for (const auto& vertex : transformed_vertices_)
      {
        glVertex2d(vertex.x(), vertex.y());
      }
      if (ui_.marker_type->currentIndex() == SHAPE_CLOSED_POLYGON &&
          transformed_vertices_.size() > 2)
      {
        glVertex2d(transformed_vertices_.front().x(), transformed_vertices_.front().y());
      }
      glEnd();
    }

    // The vertices are always drawn so that they stay grabbable regardless of
    // which shape is selected.
    glPointSize(9);
    glColor4d(color.redF(), color.greenF(), color.blueF(), alpha);
    glBegin(GL_POINTS);
    for (const auto& vertex : transformed_vertices_)
    {
      glVertex2d(vertex.x(), vertex.y());
    }
    glEnd();

    PrintInfo("OK");
  }

  void DrawMarkerPlugin::LoadConfig(const YAML::Node& node, const std::string& /*path*/)
  {
    if (node["frame"])
    {
      source_frame_ = node["frame"].as<std::string>();
      ui_.frame->setText(source_frame_.c_str());
    }

    if (node["topic"])
    {
      ui_.topic->setText(node["topic"].as<std::string>().c_str());
    }

    if (node["type"])
    {
      int type = node["type"].as<int>();
      if (type >= 0 && type < ui_.marker_type->count())
      {
        ui_.marker_type->setCurrentIndex(type);
      }
    }

    if (node["namespace"])
    {
      ui_.marker_ns->setText(node["namespace"].as<std::string>().c_str());
    }

    if (node["id"])
    {
      ui_.marker_id->setValue(node["id"].as<int>());
    }

    if (node["color"])
    {
      ui_.color->setColor(QColor(node["color"].as<std::string>().c_str()));
    }

    if (node["alpha"])
    {
      ui_.alpha->setValue(node["alpha"].as<double>());
    }

    if (node["scale"])
    {
      ui_.scale->setValue(node["scale"].as<double>());
    }

    // Restoring the vertices is what makes a drawing outlive the session it was
    // made in; without it the plugin comes back configured but empty.
    if (node["vertices"])
    {
      vertices_.clear();
      for (const auto& vertex : node["vertices"])
      {
        if (vertex.size() >= 2)
        {
          vertices_.emplace_back(vertex[0].as<double>(), vertex[1].as<double>(), 0.0);
        }
      }
      transformed_vertices_.resize(vertices_.size());
    }
  }

  void DrawMarkerPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& /*path*/)
  {
    emitter << YAML::Key << "frame" << YAML::Value << ui_.frame->text().toStdString();
    emitter << YAML::Key << "topic" << YAML::Value << ui_.topic->text().toStdString();
    emitter << YAML::Key << "type" << YAML::Value << ui_.marker_type->currentIndex();
    emitter << YAML::Key << "namespace" << YAML::Value << ui_.marker_ns->text().toStdString();
    emitter << YAML::Key << "id" << YAML::Value << ui_.marker_id->value();
    emitter << YAML::Key << "color" << YAML::Value << ui_.color->color().name().toStdString();
    emitter << YAML::Key << "alpha" << YAML::Value << ui_.alpha->value();
    emitter << YAML::Key << "scale" << YAML::Value << ui_.scale->value();

    emitter << YAML::Key << "vertices" << YAML::Value << YAML::BeginSeq;
    for (const auto& vertex : vertices_)
    {
      emitter << YAML::Flow << YAML::BeginSeq << vertex.x() << vertex.y() << YAML::EndSeq;
    }
    emitter << YAML::EndSeq;
  }
}   // namespace mapviz_plugins
