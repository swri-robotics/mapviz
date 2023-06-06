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

#include <mapviz_plugins/plan_route_plugin.h>

// QT libraries
#include <QDateTime>
#include <QDialog>
#include <QGLWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QPalette>
#include <QStaticText>

// ROS libraries
#include <rclcpp/rclcpp.hpp>

#include <swri_route_util/util.h>
#include <swri_transform_util/frames.h>

#include <marti_nav_msgs/srv/plan_route.hpp>

// Declare plugin
#include <pluginlib/class_list_macros.hpp>

// C++ standard libraries
#include <chrono>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::PlanRoutePlugin, mapviz::MapvizPlugin)

using namespace std::chrono_literals;

namespace mnm = marti_nav_msgs;
namespace sru = swri_route_util;
namespace stu = swri_transform_util;

namespace mapviz_plugins
{
  PlanRoutePlugin::PlanRoutePlugin()
  : MapvizPlugin()
  , ui_()
  , config_widget_(new QWidget())
  , map_canvas_(nullptr)
  , failed_service_(false)
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
    QObject::connect(ui_.service, SIGNAL(editingFinished()), this,
                     SLOT(PlanRoute()));
    QObject::connect(ui_.publish, SIGNAL(clicked()), this,
                     SLOT(PublishRoute()));
    QObject::connect(ui_.clear, SIGNAL(clicked()), this,
                     SLOT(Clear()));
    QObject::connect(this,
                     SIGNAL(VisibleChanged(bool)),
                     this,
                     SLOT(VisibilityChanged(bool)));
  }

  PlanRoutePlugin::~PlanRoutePlugin()
  {
    if (map_canvas_)
    {
      map_canvas_->removeEventFilter(this);
    }
  }

  void PlanRoutePlugin::VisibilityChanged(bool visible)
  {
    if (visible)
    {
      map_canvas_->installEventFilter(this);
    }
    else
    {
      map_canvas_->removeEventFilter(this);
    }
  }

  void PlanRoutePlugin::PublishRoute()
  {
    if (route_preview_)
    {
      if (route_topic_ != ui_.topic->text().toStdString())
      {
        route_topic_ = ui_.topic->text().toStdString();
        route_pub_.reset();
        route_pub_ = node_->create_publisher<marti_nav_msgs::msg::Route>(
          route_topic_,
          rclcpp::QoS(1));
      }

      route_pub_->publish(*route_preview_->toMsgPtr());
    }
  }

  void PlanRoutePlugin::PlanRoute()
  {
    route_preview_ = sru::RoutePtr();
    bool start_from_vehicle = ui_.start_from_vehicle->isChecked();
    if (waypoints_.size() + start_from_vehicle < 2 || !Visible())
    {
      return;
    }

    std::string service = ui_.service->text().toStdString();
    if (service.empty())
    {
      PrintError("Service name may not be empty.");
      return;
    }
    auto client = node_->create_client<marti_nav_msgs::srv::PlanRoute>(service);
    client->wait_for_service(1ms);

    if (!client->service_is_ready())
    {
      PrintError("Service is unavailable.");
      return;
    }

    auto plan_route = std::make_shared<marti_nav_msgs::srv::PlanRoute::Request>();

    plan_route->header.frame_id = swri_transform_util::_wgs84_frame;
    plan_route->header.stamp = node_->now();
    plan_route->plan_from_vehicle = static_cast<unsigned char>(start_from_vehicle);
    plan_route->waypoints = waypoints_;

    PrintInfo("Sending route...");
    auto result = client->async_send_request(plan_route,
        std::bind(&PlanRoutePlugin::ClientCallback, this, std::placeholders::_1));
  }

  void PlanRoutePlugin::ClientCallback(
    rclcpp::Client<marti_nav_msgs::srv::PlanRoute>::SharedFuture future)
  {
    RCLCPP_ERROR(node_->get_logger(), "Request callback happened");
    const auto& result = future.get();
    if (future.valid())
    {
      if (result->success)
      {
        PrintInfo("OK");
        route_preview_ = std::make_shared<swri_route_util::Route>(result->route);
        failed_service_ = false;
      } else {
        PrintError(result->message);
        failed_service_ = true;
      }
    } else {
      PrintError("Error calling PlanRoute service");
      failed_service_ = true;
    }
  }

  void PlanRoutePlugin::Retry()
  {
    PlanRoute();
  }

  void PlanRoutePlugin::Clear()
  {
    waypoints_.clear();
    route_preview_ = sru::RoutePtr();
  }

  void PlanRoutePlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message, 1.0);
  }

  void PlanRoutePlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message, 1.0);
  }

  void PlanRoutePlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message, 1.0);
  }

  QWidget* PlanRoutePlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool PlanRoutePlugin::Initialize(QGLWidget* canvas)
  {
    map_canvas_ = dynamic_cast<mapviz::MapCanvas*>(canvas);
    map_canvas_->installEventFilter(this);

    retry_timer_ = node_->create_wall_timer(1000ms, [this](){Retry();});

    initialized_ = true;
    return true;
  }

  bool PlanRoutePlugin::eventFilter(QObject *object, QEvent* event)
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

  bool PlanRoutePlugin::handleMousePress(QMouseEvent* event)
  {
    selected_point_ = -1;
    int closest_point = 0;
    double closest_distance = std::numeric_limits<double>::max();

    QPointF point = event->localPos();
    stu::Transform transform;
    if (tf_manager_->GetTransform(target_frame_, stu::_wgs84_frame, transform))
    {
      for (size_t i = 0; i < waypoints_.size(); i++)
      {
        tf2::Vector3 waypoint(
            waypoints_[i].position.x,
            waypoints_[i].position.y,
            0.0);
        waypoint = transform * waypoint;

        QPointF transformed =
        map_canvas_->FixedFrameToMapGlCoord(QPointF(waypoint.x(), waypoint.y()));

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
        waypoints_.erase(waypoints_.begin() + closest_point);
        PlanRoute();
        return true;
      }
    }

    return false;
  }

  bool PlanRoutePlugin::handleMouseRelease(QMouseEvent* event)
  {
    QPointF point = event->localPos();
    if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < waypoints_.size())
    {
      stu::Transform transform;
      if (tf_manager_->GetTransform(stu::_wgs84_frame, target_frame_, transform))
      {
        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
        tf2::Vector3 position(transformed.x(), transformed.y(), 0.0);
        position = transform * position;
        waypoints_[selected_point_].position.x = position.x();
        waypoints_[selected_point_].position.y = position.y();
        PlanRoute();
      }

      selected_point_ = -1;
      return true;
    } else if (is_mouse_down_) {
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
        if (tf_manager_->GetTransform(stu::_wgs84_frame, target_frame_, transform))
        {
          position = transform * position;

          geometry_msgs::msg::Pose pose;
          pose.position.x = position.x();
          pose.position.y = position.y();
          waypoints_.push_back(pose);
          PlanRoute();
        }
      }
    }
    is_mouse_down_ = false;

    return false;
  }

  bool PlanRoutePlugin::handleMouseMove(QMouseEvent* event)
  {
    if (selected_point_ >= 0 && static_cast<size_t>(selected_point_) < waypoints_.size())
    {
      QPointF point = event->localPos();
      stu::Transform transform;
      if (tf_manager_->GetTransform(stu::_wgs84_frame, target_frame_, transform))
      {
        QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
        tf2::Vector3 position(transformed.x(), transformed.y(), 0.0);
        position = transform * position;
        waypoints_[selected_point_].position.y = position.y();
        waypoints_[selected_point_].position.x = position.x();
        PlanRoute();
      }

      return true;
    }
    return false;
  }

  void PlanRoutePlugin::Draw(double x, double y, double scale)
  {
    stu::Transform transform;
    if (tf_manager_->GetTransform(target_frame_, stu::_wgs84_frame, transform))
    {
      if (!failed_service_)
      {
        if (route_preview_)
        {
          sru::Route route = *route_preview_;
          sru::transform(route, transform, target_frame_);

          glLineWidth(2);
          const QColor color = ui_.color->color();
          glColor4d(color.redF(), color.greenF(), color.blueF(), 1.0);
          glBegin(GL_LINE_STRIP);

          for (auto & point : route.points)
          {
            glVertex2d(point.position().x(), point.position().y());
          }

          glEnd();
        }
      }

      // Draw waypoints

      glPointSize(20);
      glColor4f(0.0, 1.0, 1.0, 1.0);
      glBegin(GL_POINTS);

      for (auto & waypoint : waypoints_)
      {
        tf2::Vector3 point(waypoint.position.x, waypoint.position.y, 0);
        point = transform * point;
        glVertex2d(point.x(), point.y());
      }
      glEnd();
    } else {
      PrintError("Failed to transform.");
    }
  }

  void PlanRoutePlugin::Paint(QPainter* painter, double x, double y, double scale)
  {
    painter->save();
    painter->resetTransform();

    QPen pen(QBrush(QColor(Qt::darkCyan).darker()), 1);
    painter->setPen(pen);
    painter->setFont(QFont("DejaVu Sans Mono", 7));

    stu::Transform transform;
    if (tf_manager_->GetTransform(target_frame_, stu::_wgs84_frame, transform))
    {
      for (size_t i = 0; i < waypoints_.size(); i++)
      {
        tf2::Vector3 point(waypoints_[i].position.x, waypoints_[i].position.y, 0);
        point = transform * point;
        QPointF gl_point = map_canvas_->FixedFrameToMapGlCoord(QPointF(point.x(), point.y()));
        QPointF corner(gl_point.x() - 20, gl_point.y() - 20);
        QRectF rect(corner, QSizeF(40, 40));
        painter->drawText(
          rect,
          Qt::AlignHCenter | Qt::AlignVCenter,
          QString::fromStdString(std::to_string(i + 1)));
      }
    }

    painter->restore();
  }

  void PlanRoutePlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["route_topic"])
    {
      std::string route_topic = node["route_topic"].as<std::string>();
      ui_.topic->setText(route_topic.c_str());
    }
    if (node["color"])
    {
      std::string color = node["color"].as<std::string>();
      ui_.color->setColor(QColor(color.c_str()));
    }
    if (node["service"])
    {
      std::string service = node["service"].as<std::string>();
      ui_.service->setText(service.c_str());
    }
    if (node["start_from_vehicle"])
    {
      bool start_from_vehicle = node["start_from_vehicle"].as<bool>();
      ui_.start_from_vehicle->setChecked(start_from_vehicle);
    }

    PlanRoute();
  }

  void PlanRoutePlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string route_topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "route_topic" << YAML::Value << route_topic;

    std::string color = ui_.color->color().name().toStdString();
    emitter << YAML::Key << "color" << YAML::Value << color;

    std::string service = ui_.service->text().toStdString();
    emitter << YAML::Key << "service" << YAML::Value << service;

    bool start_from_vehicle = ui_.start_from_vehicle->isChecked();
    emitter << YAML::Key << "start_from_vehicle" << YAML::Value << start_from_vehicle;
  }
}   // namespace mapviz_plugins
