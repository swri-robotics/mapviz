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

#include <mapviz_plugins/route_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QDialog>
#include <QGLWidget>
#include <QPainter>
#include <QPalette>

#include <opencv2/core/core.hpp>

// ROS libraries
#include <ros/master.h>

#include <swri_image_util/geometry_util.h>
#include <swri_route_util/util.h>
#include <swri_transform_util/transform_util.h>
#include <mapviz/select_topic_dialog.h>

#include <marti_nav_msgs/Route.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::RoutePlugin, mapviz::MapvizPlugin)

namespace sru = swri_route_util;
namespace stu = swri_transform_util;

namespace mapviz_plugins
{
  RoutePlugin::RoutePlugin() : config_widget_(new QWidget()), draw_style_(LINES)
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
    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this,
                     SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this,
                     SLOT(TopicEdited()));
    QObject::connect(ui_.selectpositiontopic, SIGNAL(clicked()), this,
                     SLOT(SelectPositionTopic()));
    QObject::connect(ui_.positiontopic, SIGNAL(editingFinished()), this,
                     SLOT(PositionTopicEdited()));
    QObject::connect(ui_.drawstyle, SIGNAL(activated(QString)), this,
                     SLOT(SetDrawStyle(QString)));
    QObject::connect(ui_.color, SIGNAL(colorEdited(const QColor&)), this,
                     SLOT(DrawIcon()));
  }

  RoutePlugin::~RoutePlugin()
  {
  }

  void RoutePlugin::DrawIcon()
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

      icon_->SetPixmap(icon);
    }
  }

  void RoutePlugin::SetDrawStyle(QString style)
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
  }

  void RoutePlugin::SelectTopic()
  {
    ros::master::TopicInfo topic =
        mapviz::SelectTopicDialog::selectTopic("marti_nav_msgs/Route");

    if (topic.name.empty())
    {
      return;
    }

    ui_.topic->setText(QString::fromStdString(topic.name));
    TopicEdited();
  }

  void RoutePlugin::SelectPositionTopic()
  {
    ros::master::TopicInfo topic =
        mapviz::SelectTopicDialog::selectTopic("marti_nav_msgs/RoutePosition");

    if (topic.name.empty())
    {
      return;
    }

    ui_.positiontopic->setText(QString::fromStdString(topic.name));
    PositionTopicEdited();
  }

  void RoutePlugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();
    if (topic != topic_)
    {
      src_route_ = sru::Route();

      route_sub_.shutdown();

      topic_ = topic;
      if (!topic.empty())
      {
        route_sub_ =
            node_.subscribe(topic_, 1, &RoutePlugin::RouteCallback, this);

        ROS_INFO("Subscribing to %s", topic_.c_str());
      }
    }
  }

  void RoutePlugin::PositionTopicEdited()
  {
    std::string topic = ui_.positiontopic->text().trimmed().toStdString();
    if (topic != position_topic_)
    {
      src_route_position_.reset();
      position_sub_.shutdown();

      if (!topic.empty())
      {
        position_topic_ = topic;
        position_sub_ = node_.subscribe(position_topic_, 1,
                                        &RoutePlugin::PositionCallback, this);

        ROS_INFO("Subscribing to %s", position_topic_.c_str());
      }
    }
  }

  void RoutePlugin::PositionCallback(
      const marti_nav_msgs::RoutePositionConstPtr& msg)
  {
    src_route_position_ = msg;
  }

  void RoutePlugin::RouteCallback(const marti_nav_msgs::RouteConstPtr& msg)
  {
    src_route_ = sru::Route(*msg);
  }

  void RoutePlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message, 1.0);
  }

  void RoutePlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message, 1.0);
  }

  void RoutePlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message, 1.0);
  }

  QWidget* RoutePlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool RoutePlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;

    DrawIcon();

    initialized_ = true;
    return true;
  }

  void RoutePlugin::Draw(double x, double y, double scale)
  {
    if (!src_route_.valid())
    {
      PrintError("No valid route received.");
      return;
    }

    sru::Route route = src_route_;
    if (route.header.frame_id.empty())
    {
      route.header.frame_id = "/wgs84";
    }

    stu::Transform transform;
    if (!GetTransform(route.header.frame_id, ros::Time(), transform))
    {
      PrintError("Failed to transform route");
      return;
    }

    sru::transform(route, transform, target_frame_);
    sru::projectToXY(route);
    sru::fillOrientations(route);

    DrawRoute(route);

    bool ok = true;
    if (route.valid() && src_route_position_)
    {
      sru::RoutePoint point;
      if (sru::interpolateRoutePosition(point, route, *src_route_position_, true))
      {
        DrawRoutePoint(point);
      }
      else
      {
        PrintError("Failed to find route position in route.");
        ok = false;
      }
    }

    if (ok)
    {
      PrintInfo("OK");
    }
  }

  void RoutePlugin::DrawStopWaypoint(double x, double y)
  {
    const double a = 2;
    const double S = a * 2.414213562373095;

    glBegin(GL_POLYGON);

    glColor3f(1.0, 0.0, 0.0);

    glVertex2d(x + S / 2.0, y - a / 2.0);
    glVertex2d(x + S / 2.0, y + a / 2.0);
    glVertex2d(x + a / 2.0, y + S / 2.0);
    glVertex2d(x - a / 2.0, y + S / 2.0);
    glVertex2d(x - S / 2.0, y + a / 2.0);
    glVertex2d(x - S / 2.0, y - a / 2.0);
    glVertex2d(x - a / 2.0, y - S / 2.0);
    glVertex2d(x + a / 2.0, y - S / 2.0);

    glEnd();
  }

  void RoutePlugin::DrawRoute(const sru::Route& route)
  {
    const QColor color = ui_.color->color();
    glColor4d(color.redF(), color.greenF(), color.blueF(), 1.0);

    if (draw_style_ == LINES)
    {
      glLineWidth(3);
      glBegin(GL_LINE_STRIP);
    }
    else
    {
      glPointSize(2);
      glBegin(GL_POINTS);
    }

    for (size_t i = 0; i < route.points.size(); i++)
    {
      glVertex2d(route.points[i].position().x(),
                 route.points[i].position().y());
    }
    glEnd();
  }

  void RoutePlugin::DrawRoutePoint(const sru::RoutePoint& point)
  {
    const double arrow_size = ui_.iconsize->value();

    tf::Vector3 v1(arrow_size, 0.0, 0.0);
    tf::Vector3 v2(0.0, arrow_size / 2.0, 0.0);
    tf::Vector3 v3(0.0, -arrow_size / 2.0, 0.0);

    tf::Transform point_g(point.orientation(), point.position());

    v1 = point_g * v1;
    v2 = point_g * v2;
    v3 = point_g * v3;

    const QColor color = ui_.positioncolor->color();
    glLineWidth(3);
    glBegin(GL_POLYGON);
    glColor4d(color.redF(), color.greenF(), color.blueF(), 1.0);
    glVertex2d(v1.x(), v1.y());
    glVertex2d(v2.x(), v2.y());
    glVertex2d(v3.x(), v3.y());
    glEnd();
  }

  void RoutePlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["topic"])
    {
      std::string route_topic;
      node["topic"] >> route_topic;
      ui_.topic->setText(route_topic.c_str());
    }
    if (node["color"])
    {
      std::string color;
      node["color"] >> color;
      ui_.color->setColor(QColor(color.c_str()));
    }
    if (node["postopic"])
    {
      std::string pos_topic;
      node["postopic"] >> pos_topic;
      ui_.positiontopic->setText(pos_topic.c_str());
    }
    if (node["poscolor"])
    {
      std::string poscolor;
      node["poscolor"] >> poscolor;
      ui_.positioncolor->setColor(QColor(poscolor.c_str()));
    }

    if (node["draw_style"])
    {
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
    }

    TopicEdited();
    PositionTopicEdited();
  }

  void RoutePlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string route_topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "topic" << YAML::Value << route_topic;

    std::string color = ui_.color->color().name().toStdString();
    emitter << YAML::Key << "color" << YAML::Value << color;

    std::string pos_topic = ui_.positiontopic->text().toStdString();
    emitter << YAML::Key << "postopic" << YAML::Value << pos_topic;

    std::string pos_color = ui_.positioncolor->color().name().toStdString();
    emitter << YAML::Key << "poscolor" << YAML::Value << pos_color;

    std::string draw_style = ui_.drawstyle->currentText().toStdString();
    emitter << YAML::Key << "draw_style" << YAML::Value << draw_style;
  }
}
