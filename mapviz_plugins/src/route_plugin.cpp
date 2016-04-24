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
#include <swri_transform_util/transform_util.h>
#include <mapviz/select_topic_dialog.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(mapviz_plugins, route, mapviz_plugins::RoutePlugin,
                        mapviz::MapvizPlugin);

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
    has_message_ = false;
    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);
    points_.clear();
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
    if (ui_.topic->text().toStdString() != topic_)
    {
      initialized_ = false;
      points_.clear();
      has_message_ = false;
      topic_ = ui_.topic->text().toStdString();
      PrintWarning("No messages received.");

      route_sub_.shutdown();
      route_sub_ =
          node_.subscribe(topic_, 1, &RoutePlugin::MartiCallback, this);

      ROS_INFO("Subscribing to %s", topic_.c_str());
    }
  }
  void RoutePlugin::PositionTopicEdited()
  {
    if (ui_.positiontopic->text().toStdString() != position_topic_)
    {
      position_topic_ = ui_.positiontopic->text().toStdString();
      PrintWarning("No messages received.");
      position_sub_.shutdown();
      position_sub_ = node_.subscribe(position_topic_, 1,
                                      &RoutePlugin::PositionCallback, this);

      ROS_INFO("Subscribing to %s", position_topic_.c_str());
    }
  }
  void RoutePlugin::PositionCallback(
      const marti_nav_msgs::RoutePositionConstPtr msg)
  {
    if (!has_message_)
    {
      ROS_ERROR_THROTTLE(1.0, "No route message");
      return;
    }
    distance_ = msg->distance;
    std::list<StampedPoint>::iterator points_it = points_.begin();
    points_it++;
    std::list<StampedPoint>::iterator end_it = points_.end();
    end_it--;

    if (points_.front().id.empty())
    {
      ROS_ERROR("Stored points have no Id");
    }

    for (; points_it != end_it; ++points_it)
    {
      if (points_it->id == msg->id)
      {
        cur_position_ = *points_it;

        std::list<StampedPoint>::iterator next_point = ++points_it;
        next_position_ = *(next_point);

        if (!points_it->transformed)
        {
          TransformPoint(cur_position_);
          TransformPoint(next_position_);
        }

        return;
      }
    }
  }

  void RoutePlugin::MartiCallback(const marti_nav_msgs::RouteConstPtr route)
  {
    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }
    if (!route->header.frame_id.empty())
    {
      source_frame_ = route->header.frame_id;
    }
    else
    {
      source_frame_ = "/wgs84";
    }
    size_t route_size = route->route_points.size();

    points_.clear();

    for (size_t i = 0; i < route_size; i++)
    {
      StampedPoint stamped_point;
      stamped_point.stamp = route->header.stamp;
      stamped_point.transformed = false;
      stamped_point.point = tf::Point(route->route_points[i].pose.position.x,
                                      route->route_points[i].pose.position.y,
                                      route->route_points[i].pose.position.z);

      stamped_point.orientation =
          tf::Quaternion(route->route_points[i].pose.orientation.x,
                         route->route_points[i].pose.orientation.y,
                         route->route_points[i].pose.orientation.z,
                         route->route_points[i].pose.orientation.w);
      stamped_point.id = route->route_points[i].id;
      points_.push_back(stamped_point);
    }
  }

  void RoutePlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_ERROR("Error: %s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void RoutePlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_INFO("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void RoutePlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_WARN("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
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

    return true;
  }

  void RoutePlugin::Draw(double x, double y, double scale)
  {
    QColor color = ui_.color->color();

    bool transformed = false;

    glColor4f(color.redF(), color.greenF(), color.blueF(), 1.0);

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

    std::list<StampedPoint>::iterator it = points_.begin();

    for (; it != points_.end(); ++it)
    {
      if (it->transformed)
      {
        glVertex2f(it->transformed_point.getX(), it->transformed_point.getY());

        transformed = true;
      }
    }

    glEnd();
    it--;
    DrawStopWaypoint(it->transformed_point.x(), it->transformed_point.y());
    if (!position_topic_.empty())
    {
      DrawCurrentWaypointIndicator();
    }
    if (transformed)
    {
      PrintInfo("OK");
    }
  }

  bool RoutePlugin::TransformPoint(StampedPoint& point)
  {
    if (point.transformed == true)
    {
      return true;
    }

    swri_transform_util::Transform transform;
    if (GetTransform(point.stamp, transform))
    {
      point.transformed_point = transform * point.point;
      point.transformed = true;
      return true;
    }

    point.transformed = false;
    return false;
  }

  void RoutePlugin::Transform()
  {
    bool transformed = false;
    std::list<StampedPoint>::iterator points_it = points_.begin();
    for (; points_it != points_.end(); ++points_it)
    {
      transformed = transformed | TransformPoint(*points_it);
    }

    if (!points_.empty() && !transformed)
    {
      PrintError("No transform between " + source_frame_ + " and " +
                 target_frame_);
    }
  }

  void RoutePlugin::DrawStopWaypoint(double x, double y)
  {
    const double a = 2;
    const double S = a * 2.414213562373095;

    glBegin(GL_POLYGON);

    glColor3f(1.0, 0.0, 0.0);

    glVertex2f(x + S / 2.0, y - a / 2.0);
    glVertex2f(x + S / 2.0, y + a / 2.0);
    glVertex2f(x + a / 2.0, y + S / 2.0);
    glVertex2f(x - a / 2.0, y + S / 2.0);
    glVertex2f(x - S / 2.0, y + a / 2.0);
    glVertex2f(x - S / 2.0, y - a / 2.0);
    glVertex2f(x - a / 2.0, y - S / 2.0);
    glVertex2f(x + a / 2.0, y - S / 2.0);

    glEnd();
  }
  void RoutePlugin::CalculateTangentVectorAtPoint(double* t_dx, double* t_dy)
  {
    ROS_ASSERT(t_dx != NULL);
    ROS_ASSERT(t_dy != NULL);
    double dx = 0.0;
    double dy = 0.0;

    dx += (next_position_.transformed_point.x() -
           cur_position_.transformed_point.x());
    dy += (next_position_.transformed_point.y() -
           cur_position_.transformed_point.y());

    // Normalize the tangent vector
    double d = sqrt(dx * dx + dy * dy);
    dx = dx / d;
    dy = dy / d;

    *t_dx = dx;
    *t_dy = dy;
  }
  void RoutePlugin::CalculateVectorsAtPoint(double* t_dx, double* t_dy,
                                            double* n_dx, double* n_dy)
  {
    ROS_ASSERT(t_dx != NULL);
    ROS_ASSERT(t_dy != NULL);
    ROS_ASSERT(n_dx != NULL);
    ROS_ASSERT(n_dy != NULL);
    CalculateTangentVectorAtPoint(t_dx, t_dy);
    *n_dx = -*t_dy;
    *n_dy = *t_dx;
  }
  void RoutePlugin::DrawCurrentWaypointIndicator()
  {
    double t_dx = 0.0;
    double t_dy = 0.0;
    double n_dx = 0.0;
    double n_dy = 0.0;
    CalculateVectorsAtPoint(&t_dx, &t_dy, &n_dx, &n_dy);
    double orig_x = cur_position_.transformed_point.x();
    double orig_y = cur_position_.transformed_point.y();
    double x = cur_position_.transformed_point.x() + t_dx * distance_;
    double y = cur_position_.transformed_point.y() + t_dy * distance_;
    glLineWidth(3);
    glBegin(GL_POLYGON);
    QColor color = ui_.positioncolor->color();
    glColor4f(color.redF(), color.greenF(), color.blueF(), 1.0);
    int outer_dist = ui_.iconsize->value();

    glVertex2f(x, y);
    glVertex2f(x + n_dx * outer_dist / 2, y + n_dy * outer_dist / 2);

    glVertex2f(x + n_dx * outer_dist / 2, y + n_dy * outer_dist / 2);
    glVertex2f(x + (t_dx * outer_dist), y + (t_dy * outer_dist));

    glVertex2f(x + (t_dx * outer_dist), y + (t_dy * outer_dist));
    glVertex2f(x - n_dx * outer_dist / 2, y - n_dy * outer_dist / 2);

    glVertex2f(x - n_dx * outer_dist / 2, y - n_dy * outer_dist / 2);
    glVertex2f(x, y);

    glEnd();
  }

  void RoutePlugin::UpdateConfig(std::map<std::string, std::string>& params)
  {
    if (params.count("topic") > 0)
    {
      ui_.topic->setText(boost::trim_copy(params["topic"]).c_str());
    }
    
    if (params.count("color") > 0)
    {
      ui_.color->setColor(QColor(params["color"].c_str()));
    }

    if (params.count("postopic") > 0)
    {
      ui_.positiontopic->setText(boost::trim_copy(params["postopic"]).c_str());
    }

    if (params.count("poscolor") > 0)
    {
      ui_.positioncolor->setColor(QColor(params["poscolor"].c_str()));
    }

    if (params.count("draw_style") > 0)
    {
      if (params["draw_style"] == "lines")
      {
        draw_style_ = LINES;
        ui_.drawstyle->setCurrentIndex(0);
      }
      else if (params["draw_style"] == "points")
      {
        draw_style_ = POINTS;
        ui_.drawstyle->setCurrentIndex(1);
      }
    }

    TopicEdited();
    PositionTopicEdited();
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
