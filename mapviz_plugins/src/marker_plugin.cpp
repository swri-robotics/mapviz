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

#include <mapviz_plugins/marker_plugin.h>

// C++ standard libraries
#include <cmath>
#include <cstdio>
#include <vector>

// Boost libraries
#include <boost/algorithm/string.hpp>

// QT libraries
#include <QDialog>
#include <QGLWidget>

// ROS libraries
#include <ros/master.h>

#include <math_util/constants.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
    mapviz_plugins,
    marker,
    mapviz_plugins::MarkerPlugin,
    mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  MarkerPlugin::MarkerPlugin() :
    config_widget_(new QWidget()),
    is_marker_array_(false)
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

    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
  }

  MarkerPlugin::~MarkerPlugin()
  {
  }

  void MarkerPlugin::SelectTopic()
  {
    QDialog dialog;
    Ui::topicselect ui;
    ui.setupUi(&dialog);

    std::vector<ros::master::TopicInfo> topics;
    ros::master::getTopics(topics);

    for (unsigned int i = 0; i < topics.size(); i++)
    {
      if (topics[i].datatype == "visualization_msgs/Marker" || topics[i].datatype == "visualization_msgs/MarkerArray")
      {
        ui.displaylist->addItem(topics[i].name.c_str());
      }
    }
    ui.displaylist->setCurrentRow(0);

    dialog.exec();

    if (dialog.result() == QDialog::Accepted && ui.displaylist->selectedItems().count() == 1)
    {
      ui_.topic->setText(ui.displaylist->selectedItems().first()->text());

      // Determine if this is a marker array
      is_marker_array_ = false;
      for (unsigned int i = 0; i < topics.size(); i++)
      {
        if (topics[i].datatype == "visualization_msgs/MarkerArray")
        {
          if (ui.displaylist->selectedItems().first()->text().toStdString() == topics[i].name)
          {
            is_marker_array_ = true;
          }
        }
      }

      TopicEdited();
    }
  }

  void MarkerPlugin::TopicEdited()
  {
    if (ui_.topic->text().toStdString() != topic_)
    {
      initialized_ = false;
      markers_.clear();
      has_message_ = false;
      topic_ = boost::trim_copy(ui_.topic->text().toStdString());
      PrintWarning("No messages received.");

      marker_sub_.shutdown();

      if (is_marker_array_)
      {
        marker_sub_ = node_.subscribe(topic_, 100, &MarkerPlugin::markerArrayCallback, this);
      }
      else
      {
        marker_sub_ = node_.subscribe(topic_, 100, &MarkerPlugin::markerCallback, this);
      }

      ResetVisibility();

      ROS_INFO("Subscribing to %s", topic_.c_str());
    }
  }

  void MarkerPlugin::markerCallback(const visualization_msgs::MarkerConstPtr marker)
  {
    if (!has_message_)
    {
      source_frame_ = marker->header.frame_id;
      initialized_ = true;
      has_message_ = true;
    }

    if (marker->action == visualization_msgs::Marker::ADD)
    {
      MarkerData& markerData = markers_[marker->ns][marker->id];
      markerData.stamp = marker->header.stamp;
      markerData.display_type = marker->type;
      markerData.color = QColor::fromRgbF(marker->color.r, marker->color.g, marker->color.b, marker->color.a);
      markerData.scale_x = marker->scale.x;
      markerData.scale_y = marker->scale.y;
      markerData.scale_z = marker->scale.z;
      markerData.transformed = true;


      // Since orientation was not implemented, many markers publish
      // invalid all-zero orientations, so we need to check for this
      // and provide a default identity transform.
      tf::Quaternion orientation(0.0, 0.0, 0.0, 1.0);      
      if (marker->pose.orientation.x ||
          marker->pose.orientation.y ||
          marker->pose.orientation.z ||
          marker->pose.orientation.w)
      {
        orientation = tf::Quaternion(marker->pose.orientation.x,
                                     marker->pose.orientation.y,
                                     marker->pose.orientation.z,
                                     marker->pose.orientation.w);
      }
      
      markerData.local_transform =  transform_util::Transform(
        tf::Transform(
          orientation,
          tf::Vector3(marker->pose.position.x,
                      marker->pose.position.y,
                      marker->pose.position.z)));

      markerData.points.clear();
      markerData.text = std::string();

      transform_util::Transform transform;
      if (!GetTransform(marker->header.stamp, transform))
      {
        markerData.transformed = false;
        PrintError("No transform between " + source_frame_ + " and " + target_frame_);
      }

      // Handle lifetime parameter
      ros::Duration lifetime = marker->lifetime;
      if (lifetime.isZero())
      {
        markerData.expire_time = ros::TIME_MAX;
      }
      else
      {
        // Temporarily add 5 seconds to fix some existing markers.
        markerData.expire_time = ros::Time::now() + lifetime + ros::Duration(5);
      }

      if (markerData.display_type == visualization_msgs::Marker::CYLINDER ||
          markerData.display_type == visualization_msgs::Marker::CUBE)
      {
        StampedPoint point;
        point.point = tf::Point(0.0, 0.0, 0.0);
        point.transformed_point = transform * (markerData.local_transform * point.point);
        point.color = markerData.color;

        markerData.points.push_back(point);
      }
      else if (markerData.display_type == visualization_msgs::Marker::TEXT_VIEW_FACING)
      {
        StampedPoint point;
        point.point = tf::Point(0.0, 0.0, 0.0);
        point.transformed_point = transform * (markerData.local_transform * point.point);
        point.color = markerData.color;

        markerData.points.push_back(point);
        markerData.text = marker->text;
      }
      else
      {
        for (unsigned int i = 0; i < marker->points.size(); i++)
        {
          StampedPoint point;
          point.point = tf::Point(marker->points[i].x, marker->points[i].y, marker->points[i].z);
          point.transformed_point = transform * (markerData.local_transform * point.point);

          if (i < marker->colors.size())
          {
            point.color = QColor::fromRgbF(
                marker->colors[i].r,
                marker->colors[i].g,
                marker->colors[i].b,
                marker->colors[i].a);
          }
          else
          {
            point.color = markerData.color;
          }

          markerData.points.push_back(point);
        }
      }

      if (!visibility_.count(marker->ns)) {
        visibility_[marker->ns].visible = true;
        visibility_[marker->ns].item = NULL;
      }

      if (!visibility_[marker->ns].item) {
        QListWidgetItem *item = new QListWidgetItem();
        item->setText(QString::fromStdString(marker->ns));
        if (visibility_[marker->ns].visible) {
          item->setCheckState(Qt::Checked);
        } else {
          item->setCheckState(Qt::Unchecked);
        }
        QObject::connect(ui_.markerList,
                         SIGNAL(itemChanged(QListWidgetItem*)), this,
                         SLOT(MarkerEdited(QListWidgetItem*)));

        visibility_[marker->ns].item = item;
        ui_.markerList->addItem(item);
      }
    }
    else
    {
      markers_[marker->ns].erase(marker->id);

      if (visibility_.count(marker->ns) && visibility_[marker->ns].item) {
        int row = ui_.markerList->row(visibility_[marker->ns].item);
        QListWidgetItem *item = ui_.markerList->takeItem(row);
        if (!item || item != visibility_[marker->ns].item) {
          ROS_ERROR("Invalid row %d for marker list item.", row);
        } else {
          delete visibility_[marker->ns].item;
          visibility_[marker->ns].item = NULL;
        }
        // Note: We do not delete the namespace from the visibility
        // list because we want the user's choice to persist between
        // DEL/ADD cycles and between mapviz sessions.
      }
    }

    canvas_->update();
  }

  void MarkerPlugin::markerArrayCallback(const visualization_msgs::MarkerArrayConstPtr markers)
  {
    for (unsigned int i = 0; i < markers->markers.size(); i++)
    {
      markerCallback(visualization_msgs::MarkerConstPtr(new visualization_msgs::Marker(markers->markers[i])));
    }
  }

  void MarkerPlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_ERROR("Error: %s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void MarkerPlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_INFO("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void MarkerPlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_WARN("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  QWidget* MarkerPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool MarkerPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;

    return true;
  }

  void MarkerPlugin::Draw(double x, double y, double scale)
  {
    ros::Time now = ros::Time::now();

    std::map<std::string, std::map<int, MarkerData> >::iterator nsIter;
    for (nsIter = markers_.begin(); nsIter != markers_.end(); ++nsIter)
    {
      if (!visibility_.count(nsIter->first)) {
        ROS_WARN("Namespace %s not in visiblity list!", nsIter->first.c_str());
      } else if (!visibility_[nsIter->first].visible) {
        continue;
      }

      std::map<int, MarkerData>::iterator markerIter;
      for (markerIter = nsIter->second.begin(); markerIter != nsIter->second.end(); ++markerIter)
      {
        MarkerData& marker = markerIter->second;

        if (marker.expire_time > now)
        {
          if (marker.transformed)
          {
            glColor4f(marker.color.redF(), marker.color.greenF(), marker.color.blueF(), 1.0f);

            if (marker.display_type == visualization_msgs::Marker::LINE_STRIP)
            {
              glLineWidth(marker.scale_x);
              glBegin(GL_LINE_STRIP);

                std::list<StampedPoint>::iterator point_it = marker.points.begin();
                for (; point_it != marker.points.end(); ++point_it)
                {
                  glColor4f(
                      point_it->color.redF(),
                      point_it->color.greenF(),
                      point_it->color.blueF(),
                      point_it->color.alphaF());

                  glVertex2f(
                      point_it->transformed_point.getX(),
                      point_it->transformed_point.getY());
                }

              glEnd();
            }
            if (marker.display_type == visualization_msgs::Marker::LINE_LIST)
            {
              glLineWidth(marker.scale_x);
              glBegin(GL_LINES);

                std::list<StampedPoint>::iterator point_it = marker.points.begin();
                for (; point_it != marker.points.end(); ++point_it)
                {
                  glColor4f(
                      point_it->color.redF(),
                      point_it->color.greenF(),
                      point_it->color.blueF(),
                      point_it->color.alphaF());

                  glVertex2f(
                      point_it->transformed_point.getX(),
                      point_it->transformed_point.getY());
                }

              glEnd();
            }
            else if (marker.display_type == visualization_msgs::Marker::POINTS)
            {
              glPointSize(marker.scale_x);
              glBegin(GL_POINTS);

              std::list<StampedPoint>::iterator point_it = marker.points.begin();
              for (; point_it != marker.points.end(); ++point_it)
              {
                glColor4f(
                    point_it->color.redF(),
                    point_it->color.greenF(),
                    point_it->color.blueF(),
                    point_it->color.alphaF());

                glVertex2f(
                    point_it->transformed_point.getX(),
                    point_it->transformed_point.getY());
              }

              glEnd();
            }
            else if (marker.display_type == visualization_msgs::Marker::TRIANGLE_LIST)
            {
              glBegin(GL_TRIANGLES);

              std::list<StampedPoint>::iterator point_it = marker.points.begin();
              for (; point_it != marker.points.end(); ++point_it)
              {
                glColor4f(
                    point_it->color.redF(),
                    point_it->color.greenF(),
                    point_it->color.blueF(),
                    point_it->color.alphaF());

                glVertex2f(
                    point_it->transformed_point.getX(),
                    point_it->transformed_point.getY());
              }

              glEnd();
            }
            else if (marker.display_type == visualization_msgs::Marker::CYLINDER ||
                marker.display_type == visualization_msgs::Marker::SPHERE ||
                marker.display_type == visualization_msgs::Marker::SPHERE_LIST)
            {
              std::list<StampedPoint>::iterator point_it = marker.points.begin();
              for (; point_it != marker.points.end(); ++point_it)
              {
                glColor4f(
                    point_it->color.redF(),
                    point_it->color.greenF(),
                    point_it->color.blueF(),
                    point_it->color.alphaF());


                glBegin(GL_TRIANGLE_FAN);


                float x = point_it->transformed_point.getX();
                float y = point_it->transformed_point.getY();

                glVertex2f(x, y);

                for (int32_t i = 0; i <= 360; i += 10)
                {
                  float radians = static_cast<float>(i) * math_util::_deg_2_rad;
                  // Spheres may be specified w/ only one scale value
                  if(marker.scale_y == 0.0)
                  {
                    marker.scale_y = marker.scale_x;
                  }
                  glVertex2f(
                      x + std::sin(radians) * marker.scale_x,
                      y + std::cos(radians) * marker.scale_y);
                }

                glEnd();
              }
            }
            else if (marker.display_type == visualization_msgs::Marker::CUBE ||
                marker.display_type == visualization_msgs::Marker::CUBE_LIST)
            {
              std::list<StampedPoint>::iterator point_it = marker.points.begin();
              for (; point_it != marker.points.end(); ++point_it)
              {
                glColor4f(
                    point_it->color.redF(),
                    point_it->color.greenF(),
                    point_it->color.blueF(),
                    point_it->color.alphaF());


                glBegin(GL_TRIANGLE_FAN);


                float x = point_it->transformed_point.getX();
                float y = point_it->transformed_point.getY();

                glVertex2f(x + marker.scale_x / 2, y + marker.scale_x / 2);
                glVertex2f(x - marker.scale_x / 2, y + marker.scale_x / 2);
                glVertex2f(x - marker.scale_x / 2, y - marker.scale_x / 2);
                glVertex2f(x + marker.scale_x / 2, y - marker.scale_x / 2);

                glEnd();
              }
            }
            else if (marker.display_type== visualization_msgs::Marker::TEXT_VIEW_FACING)
            {
              if (!marker.points.empty())
              {
                StampedPoint& point = marker.points.front();
                std::string text = marker.text;

                glPushMatrix();
                glRasterPos3f(point.transformed_point.x(), point.transformed_point.y(), 0.0);
                for (uint32_t i = 0; i < text.size(); i++)
                {
                  glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10,
                      static_cast<int>(text.c_str()[i]));
                    // glutBitmapCharacter(glutBitmapHelvetica10,buf.at(i));
                }
                glPushMatrix();
              }
            }

            PrintInfo("OK");
          }
        }
        else
        {
          PrintInfo("OK");
          ROS_ERROR("Not displaying expired marker[%s:%d]: %lf < %lf (now)", topic_.c_str(), markerIter->first, marker.expire_time.toSec(), now.toSec());
        }
      }
    }
  }

  void MarkerPlugin::Transform()
  {
    std::map<std::string, std::map<int, MarkerData> >::iterator nsIter;
    for (nsIter = markers_.begin(); nsIter != markers_.end(); ++nsIter)
    {
      std::map<int, MarkerData>::iterator markerIter;
      for (markerIter = nsIter->second.begin(); markerIter != nsIter->second.end(); ++markerIter)
      {
        MarkerData& marker = markerIter->second;

        transform_util::Transform transform;
        if (GetTransform(marker.stamp, transform))
        {
          marker.transformed = true;

          std::list<StampedPoint>::iterator point_it = marker.points.begin();
          for (; point_it != marker.points.end(); ++point_it)
          {
            point_it->transformed_point = transform * (marker.local_transform * point_it->point);
          }
        }
        else
        {
          marker.transformed = false;
        }
      }
    }
  }

  void MarkerPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    std::string topic;
    node["topic"] >> topic;
    ui_.topic->setText(boost::trim_copy(topic).c_str());

    node["is_marker_array"] >> is_marker_array_;

    LoadVisibilityConfig(node);

    TopicEdited();
  }

  void MarkerPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "topic" << YAML::Value << boost::trim_copy(ui_.topic->text().toStdString());
    emitter << YAML::Key << "is_marker_array" << YAML::Value << is_marker_array_;

    SaveVisibilityConfig(emitter);
  }

  void MarkerPlugin::MarkerEdited(QListWidgetItem *item)
  {
    std::string ns = item->text().toStdString();
    if (!visibility_.count(ns)) {
      ROS_ERROR("Could not find visibility entry for '%s'", ns.c_str());
      return;
    }

    visibility_[ns].visible = (item->checkState() == Qt::Checked);
  }

  void MarkerPlugin::LoadVisibilityConfig(const YAML::Node& top_node)
  {
    if (!top_node.FindValue("visible_items")) {
      return;
    }

    const YAML::Node& node = top_node["visible_items"];

    if (node.Type() != YAML::NodeType::Map) {
      ROS_WARN("Unexpected YAML type for visible items: %d", node.Type());
      return;
    }

    for (YAML::Iterator it = node.begin(); it != node.end(); ++it) {
      visibility_[it.first().to<std::string>()].visible = it.second().to<bool>();
      visibility_[it.first().to<std::string>()].item = NULL;
    }
  }

  void MarkerPlugin::SaveVisibilityConfig(YAML::Emitter& emitter)
  {
    emitter << YAML::Key << "visible_items";
    emitter << YAML::Value << YAML::BeginMap;

    for (std::map<std::string, MarkerListItem>::iterator it = visibility_.begin();
         it != visibility_.end();
         ++it)
    {
      if (!it->second.visible) {
        // Since the plugin will make new markers by default, we can
        // keep the config file from getting too cluttered by only
        // saving the hidden items.
        emitter << YAML::Key << it->first << YAML::Value << it->second.visible;
      }
    }

    emitter << YAML::EndMap;
  }

  void MarkerPlugin::ResetVisibility()
  {
    ui_.markerList->clear();
    visibility_.clear();
  }
}

