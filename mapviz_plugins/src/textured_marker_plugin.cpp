// *****************************************************************************
//
// Copyright (c) 2014-2020, Southwest Research Institute® (SwRI®)
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

#include <mapviz_plugins/textured_marker_plugin.h>
#include <mapviz_plugins/topic_select.h>

// Boost libraries
#include <boost/algorithm/string.hpp>

// QT libraries
#include <QDialog>
#include <QGLWidget>

// ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

// Declare plugin
#include <pluginlib/class_list_macros.hpp>

// C++ standard libraries
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

using namespace std::chrono_literals;

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::TexturedMarkerPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
TexturedMarkerPlugin::TexturedMarkerPlugin() :
  MapvizPlugin(),
  ui_(),
  alphaVal_(1.0f),
  config_widget_(new QWidget()),
  has_message_(false),
  topic_(""),
  qos_(rmw_qos_profile_default)
{
  ui_.setupUi(config_widget_);

  // Set background white
  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Window, Qt::white);
  config_widget_->setPalette(p);

  // Set status text red
  QPalette p3(ui_.status->palette());
  p3.setColor(QPalette::Text, Qt::red);
  ui_.status->setPalette(p3);

  QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
  QObject::connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
  QObject::connect(ui_.clear, SIGNAL(clicked()), this, SLOT(ClearHistory()));
  QObject::connect(ui_.alphaSlide, SIGNAL(valueChanged(int)), this, SLOT(SetAlphaLevel(int)));

  // By using a signal/slot connection, we ensure that we only generate GL textures on the
  // main thread in case a non-main thread handles the ROS callbacks.
  qRegisterMetaType<marti_visualization_msgs::msg::TexturedMarker>("TexturedMarker");

  QObject::connect(this,
    SIGNAL(MarkerReceived(marti_visualization_msgs::msg::TexturedMarker)),
    this,
    SLOT(ProcessMarker(marti_visualization_msgs::msg::TexturedMarker)));
}

void TexturedMarkerPlugin::ClearHistory()
{
  RCLCPP_DEBUG(node_->get_logger(), "TexturedMarkerPlugin::ClearHistory()");
  markers_.clear();
}

// TODO(P. J. Reed) could instead use the value() function on alphaSlide when needed,
//   assuming value is always good
// Modify min and max values by adjusting textured_marker_config.ui
void TexturedMarkerPlugin::SetAlphaLevel(int alpha)
{
  int max = ui_.alphaSlide->maximum();
  int min = ui_.alphaSlide->minimum();

  if (max < 1 ||
    min < 0 ||
    alpha > max ||
    alpha < min)    // ignore negative min and max
  {
    alphaVal_ = 1.0f;
    PrintWarning("Invalid alpha input.");
  } else {
    // Ex. convert int in range 0-100 to float in range 0-1
    alphaVal_ = (static_cast<float>(alpha) / max);
    RCLCPP_INFO(node_->get_logger(), "Adjusting alpha value to: %f", alphaVal_);
  }
}

void TexturedMarkerPlugin::SelectTopic()
{
  auto [topic, qos] = SelectTopicDialog::selectTopic(
    node_,
    "marti_visualization_msgs/msg/TexturedMarker",
    "marti_visualization_msgs/msg/TexturedMarkerArray",
    qos_);

  if (!topic.empty()) {
    connectCallback(topic, qos);
  }
}

void TexturedMarkerPlugin::TopicEdited()
{
  std::string topic = ui_.topic->text().trimmed().toStdString();
  connectCallback(topic, qos_);
}

void TexturedMarkerPlugin::connectCallback(const std::string& topic, const rmw_qos_profile_t& qos)
{
  ui_.topic->setText(QString::fromStdString(topic));

  if ((topic != topic_) || !qosEqual(qos, qos_)) {
    initialized_ = false;
    markers_.clear();
    has_message_ = false;
    PrintWarning("No messages received.");

    marker_sub_.reset();
    marker_arr_sub_.reset();

    topic_ = topic;
    qos_ = qos;
    if (!topic.empty()) {
      auto known_topics = node_->get_topic_names_and_types();
      if (known_topics.count(topic_) > 0) {
        rclcpp::QoS topic_qos(rclcpp::QoSInitialization::from_rmw(qos_));
        std::string topic_type = known_topics[topic_][0];
        if (topic_type == "marti_visualization_msgs/msg/TexturedMarkerArray") {
          marker_arr_sub_ =
            node_->create_subscription<marti_visualization_msgs::msg::TexturedMarkerArray>(
            topic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos),
            std::bind(&TexturedMarkerPlugin::MarkerArrayCallback, this, std::placeholders::_1)
            );
          RCLCPP_INFO(node_->get_logger(), "Subscribing to %s", topic_.c_str());
        }
        else if(topic_type == "marti_visualization_msgs/msg/TexturedMarker") {
          marker_sub_ = node_->create_subscription<marti_visualization_msgs::msg::TexturedMarker>(
            topic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos),
            std::bind(&TexturedMarkerPlugin::MarkerCallback, this, std::placeholders::_1)
          );
          RCLCPP_INFO(node_->get_logger(), "Subscribing to %s", topic_.c_str());
        }
        else {
          RCLCPP_ERROR(node_->get_logger(),
              "Unable to subscribe to topic %s (unsupported type %s).",
              topic_.c_str(), topic_type.c_str());
        }
      }
      else {
        RCLCPP_ERROR(node_->get_logger(),
            "Unable to subscribe to topic %s, (does not exist).", topic_.c_str());
      }
    }
  }

}

void TexturedMarkerPlugin::ProcessMarker(const marti_visualization_msgs::msg::TexturedMarker marker)
{
  if (!has_message_) {
    initialized_ = true;
    has_message_ = true;
  }

  // Note that unlike some plugins, this one does not store nor rely on the
  // source_frame_ member variable.  This one can potentially store many
  // messages with different source frames, so we need to store and transform
  // them individually.

  if (marker.action == marti_visualization_msgs::msg::TexturedMarker::ADD) {
    MarkerData & markerData = markers_[marker.ns][marker.id];
    markerData.stamp = marker.header.stamp;

    markerData.transformed = true;
    markerData.alpha_ = marker.alpha;
    markerData.source_frame_ = marker.header.frame_id;

    swri_transform_util::Transform transform;
    if (!GetTransform(markerData.source_frame_, marker.header.stamp, transform)) {
      markerData.transformed = false;
      PrintError("No transform between " + markerData.source_frame_ + " and " + target_frame_);
    }

    // Handle lifetime parameter
    rclcpp::Duration lifetime = marker.lifetime;
    if (lifetime.seconds() == 0 && lifetime.nanoseconds() == 0) {
      markerData.expire_time = rclcpp::Time::max();
    } else {
      // Temporarily add 5 seconds to fix some existing markers.
      markerData.expire_time = rclcpp::Time() + lifetime + rclcpp::Duration(5s);
    }

    tf2::Transform offset(
      tf2::Quaternion(
        marker.pose.orientation.x,
        marker.pose.orientation.y,
        marker.pose.orientation.z,
        marker.pose.orientation.w),
      tf2::Vector3(
        marker.pose.position.x,
        marker.pose.position.y,
        marker.pose.position.z));

    double right = marker.image.width * marker.resolution / 2.0;
    double left = -right;
    double top = marker.image.height * marker.resolution / 2.0;
    double bottom = -top;

    tf2::Vector3 top_left(left, top, 0);
    tf2::Vector3 top_right(right, top, 0);
    tf2::Vector3 bottom_left(left, bottom, 0);
    tf2::Vector3 bottom_right(right, bottom, 0);

    top_left = offset * top_left;
    top_right = offset * top_right;
    bottom_left = offset * bottom_left;
    bottom_right = offset * bottom_right;

    markerData.quad_.clear();
    markerData.quad_.push_back(top_left);
    markerData.quad_.push_back(top_right);
    markerData.quad_.push_back(bottom_right);

    markerData.quad_.push_back(top_left);
    markerData.quad_.push_back(bottom_right);
    markerData.quad_.push_back(bottom_left);

    markerData.transformed_quad_.clear();
    for (size_t i = 0; i < markerData.quad_.size(); i++) {
      markerData.transformed_quad_.push_back(transform * markerData.quad_[i]);
    }

    int32_t max_dimension = std::max(marker.image.height, marker.image.width);
    int32_t new_size = 1;
    while (new_size < max_dimension) {
      new_size = new_size << 1;
    }

    if (new_size != markerData.texture_size_ || markerData.encoding_ != marker.image.encoding) {
      markerData.texture_size_ = new_size;

      markerData.encoding_ = marker.image.encoding;

      GLuint ids[1];

      //  Free the current texture.
      if (markerData.texture_id_ != -1) {
        ids[0] = static_cast<GLuint>(markerData.texture_id_);
        glDeleteTextures(1, &ids[0]);
      }

      // Get a new texture id.
      glGenTextures(1, &ids[0]);
      markerData.texture_id_ = ids[0];

      // Bind the texture with the correct size and null memory.
      glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(markerData.texture_id_));

      glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

      size_t bpp = 0;
      if (markerData.encoding_ == sensor_msgs::image_encodings::BGRA8) {
        bpp = 4;
        markerData.texture_.resize(static_cast<size_t>(markerData.texture_size_ *
          markerData.texture_size_ * 4));
      } else if (markerData.encoding_ == sensor_msgs::image_encodings::BGR8) {
        bpp = 3;
        markerData.texture_.resize(static_cast<size_t>(markerData.texture_size_ *
          markerData.texture_size_ * 3));
      } else if (markerData.encoding_ == sensor_msgs::image_encodings::MONO8) {
        bpp = 1;
        markerData.texture_.resize(static_cast<size_t>(markerData.texture_size_ *
          markerData.texture_size_));
      } else {
        RCLCPP_WARN(node_->get_logger(), "Unsupported encoding: %s", markerData.encoding_.c_str());
      }

      size_t expected = marker.image.height * marker.image.width * bpp;
      if (!markerData.texture_.empty() && marker.image.data.size() < expected) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "TexturedMarker image had expected data size %li but only got %li. Dropping message.",
          expected,
          marker.image.data.size());
        return;
      }

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

      glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

      glBindTexture(GL_TEXTURE_2D, 0);

      glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    }

    glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(markerData.texture_id_));

    if (markerData.encoding_ == sensor_msgs::image_encodings::BGRA8) {
      for (size_t row = 0; row < marker.image.height; row++) {
        for (size_t col = 0; col < marker.image.width; col++) {
          size_t src_index = (row * marker.image.width + col) * 4;
          size_t dst_index = (row * markerData.texture_size_ + col) * 4;

          markerData.texture_[dst_index + 0] = marker.image.data[src_index + 0];
          markerData.texture_[dst_index + 1] = marker.image.data[src_index + 1];
          markerData.texture_[dst_index + 2] = marker.image.data[src_index + 2];
          markerData.texture_[dst_index + 3] = marker.image.data[src_index + 3];
        }
      }

      glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RGBA,
        markerData.texture_size_,
        markerData.texture_size_,
        0,
        GL_BGRA,
        GL_UNSIGNED_BYTE,
        markerData.texture_.data());
    } else if (markerData.encoding_ == sensor_msgs::image_encodings::BGR8) {
      for (size_t row = 0; row < marker.image.height; row++) {
        for (size_t col = 0; col < marker.image.width; col++) {
          size_t src_index = (row * marker.image.width + col) * 3;
          size_t dst_index = (row * markerData.texture_size_ + col) * 3;

          markerData.texture_[dst_index + 0] = marker.image.data[src_index + 0];
          markerData.texture_[dst_index + 1] = marker.image.data[src_index + 1];
          markerData.texture_[dst_index + 2] = marker.image.data[src_index + 2];
        }
      }

      glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RGB,
        markerData.texture_size_,
        markerData.texture_size_,
        0,
        GL_BGR,
        GL_UNSIGNED_BYTE,
        markerData.texture_.data());
    } else if (markerData.encoding_ == sensor_msgs::image_encodings::MONO8) {
      for (size_t row = 0; row < marker.image.height; row++) {
        for (size_t col = 0; col < marker.image.width; col++) {
          size_t src_index = row * marker.image.width + col;
          size_t dst_index = row * markerData.texture_size_ + col;

          markerData.texture_[dst_index] = marker.image.data[src_index];
        }
      }

      glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_LUMINANCE,
        markerData.texture_size_,
        markerData.texture_size_,
        0,
        GL_LUMINANCE,
        GL_UNSIGNED_BYTE,
        markerData.texture_.data());
    }

    glBindTexture(GL_TEXTURE_2D, 0);

    markerData.texture_x_ = static_cast<float>(marker.image.width) /
      static_cast<float>(markerData.texture_size_);
    markerData.texture_y_ = static_cast<float>(marker.image.height) /
      static_cast<float>(markerData.texture_size_);
  } else {
    markers_[marker.ns].erase(marker.id);
  }
}

void TexturedMarkerPlugin::MarkerCallback(
  marti_visualization_msgs::msg::TexturedMarker::ConstSharedPtr marker)
{
  Q_EMIT MarkerReceived(*marker);
}

void TexturedMarkerPlugin::MarkerArrayCallback(
  marti_visualization_msgs::msg::TexturedMarkerArray::ConstSharedPtr markers)
{
  for (const auto & marker : markers->markers) {
    Q_EMIT MarkerReceived(marker);
  }
}

void TexturedMarkerPlugin::PrintError(const std::string & message)
{
  PrintErrorHelper(ui_.status, message);
}

void TexturedMarkerPlugin::PrintInfo(const std::string & message)
{
  PrintInfoHelper(ui_.status, message);
}

void TexturedMarkerPlugin::PrintWarning(const std::string & message)
{
  PrintWarningHelper(ui_.status, message);
}

QWidget * TexturedMarkerPlugin::GetConfigWidget(QWidget * parent)
{
  config_widget_->setParent(parent);

  return config_widget_;
}

bool TexturedMarkerPlugin::Initialize(QGLWidget * canvas)
{
  canvas_ = canvas;

  return true;
}

void TexturedMarkerPlugin::Draw(double x, double y, double scale)
{
  rclcpp::Time now = rclcpp::Time();

  float alphaVal = alphaVal_;   // Set all markers to same alpha value

  std::map<std::string, std::map<int, MarkerData>>::iterator nsIter;
  for (nsIter = markers_.begin(); nsIter != markers_.end(); ++nsIter) {
    std::map<int, MarkerData>::iterator markerIter;
    for (markerIter = nsIter->second.begin(); markerIter != nsIter->second.end(); ++markerIter) {
      MarkerData & marker = markerIter->second;
      marker.alpha_ = alphaVal;   // Update current marker's alpha value

      if (marker.expire_time > now) {
        if (marker.transformed) {
          glEnable(GL_TEXTURE_2D);

          glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(marker.texture_id_));

          glBegin(GL_TRIANGLES);

          glColor4f(1.0f, 1.0f, 1.0f, marker.alpha_);

          double marker_x = marker.texture_x_;
          double marker_y = marker.texture_y_;

          glTexCoord2d(0, 0); glVertex2d(
            marker.transformed_quad_[0].x(), marker.transformed_quad_[0].y());
          glTexCoord2d(marker_x, 0); glVertex2d(
            marker.transformed_quad_[1].x(), marker.transformed_quad_[1].y());
          glTexCoord2d(marker_x, marker_y); glVertex2d(
            marker.transformed_quad_[2].x(), marker.transformed_quad_[2].y());

          glTexCoord2d(0, 0); glVertex2d(
            marker.transformed_quad_[3].x(), marker.transformed_quad_[3].y());
          glTexCoord2d(marker_x, marker_y); glVertex2d(
            marker.transformed_quad_[4].x(), marker.transformed_quad_[4].y());
          glTexCoord2d(0, marker_y); glVertex2d(
            marker.transformed_quad_[5].x(), marker.transformed_quad_[5].y());

          glEnd();

          glBindTexture(GL_TEXTURE_2D, 0);

          glDisable(GL_TEXTURE_2D);

          PrintInfo("OK");
        }
      }
    }
  }
}

void TexturedMarkerPlugin::Transform()
{
  std::map<std::string, std::map<int, MarkerData>>::iterator nsIter;
  for (nsIter = markers_.begin(); nsIter != markers_.end(); ++nsIter) {
    std::map<int, MarkerData>::iterator markerIter;
    for (markerIter = nsIter->second.begin(); markerIter != nsIter->second.end(); ++markerIter) {
      swri_transform_util::Transform transform;
      if (GetTransform(markerIter->second.source_frame_, markerIter->second.stamp, transform)) {
        markerIter->second.transformed_quad_.clear();
        for (size_t i = 0; i < markerIter->second.quad_.size(); i++) {
          markerIter->second.transformed_quad_.push_back(transform * markerIter->second.quad_[i]);
        }
      }
    }
  }
}

void TexturedMarkerPlugin::LoadConfig(const YAML::Node & node, const std::string & path)
{
  LoadQosConfig(node, qos_);
  if (node["topic"]) {
    std::string topic = node["topic"].as<std::string>();
    ui_.topic->setText(boost::trim_copy(topic).c_str());
  }

  TopicEdited();
}

void TexturedMarkerPlugin::SaveConfig(YAML::Emitter & emitter, const std::string & path)
{
  emitter << YAML::Key << "topic" << YAML::Value <<
    boost::trim_copy(ui_.topic->text().toStdString());

  SaveQosConfig(emitter, qos_);
}
}   // namespace mapviz_plugins
