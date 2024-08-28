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

#include <mapviz_plugins/marker_plugin.h>
#include <mapviz_plugins/topic_select.h>

#include <swri_math_util/constants.h>

#include <boost/algorithm/string.hpp>

// Declare plugin
#include <pluginlib/class_list_macros.hpp>

// C++ Standard Libraries
#include <string>
#include <utility>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::MarkerPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  MarkerPlugin::MarkerPlugin() :
    MapvizPlugin(),
    ui_(),
    config_widget_(new QWidget()),
    connected_(false),
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

    startTimer(1000);
  }

  void MarkerPlugin::ClearHistory()
  {
    RCLCPP_DEBUG(node_->get_logger(), "MarkerPlugin::ClearHistory()");
    markers_.clear();
    marker_visible_.clear();
    ui_.nsList->clear();
  }

  void MarkerPlugin::SelectTopic()
  {
    auto [topic, qos] = SelectTopicDialog::selectTopic(
      node_,
      "visualization_msgs/msg/Marker",
      "visualization_msgs/msg/MarkerArray",
      qos_);
    if (!topic.empty())
    {
      connectCallback(topic, qos);
    }
  }

  void MarkerPlugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();
    connectCallback(topic, qos_);
  }

  void MarkerPlugin::connectCallback(const std::string& topic, const rmw_qos_profile_t& qos)
  {
    ui_.topic->setText(QString::fromStdString(topic));

    if ((topic != topic_) || !qosEqual(qos, qos_))
    {
      initialized_ = false;
      markers_.clear();
      marker_visible_.clear();
      ui_.nsList->clear();
      has_message_ = false;
      PrintWarning("No messages received.");

      marker_sub_.reset();
      marker_array_sub_.reset();
      connected_ = false;

      topic_ = topic;
      qos_ = qos;
    }

    marker_sub_.reset();
    marker_array_sub_.reset();
    if (!topic_.empty())
    {
      // ROS 2 does not allow for a simple way to subscribe to a general type of message (i.e. Marker and MarkerArray)
      // That would require a way to de-serialize the data for mapviz to consume (based on message type)
      // The code below checks for the topic type and subscribes in the appropriate manner

      auto known_topics = node_->get_topic_names_and_types();
      if (known_topics.count(topic_) > 0)
      {
        std::string topic_type = known_topics[topic_][0];
        if (topic_type == "visualization_msgs/msg/Marker")
        {
          marker_sub_ = node_->create_subscription<visualization_msgs::msg::Marker>(
            topic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos),
            std::bind(&MarkerPlugin::handleMarker, this, std::placeholders::_1));
        }
        else if (topic_type == "visualization_msgs/msg/MarkerArray")
        {
          marker_array_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
            topic_,
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos),
            std::bind(&MarkerPlugin::handleMarkerArray, this, std::placeholders::_1));
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(),
            "Unable to subscribe to topic %s (unsupported type %s).",
            topic_.c_str(), topic_type.c_str());
          return;
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(),
            "Unable to subscribe to topic %s (does not exist).", topic_.c_str());
        return;
      }

      RCLCPP_INFO(node_->get_logger(), "Subscribing to %s", topic_.c_str());
    }
  }

  void MarkerPlugin::handleMarker(visualization_msgs::msg::Marker::ConstSharedPtr marker)
  {
    processMarker(*marker);
  }

  void MarkerPlugin::processMarker(const visualization_msgs::msg::Marker& marker)
  {
    connected_ = true;
    if (marker.type == visualization_msgs::msg::Marker::ARROW &&
        marker.points.size() == 1)
    {
      // Arrow markers must have either 0 or >1 points; exactly one point is
      // invalid.  If we get one with 1 point, assume it's corrupt and ignore it.
      return;
    }
    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }

    // Note that unlike some plugins, this one does not store nor rely on the
    // source_frame_ member variable.  This one can potentially store many
    // messages with different source frames, so we need to store and transform
    // them individually.

    if (marker.action == visualization_msgs::msg::Marker::ADD)
    {
      MarkerData& markerData = markers_[std::make_pair(marker.ns, marker.id)];
      markerData.points.clear();  // clear marker points
      markerData.text.clear();  // clear marker text
      markerData.stamp = marker.header.stamp;
      markerData.display_type = marker.type;
      markerData.color = {marker.color.r, marker.color.g, marker.color.b, marker.color.a};
      markerData.scale_x = static_cast<float>(marker.scale.x);
      markerData.scale_y = static_cast<float>(marker.scale.y);
      markerData.scale_z = static_cast<float>(marker.scale.z);
      markerData.transformed = true;
      markerData.source_frame = marker.header.frame_id;

      if (marker_visible_.emplace(marker.ns, true).second)
      {
        QString name_string(marker.ns.c_str());
        auto* item = new QListWidgetItem(name_string, ui_.nsList);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Unchecked);
        item->setCheckState(Qt::Checked);
      }


      // Since orientation was not implemented, many markers publish
      // invalid all-zero orientations, so we need to check for this
      // and provide a default identity transform.
      tf2::Quaternion orientation(0.0, 0.0, 0.0, 1.0);
      if (marker.pose.orientation.x ||
          marker.pose.orientation.y ||
          marker.pose.orientation.z ||
          marker.pose.orientation.w)
      {
        orientation = tf2::Quaternion(marker.pose.orientation.x,
                                     marker.pose.orientation.y,
                                     marker.pose.orientation.z,
                                     marker.pose.orientation.w);
      }

      markerData.local_transform = swri_transform_util::Transform(
        tf2::Transform(
          orientation,
          tf2::Vector3(marker.pose.position.x,
                      marker.pose.position.y,
                      marker.pose.position.z)));

      markerData.points.clear();
      markerData.text = std::string();

      swri_transform_util::Transform transform;
      if (!GetTransform(markerData.source_frame, marker.header.stamp, transform))
      {
        markerData.transformed = false;
        PrintError("No transform between " + markerData.source_frame + " and " + target_frame_);
      }

      // Handle lifetime parameter
      rclcpp::Duration lifetime = marker.lifetime;
      if (lifetime.nanoseconds() == 0)
      {
        markerData.expire_time = rclcpp::Time(rclcpp::Time::max().nanoseconds(),
            node_->get_clock()->get_clock_type());
      } else {
        // Temporarily add 5 seconds to fix some existing markers.
        markerData.expire_time = node_->now() + lifetime + rclcpp::Duration(5, 0);
      }

      if (markerData.display_type == visualization_msgs::msg::Marker::ARROW)
      {
        StampedPoint point;
        point.color = markerData.color;
        point.orientation = orientation;

        if (marker.points.empty())
        {
          // If the "points" array is empty, we'll use the pose as the base of
          // the arrow and scale its size based on the scale_x value.
          point.point = markerData.local_transform * tf2::Vector3(0.0, 0.0, 0.0);
          point.arrow_point = markerData.local_transform * tf2::Vector3(1.0, 0.0, 0.0);
        } else {
          // Otherwise the "points" array should have exactly two values, the
          // start and end of the arrow.
          point.point = markerData.local_transform * tf2::Vector3(
            marker.points[0].x,
            marker.points[0].y,
            marker.points[0].z);
          point.arrow_point = markerData.local_transform * tf2::Vector3(
            marker.points[1].x,
            marker.points[1].y,
            marker.points[1].z);
        }

        markerData.points.push_back(point);

        if (!marker.points.empty())
        {
          // The point we just pushed back has both the start and end of the
          // arrow, so the point we're pushing here is useless; we use it later
          // only to indicate whether the original message had two points or not.
          markerData.points.emplace_back(StampedPoint());
        }

        transformArrow(markerData, transform);
      } else if (markerData.display_type == visualization_msgs::msg::Marker::CYLINDER ||
          markerData.display_type == visualization_msgs::msg::Marker::SPHERE ||
          markerData.display_type == visualization_msgs::msg::Marker::TEXT_VIEW_FACING) {
        StampedPoint point;
        point.point = tf2::Vector3(0.0, 0.0, 0.0);
        point.transformed_point = transform * (markerData.local_transform * point.point);
        point.color = markerData.color;
        markerData.points.push_back(point);
        markerData.text = marker.text;
      } else if (markerData.display_type == visualization_msgs::msg::Marker::CUBE) {
        StampedPoint point;
        point.color = markerData.color;

        point.point = tf2::Vector3(marker.scale.x / 2, marker.scale.y / 2, 0.0);
        point.transformed_point = transform * (markerData.local_transform * point.point);
        markerData.points.push_back(point);

        point.point = tf2::Vector3(-marker.scale.x / 2, marker.scale.y / 2, 0.0);
        point.transformed_point = transform * (markerData.local_transform * point.point);
        markerData.points.push_back(point);

        point.point = tf2::Vector3(-marker.scale.x / 2, -marker.scale.y / 2, 0.0);
        point.transformed_point = transform * (markerData.local_transform * point.point);
        markerData.points.push_back(point);

        point.point = tf2::Vector3(marker.scale.x / 2, -marker.scale.y / 2, 0.0);
        point.transformed_point = transform * (markerData.local_transform * point.point);
        markerData.points.push_back(point);
      } else if (markerData.display_type == visualization_msgs::msg::Marker::LINE_STRIP ||
          markerData.display_type == visualization_msgs::msg::Marker::LINE_LIST ||
          markerData.display_type == visualization_msgs::msg::Marker::CUBE_LIST ||
          markerData.display_type == visualization_msgs::msg::Marker::SPHERE_LIST ||
          markerData.display_type == visualization_msgs::msg::Marker::POINTS ||
          markerData.display_type == visualization_msgs::msg::Marker::TRIANGLE_LIST) {
        markerData.points.reserve(marker.points.size());
        StampedPoint point;
        for (unsigned int i = 0; i < marker.points.size(); i++)
        {
          point.point = tf2::Vector3(marker.points[i].x, marker.points[i].y, marker.points[i].z);
          point.transformed_point = transform * (markerData.local_transform * point.point);

          if (i < marker.colors.size())
          {
            point.color = {
              marker.colors[i].r,
              marker.colors[i].g,
              marker.colors[i].b,
              marker.colors[i].a};
          } else {
            point.color = markerData.color;
          }

          markerData.points.push_back(point);
        }
      } else {
        RCLCPP_WARN_ONCE(
          node_->get_logger(),
          "Unsupported marker type: %d",
          markerData.display_type);
      }
    } else if (marker.action == visualization_msgs::msg::Marker::DELETE) {
      markers_.erase(std::make_pair(marker.ns, marker.id));
    } else if (marker.action == visualization_msgs::msg::Marker::DELETEALL) {
      markers_.clear();
    }
  }

  /**
   * Given a MarkerData that represents an arrow and a transform, this function
   * will generate the points involved in drawing the arrow and then transform
   * all of them into the target frame.
   * @param[inout] markerData A marker that represents an arrow.
   * @param[in] transform The tf that should be applied to the arrow's points.
   */
  void MarkerPlugin::transformArrow(MarkerData& markerData,
                      const swri_transform_util::Transform& transform)
  {
    // The first point in the markerData.points array always represents the
    // base of the arrow.
    StampedPoint& point = markerData.points.front();
    tf2::Vector3 arrowOffset;
    if (markerData.points.size() == 1)
    {
      // If the markerData only has a single point, that means its "point" is
      // the base of the arrow and the arrow's angle and length are determined
      // by the orientation and scale_x.
      point.transformed_point = transform * (markerData.local_transform * point.point);
      tf2::Transform arrow_tf(tf2::Transform(
          transform.GetOrientation()) * point.orientation,
          tf2::Vector3(0.0, 0.0, 0.0));
      point.transformed_arrow_point = point.transformed_point +
          arrow_tf * point.arrow_point * markerData.scale_x;
      arrowOffset = tf2::Vector3(0.25, 0.0, 0.0);
    } else {
      // If the markerData has two points, that means that the start and end points
      // of the arrow were explicitly specified in the original message, so the
      // length and angle are determined by them.
      point.transformed_point = transform * point.point;
      point.transformed_arrow_point = transform * point.arrow_point;
      // Also, in this mode, scale_y is the diameter of the arrow's head.
      arrowOffset = tf2::Vector3(0.25 * markerData.scale_y, 0.0, 0.0);
    }

    tf2::Vector3 pointDiff = point.transformed_arrow_point - point.transformed_point;
    double angle = std::atan2(pointDiff.getY(), pointDiff.getX());

    tf2::Quaternion left_q;
    left_q.setRPY(0, 0, M_PI*0.75 + angle);
    tf2::Transform left_tf(left_q);

    tf2::Quaternion right_q;
    right_q.setRPY(0, 0, -M_PI*0.75 + angle);
    tf2::Transform right_tf(right_q);

    point.transformed_arrow_left = point.transformed_arrow_point + left_tf * arrowOffset;
    point.transformed_arrow_right = point.transformed_arrow_point + right_tf * arrowOffset;
  }

  void MarkerPlugin::handleMarkerArray(visualization_msgs::msg::MarkerArray::ConstSharedPtr markers)
  {
    for (const auto & marker : markers->markers)
    {
      processMarker(marker);
    }
  }

  void MarkerPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void MarkerPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void MarkerPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
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
    for (size_t i = 0; i < ui_.nsList->count(); i++)
    {
      if (ui_.nsList->item(i)->checkState() == Qt::Checked)
      {
        marker_visible_[ui_.nsList->item(i)->text().toStdString()] = true;
      } else {
        marker_visible_[ui_.nsList->item(i)->text().toStdString()] = false;
      }
    }

    rclcpp::Time now = node_->now();

    auto markerIter = markers_.begin();
    while (markerIter != markers_.end())
    {
      MarkerData& marker = markerIter->second;

      if (!(marker.expire_time > now)) {
        PrintInfo("OK");
        markerIter = markers_.erase(markerIter);
        continue;
      }

      if (!marker.transformed) {
        markerIter++;
        continue;
      }

      if (!marker_visible_[markerIter->first.first])
      {
        markerIter++;
        continue;
      }

      glColor4f(marker.color.r, marker.color.g, marker.color.b, marker.color.a);

      if (marker.display_type == visualization_msgs::msg::Marker::ARROW) {
        if (marker.points.size() == 1) {
          // If the marker only has one point, use scale_y as the arrow width.
          glLineWidth(marker.scale_y);
        } else {
          // If the marker has both start and end points explicitly specified, use
          // scale_x as the shaft diameter.
          glLineWidth(marker.scale_x);
        }
        glBegin(GL_LINES);

        for (const auto &point : marker.points) {
          glColor4f(point.color.r, point.color.g, point.color.b, point.color.a);

          glVertex2d(
            point.transformed_point.getX(),
            point.transformed_point.getY());
          glVertex2d(
            point.transformed_arrow_point.getX(),
            point.transformed_arrow_point.getY());
          glVertex2d(
            point.transformed_arrow_point.getX(),
            point.transformed_arrow_point.getY());
          glVertex2d(
            point.transformed_arrow_left.getX(),
            point.transformed_arrow_left.getY());
          glVertex2d(
            point.transformed_arrow_point.getX(),
            point.transformed_arrow_point.getY());
          glVertex2d(
            point.transformed_arrow_right.getX(),
            point.transformed_arrow_right.getY());
        }

        glEnd();
      } else if (marker.display_type == visualization_msgs::msg::Marker::LINE_STRIP) {
        glLineWidth(std::max(1.0f, marker.scale_x));
        glBegin(GL_LINE_STRIP);

        for (const auto &point : marker.points) {
          glColor4f(point.color.r, point.color.g, point.color.b, point.color.a);

          glVertex2d(
            point.transformed_point.getX(),
            point.transformed_point.getY());
        }

        glEnd();
      } else if (marker.display_type == visualization_msgs::msg::Marker::LINE_LIST) {
        glLineWidth(std::max(1.0f, marker.scale_x));
        glBegin(GL_LINES);

        for (const auto &point : marker.points) {
          glColor4f(point.color.r, point.color.g, point.color.b, point.color.a);

          glVertex2d(
            point.transformed_point.getX(),
            point.transformed_point.getY());
        }

        glEnd();
      } else if (marker.display_type == visualization_msgs::msg::Marker::POINTS) {
        glPointSize(std::max(1.0f, marker.scale_x));
        glBegin(GL_POINTS);

        for (const auto &point : marker.points) {
          glColor4f(point.color.r, point.color.g, point.color.b, point.color.a);

          glVertex2d(
            point.transformed_point.getX(),
            point.transformed_point.getY());
        }

        glEnd();
      } else if (marker.display_type == visualization_msgs::msg::Marker::TRIANGLE_LIST) {
        glBegin(GL_TRIANGLES);

        for (const auto &point : marker.points) {
          glColor4f(point.color.r, point.color.g, point.color.b, point.color.a);

          glVertex2d(
            point.transformed_point.getX(),
            point.transformed_point.getY());
        }

        glEnd();
      } else if (marker.display_type == visualization_msgs::msg::Marker::CYLINDER ||
          marker.display_type == visualization_msgs::msg::Marker::SPHERE ||
          marker.display_type == visualization_msgs::msg::Marker::SPHERE_LIST) {
        for (const auto &point : marker.points) {
          glColor4f(point.color.r, point.color.g, point.color.b, point.color.a);

          glBegin(GL_TRIANGLE_FAN);


          double marker_x = point.transformed_point.getX();
          double marker_y = point.transformed_point.getY();

          glVertex2d(marker_x, marker_y);

          for (int32_t i = 0; i <= 360; i += 10) {
            double radians =
              static_cast<double>(i) * static_cast<double>(swri_math_util::_deg_2_rad);
            // Spheres may be specified w/ only one scale value
            if (marker.scale_y == 0.0) {
              marker.scale_y = marker.scale_x;
            }
            glVertex2d(
              marker_x + std::sin(radians) * marker.scale_x,
              marker_y + std::cos(radians) * marker.scale_y);
          }

          glEnd();
        }
      } else if (marker.display_type == visualization_msgs::msg::Marker::CUBE ||
          marker.display_type == visualization_msgs::msg::Marker::CUBE_LIST) {
        glBegin(GL_TRIANGLE_FAN);
        for (const auto &point : marker.points) {
          glColor4f(point.color.r, point.color.g, point.color.b, point.color.a);

          glVertex2d(point.transformed_point.getX(), point.transformed_point.getY());
        }
        glEnd();
      }

      markerIter++;
      PrintInfo("OK");
    }
  }

  void MarkerPlugin::Paint(QPainter* painter, double x, double y, double scale)
  {
    // Most of the marker drawing is done using OpenGL commands, but text labels
    // are rendered using a QPainter.  This is intended primarily as an example
    // of how the QPainter works.
    rclcpp::Time now = node_->now();

    // We don't want the text to be rotated or scaled, but we do want it to be
    // translated appropriately.  So, we save off the current world transform
    // and reset it; when we actually draw the text, we'll manually translate
    // it to the right place.
    QTransform tf = painter->worldTransform();
    QFont font("Helvetica", 10);
    painter->setFont(font);
    painter->save();
    painter->resetTransform();

    for (auto & markerIter : markers_)
    {
      MarkerData& marker = markerIter.second;

      if (marker.display_type != visualization_msgs::msg::Marker::TEXT_VIEW_FACING ||
          marker.expire_time <= now ||
          !marker.transformed)
      {
        continue;
      }

      QPen pen(QBrush(QColor::fromRgbF(marker.color.r, marker.color.g,
                             marker.color.b, marker.color.a)), 1);
      painter->setPen(pen);

      StampedPoint& rosPoint = marker.points.front();
      QPointF point = tf.map(QPointF(rosPoint.transformed_point.x(),
                                     rosPoint.transformed_point.y()));

      auto text = QString::fromStdString(marker.text);
      // Get bounding rectangle
      QRectF rect(point, QSizeF(10, 10));
      rect = painter->boundingRect(rect, Qt::AlignLeft | Qt::AlignHCenter, text);
      painter->drawText(rect, text);

      PrintInfo("OK");
    }

    painter->restore();
  }

  void MarkerPlugin::Transform()
  {
    for (auto & markerIter : markers_)
    {
      MarkerData& marker = markerIter.second;

      swri_transform_util::Transform transform;
      if (GetTransform(marker.source_frame, marker.stamp, transform))
      {
        marker.transformed = true;

        if (marker.display_type == visualization_msgs::msg::Marker::ARROW)
        {
          // Points for the ARROW marker type are stored a bit differently
          // than other types, so they have their own special transform case.
          transformArrow(marker, transform);
        } else {
          for (auto &point : marker.points)
          {
            point.transformed_point = transform * (marker.local_transform * point.point);
          }
        }
      } else {
        marker.transformed = false;
      }
    }
  }

  void MarkerPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    LoadQosConfig(node, qos_);
    if (node["topic"])
    {
      std::string topic = node["topic"].as<std::string>();
      ui_.topic->setText(boost::trim_copy(topic).c_str());

      TopicEdited();
    }
  }

  void MarkerPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key
      << "topic"
      << YAML::Value
      << boost::trim_copy(ui_.topic->text().toStdString());
    SaveQosConfig(emitter, qos_);
  }

  void MarkerPlugin::timerEvent(QTimerEvent *event)
  {
    bool new_connected = (marker_sub_ && marker_sub_->get_publisher_count() > 0) ||
        (marker_array_sub_ && marker_array_sub_->get_publisher_count() > 0);
    if (connected_ && !new_connected)
    {
      connectCallback(topic_, qos_);
    }
    connected_ = new_connected;
  }
}   // namespace mapviz_plugins

