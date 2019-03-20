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

#include <mapviz/select_topic_dialog.h>

#include <swri_math_util/constants.h>

#include <boost/algorithm/string.hpp>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::MarkerPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
#define IS_INSTANCE(msg, type) \
  (msg->getDataType() == ros::message_traits::datatype<type>())

  MarkerPlugin::MarkerPlugin() :
    config_widget_(new QWidget()),
    connected_(false)
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
    QObject::connect(ui_.clear, SIGNAL(clicked()), this, SLOT(ClearHistory()));

    startTimer(1000);
  }

  MarkerPlugin::~MarkerPlugin()
  {
  }

  void MarkerPlugin::ClearHistory()
  {
    ROS_INFO("MarkerPlugin::ClearHistory()");
    markers_.clear();
  }

  void MarkerPlugin::SelectTopic()
  {
    ros::master::TopicInfo topic = mapviz::SelectTopicDialog::selectTopic(
      "visualization_msgs/Marker",
      "visualization_msgs/MarkerArray");

    if (topic.name.empty())
    {
      return;
    }

    ui_.topic->setText(QString::fromStdString(topic.name));
    TopicEdited();
  }

  void MarkerPlugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();
    if (topic != topic_)
    {
      initialized_ = false;
      markers_.clear();
      has_message_ = false;
      PrintWarning("No messages received.");

      marker_sub_.shutdown();
      connected_ = false;

      topic_ = topic;
      if (!topic.empty())
      {
        marker_sub_ = node_.subscribe<topic_tools::ShapeShifter>(
            topic_, 100, &MarkerPlugin::handleMessage, this);

        ROS_INFO("Subscribing to %s", topic_.c_str());
      }
    }
  }

  void MarkerPlugin::handleMessage(const topic_tools::ShapeShifter::ConstPtr& msg)
  {
    connected_ = true;
    if (IS_INSTANCE(msg, visualization_msgs::Marker))
    {
      handleMarker(*(msg->instantiate<visualization_msgs::Marker>()));
    }
    else if (IS_INSTANCE(msg, visualization_msgs::MarkerArray))
    {
      handleMarkerArray(*(msg->instantiate<visualization_msgs::MarkerArray>()));
    }
    else
    {
      PrintError("Unknown message type: " + msg->getDataType());
    }
  }

  void MarkerPlugin::handleMarker(const visualization_msgs::Marker &marker)
  {
    if (marker.type == visualization_msgs::Marker::ARROW &&
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

    if (marker.action == visualization_msgs::Marker::ADD)
    {
      MarkerData& markerData = markers_[std::make_pair(marker.ns, marker.id)];
      markerData.points.clear(); // clear marker points
      markerData.text.clear(); // clear marker text
      markerData.stamp = marker.header.stamp;
      markerData.display_type = marker.type;
      markerData.color = {marker.color.r, marker.color.g, marker.color.b, marker.color.a};
      markerData.scale_x = static_cast<float>(marker.scale.x);
      markerData.scale_y = static_cast<float>(marker.scale.y);
      markerData.scale_z = static_cast<float>(marker.scale.z);
      markerData.transformed = true;
      markerData.source_frame = marker.header.frame_id;


      // Since orientation was not implemented, many markers publish
      // invalid all-zero orientations, so we need to check for this
      // and provide a default identity transform.
      tf::Quaternion orientation(0.0, 0.0, 0.0, 1.0);
      if (marker.pose.orientation.x ||
          marker.pose.orientation.y ||
          marker.pose.orientation.z ||
          marker.pose.orientation.w)
      {
        orientation = tf::Quaternion(marker.pose.orientation.x,
                                     marker.pose.orientation.y,
                                     marker.pose.orientation.z,
                                     marker.pose.orientation.w);
      }

      markerData.local_transform = tf::Transform(
          orientation,
          tf::Vector3(marker.pose.position.x,
                      marker.pose.position.y,
                      marker.pose.position.z));

      markerData.points.clear();
      markerData.text = std::string();

      swri_transform_util::Transform transform;
      if (!GetTransform(markerData.source_frame, marker.header.stamp, transform))
      {
        markerData.transformed = false;
        PrintError("No transform between " + markerData.source_frame + " and " + target_frame_);
      }

      // Handle lifetime parameter
      ros::Duration lifetime = marker.lifetime;
      if (lifetime.isZero())
      {
        markerData.expire_time = ros::TIME_MAX;
      }
      else
      {
        // Temporarily add 5 seconds to fix some existing markers.
        markerData.expire_time = ros::Time::now() + lifetime + ros::Duration(5);
      }

      if (markerData.display_type == visualization_msgs::Marker::ARROW)
      {
        StampedPoint point;
        point.color = markerData.color;
        point.orientation = orientation;

        if (marker.points.empty())
        {
          // If the "points" array is empty, we'll use the pose as the base of
          // the arrow and scale its size based on the scale_x value.
          point.point = markerData.local_transform * tf::Point(0.0, 0.0, 0.0);
          point.arrow_point = markerData.local_transform * tf::Point(1.0, 0.0, 0.0);
        }
        else
        {
          // Otherwise the "points" array should have exactly two values, the
          // start and end of the arrow.
          point.point = markerData.local_transform * tf::Point(marker.points[0].x, marker.points[0].y, marker.points[0].z);
          point.arrow_point = markerData.local_transform * tf::Point(marker.points[1].x, marker.points[1].y, marker.points[1].z);
        }

        markerData.points.push_back(point);

        if (!marker.points.empty())
        {
          // The point we just pushed back has both the start and end of the
          // arrow, so the point we're pushing here is useless; we use it later
          // only to indicate whether the original message had two points or not.
          markerData.points.push_back(StampedPoint());
        }

        transformArrow(markerData, transform);
      }
      else if (markerData.display_type == visualization_msgs::Marker::CYLINDER ||
        markerData.display_type == visualization_msgs::Marker::SPHERE ||
        markerData.display_type == visualization_msgs::Marker::TEXT_VIEW_FACING)
      {
        StampedPoint point;
        point.point = tf::Point(0.0, 0.0, 0.0);
        point.transformed_point = transform * (markerData.local_transform * point.point);
        point.color = markerData.color;
        markerData.points.push_back(point);
        markerData.text = marker.text;
      }
      else if (markerData.display_type == visualization_msgs::Marker::CUBE)
      {
        StampedPoint point;
        point.color = markerData.color;

        point.point = tf::Point(marker.scale.x / 2, marker.scale.y / 2, 0.0);
        point.transformed_point = transform * (markerData.local_transform * point.point);
        markerData.points.push_back(point);

        point.point = tf::Point(-marker.scale.x / 2, marker.scale.y / 2, 0.0);
        point.transformed_point = transform * (markerData.local_transform * point.point);
        markerData.points.push_back(point);

        point.point = tf::Point(-marker.scale.x / 2, -marker.scale.y / 2, 0.0);
        point.transformed_point = transform * (markerData.local_transform * point.point);
        markerData.points.push_back(point);

        point.point = tf::Point(marker.scale.x / 2, -marker.scale.y / 2, 0.0);
        point.transformed_point = transform * (markerData.local_transform * point.point);
        markerData.points.push_back(point);
      }
      else if (markerData.display_type == visualization_msgs::Marker::LINE_STRIP ||
        markerData.display_type == visualization_msgs::Marker::LINE_LIST ||
        markerData.display_type == visualization_msgs::Marker::CUBE_LIST ||
        markerData.display_type == visualization_msgs::Marker::SPHERE_LIST ||
        markerData.display_type == visualization_msgs::Marker::POINTS ||
        markerData.display_type == visualization_msgs::Marker::TRIANGLE_LIST)
      {
        tf::Transform tfTransform(transform.GetTF());
        tfTransform *= markerData.local_transform;

        markerData.points.reserve(marker.points.size());
        StampedPoint point;
        for (unsigned int i = 0; i < marker.points.size(); i++)
        {
          point.point = tf::Point(marker.points[i].x, marker.points[i].y, marker.points[i].z);
          point.transformed_point = tfTransform * point.point;

          if (i < marker.colors.size())
          {
            point.color = {marker.colors[i].r, marker.colors[i].g, marker.colors[i].b, marker.colors[i].a};
          }
          else
          {
            point.color = markerData.color;
          }

          markerData.points.push_back(point);
        }
      }
      else
      {
        ROS_WARN_ONCE("Unsupported marker type: %d", markerData.display_type);
      }
    }
    else if (marker.action == visualization_msgs::Marker::DELETE)
    {
      markers_.erase(std::make_pair(marker.ns, marker.id));
    }
    else if (marker.action == 3) // The DELETEALL enum doesn't exist in Indigo
    {
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
    tf::Point arrowOffset;
    if (markerData.points.size() == 1)
    {
      // If the markerData only has a single point, that means its "point" is
      // the base of the arrow and the arrow's angle and length are determined
      // by the orientation and scale_x.
      point.transformed_point = transform * (markerData.local_transform * point.point);
      tf::Transform arrow_tf(tf::Transform(
          transform.GetOrientation()) * point.orientation,
          tf::Point(0.0, 0.0, 0.0));
      point.transformed_arrow_point = point.transformed_point +
          arrow_tf * point.arrow_point * markerData.scale_x;
      arrowOffset = tf::Point(0.25, 0.0, 0.0);
    }
    else
    {
      // If the markerData has two points, that means that the start and end points
      // of the arrow were explicitly specified in the original message, so the
      // length and angle are determined by them.
      point.transformed_point = transform * point.point;
      point.transformed_arrow_point = transform * point.arrow_point;
      // Also, in this mode, scale_y is the diameter of the arrow's head.
      arrowOffset = tf::Point(0.25 * markerData.scale_y, 0.0, 0.0);
    }

    tf::Vector3 pointDiff = point.transformed_arrow_point - point.transformed_point;
    double angle = std::atan2(pointDiff.getY(), pointDiff.getX());

    tf::Transform left_tf(tf::createQuaternionFromRPY(0, 0, M_PI*0.75 + angle));
    tf::Transform right_tf(tf::createQuaternionFromRPY(0, 0, -M_PI*0.75 + angle));

    point.transformed_arrow_left = point.transformed_arrow_point + left_tf * arrowOffset;
    point.transformed_arrow_right = point.transformed_arrow_point + right_tf * arrowOffset;
  }

  void MarkerPlugin::handleMarkerArray(const visualization_msgs::MarkerArray &markers)
  {
    for (unsigned int i = 0; i < markers.markers.size(); i++)
    {
      handleMarker(markers.markers[i]);
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
    ros::Time now = ros::Time::now();

    for (auto markerIter = markers_.begin(); markerIter != markers_.end(); ++markerIter)
    {
      MarkerData& marker = markerIter->second;

      if (!(marker.expire_time > now)) {
        PrintInfo("OK");
        markerIter = markers_.erase(markerIter);
        continue;
      }

      if (!marker.transformed) {
        continue;
      }

      glColor4f(marker.color.r, marker.color.g, marker.color.b, marker.color.a);

      if (marker.display_type == visualization_msgs::Marker::ARROW) {
        if (marker.points.size() == 1) {
          // If the marker only has one point, use scale_y as the arrow width.
          glLineWidth(marker.scale_y);
        }
        else {
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
      }
      else if (marker.display_type == visualization_msgs::Marker::LINE_STRIP) {
        glLineWidth(marker.scale_x);
        glBegin(GL_LINE_STRIP);

        for (const auto &point : marker.points) {
          glColor4f(point.color.r, point.color.g, point.color.b, point.color.a);

          glVertex2d(
            point.transformed_point.getX(),
            point.transformed_point.getY());
        }

        glEnd();
      }
      else if (marker.display_type == visualization_msgs::Marker::LINE_LIST) {
        glLineWidth(marker.scale_x);
        glBegin(GL_LINES);

        for (const auto &point : marker.points) {
          glColor4f(point.color.r, point.color.g, point.color.b, point.color.a);

          glVertex2d(
            point.transformed_point.getX(),
            point.transformed_point.getY());
        }

        glEnd();
      }
      else if (marker.display_type == visualization_msgs::Marker::POINTS) {
        glPointSize(marker.scale_x);
        glBegin(GL_POINTS);

        for (const auto &point : marker.points) {
          glColor4f(point.color.r, point.color.g, point.color.b, point.color.a);

          glVertex2d(
            point.transformed_point.getX(),
            point.transformed_point.getY());
        }

        glEnd();
      }
      else if (marker.display_type == visualization_msgs::Marker::TRIANGLE_LIST) {
        glBegin(GL_TRIANGLES);

        for (const auto &point : marker.points) {
          glColor4f(point.color.r, point.color.g, point.color.b, point.color.a);

          glVertex2d(
            point.transformed_point.getX(),
            point.transformed_point.getY());
        }

        glEnd();
      }
      else if (marker.display_type == visualization_msgs::Marker::CYLINDER ||
        marker.display_type == visualization_msgs::Marker::SPHERE ||
        marker.display_type == visualization_msgs::Marker::SPHERE_LIST) {
        for (const auto &point : marker.points) {
          glColor4f(point.color.r, point.color.g, point.color.b, point.color.a);

          glBegin(GL_TRIANGLE_FAN);


          double marker_x = point.transformed_point.getX();
          double marker_y = point.transformed_point.getY();

          glVertex2d(marker_x, marker_y);

          for (int32_t i = 0; i <= 360; i += 10) {
            double radians = static_cast<double>(i) * static_cast<double>(swri_math_util::_deg_2_rad);
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
      }
      else if (marker.display_type == visualization_msgs::Marker::CUBE ||
        marker.display_type == visualization_msgs::Marker::CUBE_LIST) {
        glBegin(GL_TRIANGLE_FAN);
        for (const auto &point : marker.points) {
          glColor4f(point.color.r, point.color.g, point.color.b, point.color.a);

          glVertex2d(point.transformed_point.getX(), point.transformed_point.getY());
        }
        glEnd();
      }

      PrintInfo("OK");
    }
  }

  void MarkerPlugin::Paint(QPainter* painter, double x, double y, double scale)
  {
    // Most of the marker drawing is done using OpenGL commands, but text labels
    // are rendered using a QPainter.  This is intended primarily as an example
    // of how the QPainter works.
    ros::Time now = ros::Time::now();

    // We don't want the text to be rotated or scaled, but we do want it to be
    // translated appropriately.  So, we save off the current world transform
    // and reset it; when we actually draw the text, we'll manually translate
    // it to the right place.
    QTransform tf = painter->worldTransform();
    QFont font("Helvetica", 10);
    painter->setFont(font);
    painter->save();
    painter->resetTransform();

    for (auto markerIter = markers_.begin(); markerIter != markers_.end(); ++markerIter)
    {
      MarkerData& marker = markerIter->second;

      if (marker.display_type != visualization_msgs::Marker::TEXT_VIEW_FACING ||
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
      QRectF rect(point, QSizeF(10,10));
      rect = painter->boundingRect(rect, Qt::AlignLeft | Qt::AlignHCenter, text);
      painter->drawText(rect, text);

      PrintInfo("OK");
    }

    painter->restore();
  }

  void MarkerPlugin::Transform()
  {
    for (auto markerIter = markers_.begin(); markerIter != markers_.end(); ++markerIter)
    {
      MarkerData& marker = markerIter->second;

      swri_transform_util::Transform transform;
      if (GetTransform(marker.source_frame, marker.stamp, transform))
      {
        marker.transformed = true;

        if (marker.display_type == visualization_msgs::Marker::ARROW)
        {
          // Points for the ARROW marker type are stored a bit differently
          // than other types, so they have their own special transform case.
          transformArrow(marker, transform);
        }
        else
        {
          tf::Transform tfTransform(transform.GetTF());
          tfTransform *= marker.local_transform;
          for (auto &point : marker.points)
          {
            point.transformed_point = tfTransform * point.point;
          }
        }
      }
      else
      {
        marker.transformed = false;
      }
    }
  }

  void MarkerPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["topic"])
    {
      std::string topic;
      node["topic"] >> topic;
      ui_.topic->setText(boost::trim_copy(topic).c_str());

      TopicEdited();
    }
  }

  void MarkerPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "topic" << YAML::Value << boost::trim_copy(ui_.topic->text().toStdString());
  }

  void MarkerPlugin::timerEvent(QTimerEvent *event)
  {
    bool new_connected = (marker_sub_.getNumPublishers() > 0);
    if (connected_ && !new_connected)
    {
      marker_sub_.shutdown();
      if (!topic_.empty())
      {
        marker_sub_ = node_.subscribe<topic_tools::ShapeShifter>(
            topic_, 100, &MarkerPlugin::handleMessage, this);
      }
    }
    connected_ = new_connected;
  }
}

