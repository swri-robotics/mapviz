// *****************************************************************************
//
// Copyright (c) 2020, Southwest Research Institute® (SwRI®)
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

#include <mapviz_plugins/object_plugin.h>

#include <mapviz/select_topic_dialog.h>

#include <swri_math_util/constants.h>

#include <boost/algorithm/string.hpp>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::ObjectPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
#define IS_INSTANCE(msg, type) \
  (msg->getDataType() == ros::message_traits::datatype<type>())

  ObjectPlugin::ObjectPlugin() :
    config_widget_(new QWidget()),
    connected_(false)
  {
    ui_.setupUi(config_widget_);

    ui_.color->setColor(Qt::red);

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
    QObject::connect(ui_.color, SIGNAL(colorEdited(const QColor&)), this,
            SLOT(SetColor(const QColor&)));

    startTimer(1000);
  }

  ObjectPlugin::~ObjectPlugin()
  {
  }

  void ObjectPlugin::SetColor(const QColor& color)
  {
    color_ = color;
  }

  void ObjectPlugin::ClearHistory()
  {
    objects_.clear();
  }

  void ObjectPlugin::SelectTopic()
  {
    ros::master::TopicInfo topic = mapviz::SelectTopicDialog::selectTopic(
      "marti_nav_msgs/TrackedObjectArray",
      "marti_nav_msgs/ObstacleArray");

    if (topic.name.empty())
    {
      return;
    }

    ui_.topic->setText(QString::fromStdString(topic.name));
    TopicEdited();
  }

  void ObjectPlugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();
    if (topic != topic_)
    {
      initialized_ = false;
      objects_.clear();
      has_message_ = false;
      PrintWarning("No messages received.");

      object_sub_.shutdown();
      connected_ = false;

      topic_ = topic;
      if (!topic.empty())
      {
        object_sub_ = node_.subscribe<topic_tools::ShapeShifter>(
            topic_, 100, &ObjectPlugin::handleMessage, this);

        ROS_INFO("Subscribing to %s", topic_.c_str());
      }
    }
  }

  void ObjectPlugin::handleMessage(const topic_tools::ShapeShifter::ConstPtr& msg)
  {
    connected_ = true;
    if (IS_INSTANCE(msg, marti_nav_msgs::TrackedObjectArray))
    {
      objects_.clear();

      auto objs = msg->instantiate<marti_nav_msgs::TrackedObjectArray>();
      objects_.reserve(objs->objects.size());
      for (const auto& obj: objs->objects)
      {
        handleTrack(obj);
      }
    }
    else if (IS_INSTANCE(msg, marti_nav_msgs::ObstacleArray))
    {
      objects_.clear();

      auto objs = msg->instantiate<marti_nav_msgs::ObstacleArray>();
      objects_.reserve(objs->obstacles.size());
      for (const auto& obj: objs->obstacles)
      {
        handleObstacle(obj, objs->header);
      }
    }
    else
    {
      PrintError("Unknown message type: " + msg->getDataType());
    }
  }

  void ObjectPlugin::handleObstacle(const marti_nav_msgs::Obstacle& obj, const std_msgs::Header& header)
  {
    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }

    // Since orientation was not implemented, many markers publish
    // invalid all-zero orientations, so we need to check for this
    // and provide a default identity transform.
    tf::Quaternion orientation(0.0, 0.0, 0.0, 1.0);
    if (obj.pose.orientation.x ||
        obj.pose.orientation.y ||
        obj.pose.orientation.z ||
        obj.pose.orientation.w)
    {
      orientation = tf::Quaternion(obj.pose.orientation.x,
                                   obj.pose.orientation.y,
                                   obj.pose.orientation.z,
                                   obj.pose.orientation.w);
    }

    ObjectData data;
    data.local_transform = swri_transform_util::Transform(
      tf::Transform(
        orientation,
        tf::Vector3(obj.pose.position.x,
                    obj.pose.position.y,
                    obj.pose.position.z)));
    data.source_frame = header.frame_id;
    data.id = obj.id;
    data.stamp = header.stamp;
    data.transformed = false;
    data.active = true;

    swri_transform_util::Transform transform;
    if (GetTransform(data.source_frame, data.stamp, transform))
    {
      data.transformed = true;
    }

    data.polygon.reserve(obj.polygon.size()+1);
    for (auto& point: obj.polygon)
    {
      tf::Vector3 pt(point.x, point.y, point.z);
      pt = data.local_transform*pt;
      tf::Vector3 transformed = transform *pt;
      data.polygon.push_back({pt, transformed});
    }
    if (data.polygon.size())
    {
      data.polygon.push_back(data.polygon.front());
    }
    objects_.push_back(data);
  }

  void ObjectPlugin::handleTrack(const marti_nav_msgs::TrackedObject &obj)
  {
    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }

    // Since orientation was not implemented, many markers publish
    // invalid all-zero orientations, so we need to check for this
    // and provide a default identity transform.
    tf::Quaternion orientation(0.0, 0.0, 0.0, 1.0);
    if (obj.pose.pose.orientation.x ||
        obj.pose.pose.orientation.y ||
        obj.pose.pose.orientation.z ||
        obj.pose.pose.orientation.w)
    {
      orientation = tf::Quaternion(obj.pose.pose.orientation.x,
                                   obj.pose.pose.orientation.y,
                                   obj.pose.pose.orientation.z,
                                   obj.pose.pose.orientation.w);
    }

    ObjectData data;
    data.local_transform = swri_transform_util::Transform(
      tf::Transform(
        orientation,
        tf::Vector3(obj.pose.pose.position.x,
                    obj.pose.pose.position.y,
                    obj.pose.pose.position.z)));
    data.stamp = obj.header.stamp;
    data.source_frame = obj.header.frame_id;
    data.id = std::to_string(obj.id);
    data.transformed = false;
    data.active = obj.active;

    swri_transform_util::Transform transform;
    if (GetTransform(data.source_frame, data.stamp, transform))
    {
      data.transformed = true;
    }

    data.polygon.reserve(obj.polygon.size()+1);
    for (auto& point: obj.polygon)
    {
      tf::Vector3 pt(point.x, point.y, point.z);
      pt = data.local_transform*pt;
      tf::Vector3 transformed = transform *pt;
      data.polygon.push_back({pt, transformed});
    }
    if (data.polygon.size())
    {
      data.polygon.push_back(data.polygon.front());
    }
    objects_.push_back(data);
  }

  void ObjectPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void ObjectPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void ObjectPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  QWidget* ObjectPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool ObjectPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;
    SetColor(ui_.color->color());

    return true;
  }

  void ObjectPlugin::Draw(double x, double y, double scale)
  {
    for (const auto& obj: objects_)
    {
      if (!obj.transformed)
      {
        continue;
      }

      if (!obj.active && !ui_.show_inactive->isChecked())
      {
        continue;
      }

      glColor4f(color_.redF(), color_.greenF(), color_.blueF(), 1.0);

      glLineWidth(3.0);
      glBegin(GL_LINE_STRIP);

      for (const auto &point : obj.polygon) {
        glVertex2d(
          point.transformed_point.getX(),
          point.transformed_point.getY());
      }

      glEnd();

      PrintInfo("OK");
    }
  }

  void ObjectPlugin::Paint(QPainter* painter, double x, double y, double scale)
  {
    if (!ui_.show_ids->isChecked())
    {
      return;
    }

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

    for (const auto& obj: objects_)
    {
      if (!obj.transformed)
      {
        continue;
      }

      QPen pen(QBrush(QColor::fromRgbF(0, 0, 0, 1)), 1);
      painter->setPen(pen);

      const StampedPoint& rosPoint = obj.polygon.front();
      QPointF point = tf.map(QPointF(rosPoint.transformed_point.x(),
                                     rosPoint.transformed_point.y()));

      auto text = QString::fromStdString(obj.id);
      // Get bounding rectangle
      QRectF rect(point, QSizeF(10,10));
      rect = painter->boundingRect(rect, Qt::AlignLeft | Qt::AlignHCenter, text);
      painter->drawText(rect, text);

      PrintInfo("OK");
    }

    painter->restore();
  }

  void ObjectPlugin::Transform()
  {
    for (auto& obj: objects_)
    {
      swri_transform_util::Transform transform;
      if (GetTransform(obj.source_frame, obj.stamp, transform))
      {
        obj.transformed = true;

        for (auto &point : obj.polygon)
        {
          point.transformed_point = transform * point.point;
        }
      }
      else
      {
        obj.transformed = false;
      }
    }
  }

  void ObjectPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["topic"])
    {
      std::string topic;
      node["topic"] >> topic;
      ui_.topic->setText(boost::trim_copy(topic).c_str());

      TopicEdited();
    }

    if (node["color"])
    {
      std::string color;
      node["color"] >> color;
      QColor qcolor(color.c_str());
      SetColor(qcolor);
      ui_.color->setColor(qcolor);
    }

    if (node["show_ids"])
    {
      bool checked;
      node["show_ids"] >> checked;
      ui_.show_ids->setChecked( checked );
    }

    if (node["show_inactive"])
    {
      bool checked;
      node["show_inactive"] >> checked;
      ui_.show_inactive->setChecked( checked );
    }
  }

  void ObjectPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "topic" << YAML::Value << boost::trim_copy(ui_.topic->text().toStdString());

    emitter << YAML::Key << "color" << YAML::Value
            << ui_.color->color().name().toStdString();

    emitter << YAML::Key << "show_ids" << YAML::Value << ui_.show_ids->isChecked();

    emitter << YAML::Key << "show_inactive" << YAML::Value << ui_.show_inactive->isChecked();
  }

  void ObjectPlugin::timerEvent(QTimerEvent *event)
  {
    bool new_connected = (object_sub_.getNumPublishers() > 0);
    if (connected_ && !new_connected)
    {
      object_sub_.shutdown();
      if (!topic_.empty())
      {
        object_sub_ = node_.subscribe<topic_tools::ShapeShifter>(
            topic_, 100, &ObjectPlugin::handleMessage, this);
      }
    }
    connected_ = new_connected;
  }
}

