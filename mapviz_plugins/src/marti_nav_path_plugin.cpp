// *****************************************************************************
//
// Copyright (c) 2021, Southwest Research Institute® (SwRI®)
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

#include <mapviz_plugins/marti_nav_path_plugin.h>

#include <string>
#include <vector>

#include <QObject>
#include <QColor>
#include <QColorDialog>
#include <QDialog>
#include <QPalette>
#include <QWidget>
#include <QGLWidget>

#include <ros/ros.h>
#include <ros/master.h>

#include <swri_yaml_util/yaml_util.h>

#include <mapviz/select_topic_dialog.h>

namespace mapviz_plugins
{
MartiNavPathPlugin::MartiNavPathPlugin()
  :
  config_widget_(new QWidget()),
  topic_("")
{
  ui_.setupUi(config_widget_);

  ui_.forward_color->setColor(QColor(0, 0, 255));
  ui_.reverse_color->setColor(QColor(255, 0, 0));

  historyChanged();

  // Set background white
  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Background, Qt::white);
  config_widget_->setPalette(p);

  QPalette p3(ui_.status->palette());
  p3.setColor(QPalette::Text, Qt::red);
  ui_.status->setPalette(p3);

  // Connect slots
  QObject::connect(ui_.select_topic, SIGNAL(clicked()),
                   this, SLOT(selectTopic()));
  QObject::connect(ui_.topic, SIGNAL(editingFinished()),
                   this, SLOT(topicEdited()));

  QObject::connect(ui_.history_size, SIGNAL(valueChanged(int)),
                   this, SLOT(historyChanged()));
}

bool MartiNavPathPlugin::Initialize(QGLWidget* canvas)
{
  canvas_ = canvas;
  return true;
}

void MartiNavPathPlugin::Shutdown()
{
}

void MartiNavPathPlugin::historyChanged()
{
  items_.set_capacity(ui_.history_size->value());
}

void MartiNavPathPlugin::setColorForDirection(const bool in_reverse)
{
  QColor color = in_reverse ? ui_.reverse_color->color() : ui_.forward_color->color();
  glColor3f(color.redF(), color.greenF(), color.blueF());
}

static inline
void drawArrow(const double x, const double y, const double yaw, const double L)
{
  double tip_x = x + std::cos(yaw)*L;
  double tip_y = y + std::sin(yaw)*L;

  double right_wing_x = tip_x + std::cos(yaw - M_PI*3.0/4.0)*L*0.25;
  double right_wing_y = tip_y + std::sin(yaw - M_PI*3.0/4.0)*L*0.25;

  double left_wing_x = tip_x + std::cos(yaw + M_PI*3.0/4.0)*L*0.25;
  double left_wing_y = tip_y + std::sin(yaw + M_PI*3.0/4.0)*L*0.25;

  glBegin(GL_LINES);

  glVertex2f(x,y);
  glVertex2f(tip_x,tip_y);

  glVertex2f(tip_x,tip_y);
  glVertex2f(right_wing_x, right_wing_y);

  glVertex2f(tip_x, tip_y);
  glVertex2f(left_wing_x, left_wing_y);

  glEnd();
}

void MartiNavPathPlugin::Draw(double /*x*/, double /*y*/, double /*scale*/)
{
  std::map<std::string, swri_transform_util::Transform> transforms;
  PrintInfo("Ok");

  for (size_t i = 0; i < items_.size(); i++)
  {
    const auto& item = items_[i];

    std::string src_frame = item.header.frame_id.length() ? item.header.frame_id : target_frame_;
    if (transforms.count(src_frame) == 0)
    {
      swri_transform_util::Transform transform;
      if (tf_manager_->GetTransform(target_frame_, src_frame, ros::Time(), transform))
      {
        transforms[src_frame] = transform;
      }
      else
      {
        ROS_ERROR("Failed to get transform from %s to %s", src_frame.c_str(), target_frame_.c_str());
        PrintError("Failed to get transform from " + src_frame + " to " + target_frame_);
        continue;
      }
    }

    const swri_transform_util::Transform &transform = transforms[src_frame];

    marti_nav_msgs::Path path = items_[i]; //mutable copy

    // transform the points
    for (auto& pt: path.points)
    {
      // transform position
      tf::Vector3 pt3(pt.x, pt.y, 0.0);
      pt3 = transform*pt3;
      pt.x = pt3.x();
      pt.y = pt3.y();

      // transform yaw
      tf::Quaternion q = tf::createQuaternionFromYaw(pt.yaw);
      q = transform*q;
      pt.yaw = tf::getYaw(q);
    }

    if (ui_.draw_lines->isChecked())
    {
      glLineWidth(ui_.line_width->value());

      setColorForDirection(path.in_reverse);

      glBegin(GL_LINE_STRIP);
      for (auto const &point : path.points)
      {
        glVertex2f(point.x, point.y);
      }
      glEnd();
    }
    if (ui_.draw_points->isChecked())
    {
      glPointSize(2*ui_.line_width->value() + 1);

      setColorForDirection(path.in_reverse);

      glBegin(GL_POINTS);
      for (auto const &point : path.points)
      {
        glVertex2f(point.x, point.y);
      }
      glEnd();
    }
    if (ui_.draw_yaw->isChecked())
    {
      glLineWidth(ui_.line_width->value());

      setColorForDirection(path.in_reverse);
      for (auto const &point : path.points)
      {
        drawArrow(point.x, point.y, point.yaw, ui_.arrow_length->value());
      }
    }
  }
}

void MartiNavPathPlugin::Transform()
{
}

void MartiNavPathPlugin::LoadConfig(
  const YAML::Node& node,
  const std::string& /*path*/)
{
  std::string topic;
  node["topic"] >> topic;
  ui_.topic->setText(topic.c_str());
  topicEdited();

  if (swri_yaml_util::FindValue(node, "draw_lines"))
  {
    bool val;
    node["draw_lines"] >> val;
    ui_.draw_lines->setChecked(val);
  }

  if (swri_yaml_util::FindValue(node, "draw_points"))
  {
    bool val;
    node["draw_points"] >> val;
    ui_.draw_points->setChecked(val);
  }

  if (swri_yaml_util::FindValue(node, "draw_yaw"))
  {
    bool val;
    node["draw_yaw"] >> val;
    ui_.draw_yaw->setChecked(val);
  }

  if (swri_yaml_util::FindValue(node, "line_width"))
  {
    double val;
    node["line_width"] >> val;
    ui_.line_width->setValue(val);
  }

  if (swri_yaml_util::FindValue(node, "arrow_length"))
  {
    double val;
    node["arrow_length"] >> val;
    ui_.arrow_length->setValue(val);
  }

  if (swri_yaml_util::FindValue(node, "history_size"))
  {
    int val;
    node["history_size"] >> val;
    ui_.arrow_length->setValue(val);
  }

  if (swri_yaml_util::FindValue(node, "forward_color"))
  {
    std::string color;
    node["forward_color"] >> color;
    ui_.forward_color->setColor(QColor(color.c_str()));
  }

  if (swri_yaml_util::FindValue(node, "reverse_color"))
  {
    std::string color;
    node["reverse_color"] >> color;
    ui_.reverse_color->setColor(QColor(color.c_str()));
  }
}

void MartiNavPathPlugin::SaveConfig(
  YAML::Emitter& emitter, const std::string& /*path*/)
{
  emitter << YAML::Key << "topic"
          << YAML::Value << ui_.topic->text().toStdString();

  emitter << YAML::Key << "draw_lines"
          << YAML::Value << ui_.draw_lines->isChecked();
  emitter << YAML::Key << "draw_points"
          << YAML::Value << ui_.draw_points->isChecked();
  emitter << YAML::Key << "draw_yaw"
          << YAML::Value << ui_.draw_yaw->isChecked();

  emitter << YAML::Key << "line_width"
          << YAML::Value << ui_.line_width->value();
  emitter << YAML::Key << "arrow_length"
          << YAML::Value << ui_.arrow_length->value();
  emitter << YAML::Key << "history_size"
          << YAML::Value << ui_.history_size->value();

  emitter << YAML::Key << "forward_color"
          << YAML::Value << ui_.forward_color->color().name().toStdString();
  emitter << YAML::Key << "reverse_color"
          << YAML::Value << ui_.reverse_color->color().name().toStdString();
}

QWidget* MartiNavPathPlugin::GetConfigWidget(QWidget* parent)
{
  config_widget_->setParent(parent);

  return config_widget_;
}

void MartiNavPathPlugin::PrintInfo(const std::string& message)
{
  if (message == ui_.status->text().toStdString())
  {
    return;
  }

  ROS_INFO("%s", message.c_str());
  QPalette p(ui_.status->palette());
  p.setColor(QPalette::Text, Qt::green);
  ui_.status->setPalette(p);
  ui_.status->setText(message.c_str());
}

void MartiNavPathPlugin::PrintWarning(const std::string& message)
{
  if (message == ui_.status->text().toStdString())
  {
    return;
  }

  ROS_WARN("%s", message.c_str());
  QPalette p(ui_.status->palette());
  p.setColor(QPalette::Text, Qt::darkYellow);
  ui_.status->setPalette(p);
  ui_.status->setText(message.c_str());
}

void MartiNavPathPlugin::PrintError(const std::string& message)
{
  if (message == ui_.status->text().toStdString())
  {
    return;
  }

  ROS_ERROR("Error: %s", message.c_str());
  QPalette p(ui_.status->palette());
  p.setColor(QPalette::Text, Qt::red);
  ui_.status->setPalette(p);
  ui_.status->setText(message.c_str());
}

void MartiNavPathPlugin::selectTopic()
{
  std::vector<std::string> supported_types;
  supported_types.push_back("marti_nav_msgs/Path");
  supported_types.push_back("marti_nav_msgs/PathPoint");

  ros::master::TopicInfo topic = mapviz::SelectTopicDialog::selectTopic(supported_types);

  if (!topic.name.empty())
  {
    ui_.topic->setText(QString::fromStdString(topic.name));
    topicEdited();
  }
}

void MartiNavPathPlugin::topicEdited()
{
  if (ui_.topic->text().toStdString() != topic_)
  {
    initialized_ = true;
    items_.clear();
    topic_ = ui_.topic->text().toStdString();

    subscriber_ = node_.subscribe<topic_tools::ShapeShifter>(
        topic_, 100, &MartiNavPathPlugin::messageCallback, this);

    ROS_INFO("Subscribing to %s", topic_.c_str());
    PrintWarning("No messages received.");
  }
}

void MartiNavPathPlugin::messageCallback(
  const topic_tools::ShapeShifter::ConstPtr& msg)
{
#define IS_INSTANCE(msg, type) (msg->getDataType() == ros::message_traits::datatype<type>())

  if (IS_INSTANCE(msg, marti_nav_msgs::Path))
  {
    marti_nav_msgs::PathConstPtr msg_typed = msg->instantiate<marti_nav_msgs::Path>();
    handlePath(*msg_typed);
  }
  else if (IS_INSTANCE(msg, marti_nav_msgs::PathPoint))
  {
    marti_nav_msgs::PathPointConstPtr msg_typed = msg->instantiate<marti_nav_msgs::PathPoint>();
    handlePathPoint(*msg_typed);
  }
  else
  {
    ROS_ERROR_THROTTLE(1.0, "Unknown message type: %s", msg->getDataType().c_str());
    return;
  }

  canvas_->update();

#undef IS_INSTANCE
}

void MartiNavPathPlugin::handlePath(
  const marti_nav_msgs::Path &path)
{
  items_.push_back(path);
}

void MartiNavPathPlugin::handlePathPoint(
  const marti_nav_msgs::PathPoint &point)
{
  marti_nav_msgs::Path segment;
  segment.in_reverse = false;// no inherent direction for a point, just pick one
  segment.points.push_back(point);
  handlePath(segment);
}
}  // namespace mapviz_plugins

// Register nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( //NOLINT
  mapviz_plugins::MartiNavPathPlugin,
  mapviz::MapvizPlugin)
