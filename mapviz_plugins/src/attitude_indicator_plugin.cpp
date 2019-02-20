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

#include <mapviz_plugins/attitude_indicator_plugin.h>
#include <GL/glut.h>

// C++ standard libraries
#include <algorithm>
#include <cstdio>
#include <vector>

// QT libraries
#include <QDebug>
#include <QDialog>
#include <QGLWidget>

// ROS libraries
#include <ros/master.h>

#include <mapviz/select_topic_dialog.h>
#include <mapviz/select_frame_dialog.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::AttitudeIndicatorPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
#define IS_INSTANCE(msg, type) \
  (msg->getDataType() == ros::message_traits::datatype<type>())

  AttitudeIndicatorPlugin::AttitudeIndicatorPlugin() :
      config_widget_(new QWidget())
  {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);
    roll_ = pitch_ = yaw_ = 0;
    topics_.push_back("nav_msgs/Odometry");
    topics_.push_back("geometry_msgs/Pose");
    topics_.push_back("sensor_msgs/Imu");
    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    placer_.setRect(QRect(0, 0, 100, 100));
    QObject::connect(this, SIGNAL(VisibleChanged(bool)),
                     &placer_, SLOT(setVisible(bool)));

    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
  }

  AttitudeIndicatorPlugin::~AttitudeIndicatorPlugin()
  {
  }

  void AttitudeIndicatorPlugin::SelectTopic()
  {
    ros::master::TopicInfo topic = mapviz::SelectTopicDialog::selectTopic(
        topics_);
    if (topic.name.empty())
    {
      return;
    }

    ui_.topic->setText(QString::fromStdString(topic.name));
    TopicEdited();
  }

  void AttitudeIndicatorPlugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();
    if (topic != topic_)
    {
      initialized_ = true;
      PrintWarning("No messages received.");

      odometry_sub_.shutdown();
      topic_ = topic;
      if (!topic_.empty())
      {
        odometry_sub_ = node_.subscribe<topic_tools::ShapeShifter>(
            topic_, 100, &AttitudeIndicatorPlugin::handleMessage, this);

        ROS_INFO("Subscribing to %s", topic_.c_str());
      }
    }
  }

  void AttitudeIndicatorPlugin::handleMessage(const topic_tools::ShapeShifter::ConstPtr& msg)
  {
    if (IS_INSTANCE(msg, nav_msgs::Odometry))
    {
      AttitudeCallbackOdom(msg->instantiate<nav_msgs::Odometry>());
    }
    else if (IS_INSTANCE(msg, sensor_msgs::Imu))
    {
      AttitudeCallbackImu(msg->instantiate<sensor_msgs::Imu>());
    }
    else if (IS_INSTANCE(msg, geometry_msgs::Pose))
    {
      AttitudeCallbackPose(msg->instantiate<geometry_msgs::Pose>());
    }
    else
    {
      PrintError("Unknown message type: " + msg->getDataType());
    }
  }

  void AttitudeIndicatorPlugin::AttitudeCallbackOdom(const nav_msgs::OdometryConstPtr& odometry)
  {
    applyAttitudeOrientation(odometry->pose.pose.orientation);
  }

  void AttitudeIndicatorPlugin::AttitudeCallbackImu(const sensor_msgs::ImuConstPtr& imu)
  {
    applyAttitudeOrientation(imu->orientation);
  }

  void AttitudeIndicatorPlugin::AttitudeCallbackPose(const geometry_msgs::PoseConstPtr& pose)
  {
    applyAttitudeOrientation(pose->orientation);
  }

  void AttitudeIndicatorPlugin::applyAttitudeOrientation(const geometry_msgs::Quaternion &orientation)
  {
    tf::Quaternion attitude_orientation(
      orientation.x,
      orientation.y,
      orientation.z,
      orientation.w);

    tf::Matrix3x3 m(attitude_orientation);
    m.getRPY(roll_, pitch_, yaw_);
    roll_ = roll_ * (180.0 / M_PI);
    pitch_ = pitch_ * (180.0 / M_PI);
    yaw_ = yaw_ * (180.0 / M_PI);

    canvas_->update();
  }

  void AttitudeIndicatorPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void AttitudeIndicatorPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void AttitudeIndicatorPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  QWidget* AttitudeIndicatorPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);
    return config_widget_;
  }

  bool AttitudeIndicatorPlugin::Initialize(QGLWidget* canvas)
  {
    initialized_ = true;
    canvas_ = canvas;
    placer_.setContainer(canvas_);
    startTimer(50);
    return true;
  }

  void AttitudeIndicatorPlugin::Shutdown()
  {
    placer_.setContainer(NULL);
  }

  void AttitudeIndicatorPlugin::timerEvent(QTimerEvent*)
  {
    canvas_->update();
  }

  void AttitudeIndicatorPlugin::drawBall()
  {
    GLdouble eqn[4] = {0.0, 0.0, 1.0, 0.0};
    GLdouble eqn2[4] = {0.0, 0.0, -1.0, 0.0};
    GLdouble eqn4[4] = {0.0, 0.0, 1.0, 0.05};
    GLdouble eqn3[4] = {0.0, 0.0, -1.0, 0.05};

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glPushMatrix();

    glColor3f(0.392156863f, 0.584313725f, 0.929411765f);
    glRotated(90.0 + pitch_, 1.0, 0.0, 0.0);

    glRotated(roll_, 0.0, 1.0, 0.0);
    glRotated(yaw_, 0.0, 0.0, 1.0);
    glClipPlane(GL_CLIP_PLANE1, eqn2);
    glEnable(GL_CLIP_PLANE1);
    glutSolidSphere(.8, 20, 16);
    glDisable(GL_CLIP_PLANE1);
    glPopMatrix();

    glPushMatrix();

    glLineWidth(2);
    glColor3f(1.0f, 1.0f, 1.0f);
    glRotated(90.0 + pitch_, 1.0, 0.0, 0.0);
    glRotated(roll_, 0.0, 1.0, 0.0);
    glRotated(yaw_, 0.0, 0.0, 1.0);
    glClipPlane(GL_CLIP_PLANE3, eqn4);
    glClipPlane(GL_CLIP_PLANE2, eqn3);
    glEnable(GL_CLIP_PLANE2);
    glEnable(GL_CLIP_PLANE3);
    glutWireSphere(.801, 10, 16);
    glDisable(GL_CLIP_PLANE2);
    glDisable(GL_CLIP_PLANE3);
    glPopMatrix();

    glPushMatrix();
    glColor3f(0.62745098f, 0.321568627f, 0.176470588f);
    glRotated(90.0 + pitch_, 1.0, 0.0, 0.0);//x
    glRotated(roll_, 0.0, 1.0, 0.0);//y
    glRotated(yaw_, 0.0, 0.0, 1.0);//z
    glClipPlane(GL_CLIP_PLANE0, eqn);
    glEnable(GL_CLIP_PLANE0);
    glutSolidSphere(.8, 20, 16);
    glDisable(GL_CLIP_PLANE0);
    glPopMatrix();
    glDisable(GL_DEPTH_TEST);
  }

  void AttitudeIndicatorPlugin::Draw(double x, double y, double scale)
  {
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, canvas_->width(), canvas_->height(), 0, -1.0f, 1.0f);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    // Setup coordinate system so that we have a [-1,1]x[1,1] cube on
    // the screen.
    QRect rect = placer_.rect();
    double s_x = rect.width() / 2.0;
    double s_y = -rect.height() / 2.0;
    double t_x = rect.right() - s_x;
    double t_y = rect.top() - s_y;

    double m[16] = {
        s_x, 0, 0, 0,
        0, s_y, 0, 0,
        0, 0, 1.0, 0,
        t_x, t_y, 0, 1.0};
    glMultMatrixd(m);

    // Placed in a separate function so that we don't forget to pop the
    // GL state back.

    drawBackground();
    drawBall();

    drawPanel();

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopAttrib();
    PrintInfo("OK!");
  }

  void AttitudeIndicatorPlugin::drawBackground()
  {
    glBegin(GL_TRIANGLES);
    glColor4f(0.0f, 0.0f, 0.0f, 1.0f);

    glVertex2d(-1.0, -1.0);
    glVertex2d(-1.0, 1.0);
    glVertex2d(1.0, 1.0);

    glVertex2d(-1.0, -1.0);
    glVertex2d(1.0, 1.0);
    glVertex2d(1.0, -1.0);

    glEnd();
  }

  void AttitudeIndicatorPlugin::drawPanel()
  {
    glLineWidth(2);

    glBegin(GL_LINE_STRIP);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

    glVertex2d(-0.9, 0.0);
    glVertex2d(-0.2, 0.0);

    int divisions = 20;
    for (int i = 1; i < divisions; i++)
    {
      glVertex2d(-0.2 * std::cos(M_PI * i / divisions),
                 -0.2 * std::sin(M_PI * i / divisions));
    }

    glVertex2f(0.2, 0.0);
    glVertex2f(0.9, 0.0);
    glEnd();

    glBegin(GL_LINES);
    glVertex2f(0.0, -0.2f);
    glVertex2f(0.0, -0.9f);
    glEnd();
  }

  void AttitudeIndicatorPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["topic"])
    {
      std::string topic;
      node["topic"] >> topic;
      ui_.topic->setText(topic.c_str());
    }

    QRect current = placer_.rect();
    int x = current.x();
    int y = current.y();
    int width = current.width();
    int height = current.height();

    if (swri_yaml_util::FindValue(node, "x"))
    {
      node["x"] >> x;
    }

    if (swri_yaml_util::FindValue(node, "y"))
    {
      node["y"] >> y;
    }

    if (swri_yaml_util::FindValue(node, "width"))
    {
      node["width"] >> width;
    }

    if (swri_yaml_util::FindValue(node, "height"))
    {
      node["height"] >> height;
    }

    QRect position(x, y, width, height);
    placer_.setRect(position);

    TopicEdited();
  }

  void AttitudeIndicatorPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "topic" << YAML::Value << ui_.topic->text().toStdString();

    QRect position = placer_.rect();

    emitter << YAML::Key << "x" << YAML::Value << position.x();
    emitter << YAML::Key << "y" << YAML::Value << position.y();
    emitter << YAML::Key << "width" << YAML::Value << position.width();
    emitter << YAML::Key << "height" << YAML::Value << position.height();
  }
}
