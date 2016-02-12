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

// C++ standard libraries
#include <cstdio>
#include <algorithm>
#include <vector>

// QT libraries
#include <QDialog>
#include <QGLWidget>
#include <QDebug>

// ROS libraries
#include <ros/master.h>

#include <mapviz/select_topic_dialog.h>
#include <mapviz/select_frame_dialog.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
  mapviz_plugins,
  attitude_indicator,
  mapviz_plugins::AttitudeIndicatorPlugin,
  mapviz::MapvizPlugin)

namespace mapviz_plugins
{
#define IS_INSTANCE(msg, type)                                  \
  (msg->getDataType() == ros::message_traits::datatype<type>())

AttitudeIndicatorPlugin::AttitudeIndicatorPlugin() :
  config_widget_(new QWidget())
{
  ui_.setupUi(config_widget_);

  // Set background white
  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Background, Qt::white);
  config_widget_->setPalette(p);
  roll=pitch=yaw=0;
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
     if (ui_.topic->text().toStdString() != topic_)
     {
       initialized_ = true;
       has_message_ = false;
       topic_ = boost::trim_copy(ui_.topic->text().toStdString());
       PrintWarning("No messages received.");

       odometry_sub_.shutdown();
       odometry_sub_ = node_.subscribe<topic_tools::ShapeShifter>(
         topic_, 100, &AttitudeIndicatorPlugin::handleMessage, this);

       ROS_INFO("Subscribing to %s", topic_.c_str());
     }
 }
 void AttitudeIndicatorPlugin::handleMessage(const topic_tools::ShapeShifter::ConstPtr& msg)
 {

   if (IS_INSTANCE(msg, nav_msgs::Odometry)){
     AttitudeCallbackOdom(*(msg->instantiate<nav_msgs::Odometry>()));
   }else if (IS_INSTANCE(msg, sensor_msgs::Imu)){
      AttitudeCallbackImu(*(msg->instantiate<sensor_msgs::Imu>()));
    }else if (IS_INSTANCE(msg, geometry_msgs::Pose)){
       AttitudeCallbackPose(*(msg->instantiate<geometry_msgs::Pose>()));
      }else {
     PrintError("Unknown message type: " + msg->getDataType());
   }
 }


 void AttitudeIndicatorPlugin::AttitudeCallbackOdom(const nav_msgs::Odometry &odometry)
 {

    point_.stamp = odometry.header.stamp;

    point_.point = tf::Point(
              odometry.pose.pose.position.x,
              odometry.pose.pose.position.y,
              odometry.pose.pose.position.z);


    point_.orientation = tf::Quaternion(
              odometry.pose.pose.orientation.x,
              odometry.pose.pose.orientation.y,
              odometry.pose.pose.orientation.z,
              odometry.pose.pose.orientation.w);

    tf::Matrix3x3 m(point_.orientation);
    m.getRPY(roll,pitch,yaw);
    roll=roll*(180.0/M_PI);
    pitch=pitch*(180.0/M_PI);
    yaw=yaw*(180.0/M_PI);

  ROS_INFO("roll %f,pitch %f, yaw %f",roll,pitch,yaw);
    canvas_->update();
 }
 void AttitudeIndicatorPlugin::AttitudeCallbackImu(const sensor_msgs::Imu &Imu)
 {
    point_.stamp = Imu.header.stamp;

    point_.point = tf::Point(0,0,0);


    point_.orientation = tf::Quaternion(
              Imu.orientation.x,
              Imu.orientation.y,
              Imu.orientation.z,
              Imu.orientation.w);

    tf::Matrix3x3 m(point_.orientation);
    m.getRPY(roll,pitch,yaw);
    roll=roll*(180.0/M_PI);
    pitch=pitch*(180.0/M_PI);
    yaw=yaw*(180.0/M_PI);
  ROS_INFO("roll %f,pitch %f, yaw %f",roll,pitch,yaw);

    canvas_->update();
 }
 void AttitudeIndicatorPlugin::AttitudeCallbackPose(const geometry_msgs::Pose &pose)
 {


   point_.point = tf::Point(
              pose.position.x,
              pose.position.y,
              pose.position.z);


    point_.orientation = tf::Quaternion(
              pose.orientation.x,
              pose.orientation.y,
              pose.orientation.z,
              pose.orientation.w);

    tf::Matrix3x3 m(point_.orientation);
    m.getRPY(roll,pitch,yaw);
    roll=roll*(180.0/M_PI);
    pitch=pitch*(180.0/M_PI);
    yaw=yaw*(180.0/M_PI);

  ROS_INFO("roll %f,pitch %f, yaw %f",roll,pitch,yaw);
    canvas_->update();
 }
void AttitudeIndicatorPlugin::PrintError(const std::string& message)
{
  if (message == ui_.status->text().toStdString())
    return;

  ROS_ERROR("Error: %s", message.c_str());
  QPalette p(ui_.status->palette());
  p.setColor(QPalette::Text, Qt::red);
  ui_.status->setPalette(p);
  ui_.status->setText(message.c_str());
}

void AttitudeIndicatorPlugin::PrintInfo(const std::string& message)
{
  if (message == ui_.status->text().toStdString())
    return;

  ROS_INFO("%s", message.c_str());
  QPalette p(ui_.status->palette());
  p.setColor(QPalette::Text, Qt::green);
  ui_.status->setPalette(p);
  ui_.status->setText(message.c_str());
}

void AttitudeIndicatorPlugin::PrintWarning(const std::string& message)
{
  if (message == ui_.status->text().toStdString())
    return;

  ROS_WARN("%s", message.c_str());
  QPalette p(ui_.status->palette());
  p.setColor(QPalette::Text, Qt::darkYellow);
  ui_.status->setPalette(p);
  ui_.status->setText(message.c_str());
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

void AttitudeIndicatorPlugin::timerEvent(QTimerEvent *)
{
  canvas_->update();
}

void AttitudeIndicatorPlugin::drawBall()
{
  GLdouble eqn[4]={0.0,0.0,1.0,0.0};
  GLdouble eqn2[4]={0.0,0.0,-1.0,0.0};
  GLdouble eqn4[4]={0.0,0.0,1.0,0.05};
  GLdouble eqn3[4]={0.0,0.0,-1.0,0.05};




  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  glPushMatrix();


  glColor3f(0.392156863f, 0.584313725f,0.929411765f);
  glRotatef(90.0+pitch, 1.0, 0.0, 0.0);


  glRotatef(roll, 0.0, 1.0, 0.0);
  glRotatef(yaw, 0.0, 0.0, 1.0);
  glClipPlane(GL_CLIP_PLANE1,eqn2);
  glEnable(GL_CLIP_PLANE1);
  glutSolidSphere(.8, 20 , 16);
  glDisable(GL_CLIP_PLANE1);
  glPopMatrix();


  glPushMatrix();

  glLineWidth(2);
  glColor3f(1.0f,1.0f,1.0f);
  glRotatef(90.0+pitch, 1.0, 0.0, 0.0);
  glRotatef(roll, 0.0, 1.0, 0.0);
  glRotatef(yaw, 0.0, 0.0, 1.0);
  glClipPlane(GL_CLIP_PLANE3,eqn4);
  glClipPlane(GL_CLIP_PLANE2,eqn3);
  glEnable(GL_CLIP_PLANE2);
  glEnable(GL_CLIP_PLANE3);
  glutWireSphere(.801, 10 ,16);
  glDisable(GL_CLIP_PLANE2);
  glDisable(GL_CLIP_PLANE3);
  glPopMatrix();



  glPushMatrix();
  glColor3f(0.62745098f,0.321568627f,0.176470588f);
  glRotatef(90.0+pitch, 1.0, 0.0, 0.0);//x
  glRotatef(roll, 0.0, 1.0, 0.0);//y
  glRotatef(yaw, 0.0, 0.0, 1.0);//z
  glClipPlane(GL_CLIP_PLANE0,eqn);
  glEnable(GL_CLIP_PLANE0);
  glutSolidSphere(.8, 20 , 16);
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
  
  double m[16] = {s_x, 0, 0, 0,
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
  
  glVertex2f(-1.0, -1.0);
  glVertex2f(-1.0,  1.0);
  glVertex2f( 1.0,  1.0);

  glVertex2f(-1.0, -1.0);
  glVertex2f( 1.0,  1.0);
  glVertex2f( 1.0, -1.0);

  glEnd();
}

void AttitudeIndicatorPlugin::drawPanel()
{  
  glLineWidth(2);
    
  glBegin(GL_LINE_STRIP);
  glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

  glVertex2f(-0.9, 0.0);
  glVertex2f(-0.2, 0.0);

  int divisions = 20;
  for (int i = 1; i < divisions; i++) {
    glVertex2f(-0.2*std::cos(M_PI * i / divisions),
               -0.2*std::sin(M_PI * i / divisions));
  }
  
  glVertex2f(0.2, 0.0);
  glVertex2f(0.9, 0.0);
  glEnd();

  glBegin(GL_LINES);
  glVertex2f(0.0,-0.2);
  glVertex2f(0.0,-0.9);
  glEnd();
}

void AttitudeIndicatorPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
{ 
   std::string topic;
   node["topic"] >> topic;
   ui_.topic->setText(topic.c_str());

   TopicEdited();


}

void AttitudeIndicatorPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
{
     emitter << YAML::Key << "topic" << YAML::Value << ui_.topic->text().toStdString();

}


}
