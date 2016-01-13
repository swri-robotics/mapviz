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
AttitudeIndicatorPlugin::AttitudeIndicatorPlugin() :
  config_widget_(new QWidget())
  // anchor_(TOP_LEFT),
  // units_(PIXELS),
  // offset_x_(0),
  // offset_y_(0),
  // width_(320),
  // height_(240),
  // has_image_(false),
  // last_width_(0),
  // last_height_(0)
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

  placer_.setRect(QRect(0, 0, 100, 100));  
  QObject::connect(this, SIGNAL(VisibleChanged(bool)),
                   &placer_, SLOT(setVisible(bool)));
  
  // QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
  // QObject::connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
  // QObject::connect(ui_.anchor, SIGNAL(activated(QString)), this, SLOT(SetAnchor(QString)));
  // QObject::connect(ui_.units, SIGNAL(activated(QString)), this, SLOT(SetUnits(QString)));
  // QObject::connect(ui_.offsetx, SIGNAL(valueChanged(int)), this, SLOT(SetOffsetX(int)));
  // QObject::connect(ui_.offsety, SIGNAL(valueChanged(int)), this, SLOT(SetOffsetY(int)));
  // QObject::connect(ui_.width, SIGNAL(valueChanged(int)), this, SLOT(SetWidth(int)));
  // QObject::connect(ui_.height, SIGNAL(valueChanged(int)), this, SLOT(SetHeight(int)));

}

AttitudeIndicatorPlugin::~AttitudeIndicatorPlugin()
{
}

// void AttitudeIndicatorPlugin::SetOffsetX(int offset)
// {
//   offset_x_ = offset;
//   canvas_->update();
// }

// void AttitudeIndicatorPlugin::SetOffsetY(int offset)
// {
//   offset_y_ = offset;
//   canvas_->update();
// }

// void AttitudeIndicatorPlugin::SetWidth(int width)
// {
//   width_ = width;

//   canvas_->update();
// }

// void AttitudeIndicatorPlugin::SetHeight(int height)
// {
//   height_ = height;

//   canvas_->update();
// }

// void AttitudeIndicatorPlugin::SetAnchor(QString anchor)
// {
//   if (anchor == "top left")
//   {
//     anchor_ = TOP_LEFT;
//   }
//   else if (anchor == "top center")
//   {
//     anchor_ = TOP_CENTER;
//   }
//   else if (anchor == "top right")
//   {
//     anchor_ = TOP_RIGHT;
//   }
//   else if (anchor == "center left")
//   {
//     anchor_ = CENTER_LEFT;
//   }
//   else if (anchor == "center")
//   {
//     anchor_ = CENTER;
//   }
//   else if (anchor == "center right")
//   {
//     anchor_ = CENTER_RIGHT;
//   }
//   else if (anchor == "bottom left")
//   {
//     anchor_ = BOTTOM_LEFT;
//   }
//   else if (anchor == "bottom center")
//   {
//     anchor_ = BOTTOM_CENTER;
//   }
//   else if (anchor == "bottom right")
//   {
//     anchor_ = BOTTOM_RIGHT;
//   }

//   canvas_->update();
// }

// void AttitudeIndicatorPlugin::SetUnits(QString units)
// {
//   if (units == "pixels")
//   {
//     units_ = PIXELS;
//   }
//   else if (units == "percent")
//   {
//     units_ = PERCENT;
//   }

//   canvas_->update();
// }

// void AttitudeIndicatorPlugin::SelectTopic()
// {
//   ros::master::TopicInfo topic = mapviz::SelectTopicDialog::selectTopic(
//     "stereo_msgs/AttitudeIndicatorImage");

//   if (!topic.name.empty())
//   {
//     ui_.topic->setText(QString::fromStdString(topic.name));
//     TopicEdited();
//   }
// }

// void AttitudeIndicatorPlugin::TopicEdited()
// {
//   if (ui_.topic->text().toStdString() != topic_)
//   {
//     initialized_ = false;
//     has_message_ = false;
//     topic_ = ui_.topic->text().toStdString();
//     PrintWarning("No messages received.");

//     disparity_sub_.shutdown();
//     disparity_sub_ = node_.subscribe(topic_, 1, &AttitudeIndicatorPlugin::disparityCallback, this);

//     ROS_INFO("Subscribing to %s", topic_.c_str());
//   }
// }

// void AttitudeIndicatorPlugin::disparityCallback(const stereo_msgs::AttitudeIndicatorImageConstPtr disparity)
// {
//   if (!has_message_)
//   {
//     initialized_ = true;
//     has_message_ = true;
//   }

//   if (disparity->min_disparity == 0.0 && disparity->max_disparity == 0.0)
//   {
//     PrintError("Min and max disparity not set.");
//     has_image_ = false;
//     return;
//   }

//   if (disparity->image.encoding != sensor_msgs::image_encodings::TYPE_32FC1)
//   {
//     PrintError("Invalid encoding.");
//     has_image_ = false;
//     return;
//   }

//   disparity_ = *disparity;

//   // Colormap and display the disparity image
//   float min_disparity = disparity->min_disparity;
//   float max_disparity = disparity->max_disparity;
//   float multiplier = 255.0f / (max_disparity - min_disparity);

//   cv_bridge::CvImageConstPtr cv_disparity = 
//     cv_bridge::toCvShare(disparity->image, disparity);

//   disparity_color_.create(disparity->image.height, disparity->image.width);

//   for (int row = 0; row < disparity_color_.rows; row++)
//   {
//     const float* d = cv_disparity->image.ptr<float>(row);
//     for (int col = 0; col < disparity_color_.cols; col++)
//     {
//       int index = (d[col] - min_disparity) * multiplier + 0.5;
//       index = std::min(255, std::max(0, index));
//       // Fill as BGR
//       disparity_color_(row, col)[2] = colormap[3*index + 0];
//       disparity_color_(row, col)[1] = colormap[3*index + 1];
//       disparity_color_(row, col)[0] = colormap[3*index + 2];
//     }
//   }

//   last_width_ = 0;
//   last_height_ = 0;

//   has_image_ = true;

//   canvas_->update();
// }

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

void AttitudeIndicatorPlugin::Draw(double x, double y, double scale)
{
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, canvas_->width(), canvas_->height(), 0, -1.0f, 1.0f);

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
  //drawIndicator();
  drawBackground();
  // drawInnerShell();
  drawPanel();
  
  glPopMatrix();
  glPopAttrib();
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
  glVertex2f(0.0, 0.9);
  glVertex2f(0.0, 0.2);
  glEnd();
}

void AttitudeIndicatorPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
{ 
//   std::string topic;
//   node["topic"] >> topic;
//   ui_.topic->setText(topic.c_str());

//   TopicEdited();

//   std::string anchor;
//   node["anchor"] >> anchor;
//   ui_.anchor->setCurrentIndex(ui_.anchor->findText(anchor.c_str()));
//   SetAnchor(anchor.c_str());

//   std::string units;
//   node["units"] >> units;
//   ui_.units->setCurrentIndex(ui_.units->findText(units.c_str()));
//   SetUnits(units.c_str());

//   node["offset_x"] >> offset_x_;
//   ui_.offsetx->setValue(offset_x_);

//   node["offset_y"] >> offset_y_;
//   ui_.offsety->setValue(offset_y_);

//   node["width"] >> width_;
//   ui_.width->setValue(width_);

//   node["height"] >> height_;
//   ui_.height->setValue(height_);
}

void AttitudeIndicatorPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
{
//   emitter << YAML::Key << "topic" << YAML::Value << ui_.topic->text().toStdString();
//   emitter << YAML::Key << "anchor" << YAML::Value << AnchorToString(anchor_);
//   emitter << YAML::Key << "units" << YAML::Value << UnitsToString(units_);
//   emitter << YAML::Key << "offset_x" << YAML::Value << offset_x_;
//   emitter << YAML::Key << "offset_y" << YAML::Value << offset_y_;
//   emitter << YAML::Key << "width" << YAML::Value << width_;
//   emitter << YAML::Key << "height" << YAML::Value << height_;
}

// std::string AttitudeIndicatorPlugin::AnchorToString(Anchor anchor)
// {
//   std::string anchor_string = "top left";

//   if (anchor == TOP_LEFT)
//   {
//     anchor_string = "top left";
//   }
//   else if (anchor == TOP_CENTER)
//   {
//     anchor_string = "top center";
//   }
//   else if (anchor == TOP_RIGHT)
//   {
//     anchor_string = "top right";
//   }
//   else if (anchor == CENTER_LEFT)
//   {
//     anchor_string = "center left";
//   }
//   else if (anchor == CENTER)
//   {
//     anchor_string = "center";
//   }
//   else if (anchor == CENTER_RIGHT)
//   {
//     anchor_string = "center right";
//   }
//   else if (anchor == BOTTOM_LEFT)
//   {
//     anchor_string = "bottom left";
//   }
//   else if (anchor == BOTTOM_CENTER)
//   {
//     anchor_string = "bottom center";
//   }
//   else if (anchor == BOTTOM_RIGHT)
//   {
//     anchor_string = "bottom right";
//   }

//   return anchor_string;
// }

// std::string AttitudeIndicatorPlugin::UnitsToString(Units units)
// {
//   std::string units_string = "pixels";

//   if (units == PIXELS)
//   {
//     units_string = "pixels";
//   }
//   else if (units == PERCENT)
//   {
//     units_string = "percent";
//   }

//   return units_string;
// }
}  // namespace mapviz_plugins
