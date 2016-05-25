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

#include <mapviz_plugins/laps_plugin.h>

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
PLUGINLIB_DECLARE_CLASS(
    mapviz_plugins,
    laps,
    mapviz_plugins::LapsPlugin,
    mapviz::MapvizPlugin);

namespace mapviz_plugins
{
  LapsPlugin::LapsPlugin() :
    config_widget_(new QWidget()),
    draw_style_(LINES),
    new_lap_(true),
    position_tolerance_(0.0)
  {
    ui_.setupUi(config_widget_);

    ui_.color->setColor(Qt::green);
    
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
    QObject::connect(ui_.drawstyle, SIGNAL(activated(QString)), this, SLOT(SetDrawStyle(QString)));
    connect(ui_.color, SIGNAL(colorEdited(const QColor &)),
            this, SLOT(DrawIcon()));
  }

  LapsPlugin::~LapsPlugin()
  {
  }

  void LapsPlugin::DrawIcon()
  {
    if (icon_)
    {
      QPixmap icon(16, 16);
      icon.fill(Qt::transparent);
      
      QPainter painter(&icon);
      painter.setRenderHint(QPainter::Antialiasing, true);
      
      QPen pen(ui_.color->color());
      
      if (draw_style_ == LINES)
      {
        pen.setWidth(3);
        pen.setCapStyle(Qt::FlatCap);
        painter.setPen(pen);
        painter.drawLine(1, 14, 14, 1);
      }
      else if (draw_style_ == ARROWS)
      {
        pen.setWidth(2);
        pen.setCapStyle(Qt::SquareCap);
        painter.setPen(pen);
        painter.drawLine(2, 13, 13, 2);
        painter.drawLine(13, 2, 13, 8);
        painter.drawLine(13, 2, 7, 2);
      }
      
      icon_->SetPixmap(icon);
    }
  }

  void LapsPlugin::SetDrawStyle(QString style)
  {
    if (style == "lines")
    {
      draw_style_ = LINES;
    }
    else if (style == "arrows")
    {
      draw_style_ = ARROWS;
    }

    DrawIcon();
  }

  void LapsPlugin::SelectTopic()
  {
    ros::master::TopicInfo topic = mapviz::SelectTopicDialog::selectTopic(
      "nav_msgs/Odometry");
    
    if (!topic.name.empty())
    {
      ui_.topic->setText(QString::fromStdString(topic.name));
      TopicEdited();
    }
  }

  void LapsPlugin::TopicEdited()
  {
    if (ui_.topic->text().toStdString() != topic_)
    {
      initialized_ = false;
      points_.clear();
      has_message_ = false;
      topic_ = ui_.topic->text().toStdString();
      PrintWarning("No messages received.");

      odometry_sub_.shutdown();
      odometry_sub_ = node_.subscribe(topic_, 1, &LapsPlugin::odometryCallback, this);

      ROS_INFO("Subscribing to %s", topic_.c_str());
    }
  }

  void LapsPlugin::odometryCallback(const nav_msgs::OdometryConstPtr odometry)
  {
    if (!has_message_)
    {
      ROS_INFO("Got a new message");
      initialized_ = true;
      has_message_ = true;
      begin_= tf::Point(
                  odometry->pose.pose.position.x,
                  odometry->pose.pose.position.y,
                  odometry->pose.pose.position.z);
     PrintInfo("OK");
    }

    // Note that unlike some plugins, this one does not store nor rely on the
    // source_frame_ member variable.  This one can potentially store many
    // messages with different source frames, so we need to store and transform
    // them individually.

    StampedPoint stamped_point;
    stamped_point.stamp = odometry->header.stamp;
    stamped_point.source_frame = odometry->header.frame_id;

    stamped_point.point = tf::Point(
        odometry->pose.pose.position.x,
        odometry->pose.pose.position.y,
        odometry->pose.pose.position.z);

    stamped_point.orientation = tf::Quaternion(
        odometry->pose.pose.orientation.x,
        odometry->pose.pose.orientation.y,
        odometry->pose.pose.orientation.z,
        odometry->pose.pose.orientation.w);
    tf::Point check = begin_- stamped_point.point;



    if (points_.empty() || stamped_point.point.distance(points_.back().point) >= position_tolerance_)
    {
      points_.push_back(stamped_point);
    }



    cur_point_ = stamped_point;
    if(((std::fabs(check.x())<=3)&&(std::fabs(check.y())<=3)) && (new_lap_==false))
    {

        new_lap_=true;

        laps_.push_back(points_);
        laps_[0].pop_back();
        points_.clear();
        points_.push_back(stamped_point);
    }

    if(((std::fabs(check.x())>25)&&(std::fabs(check.y())>25)) && (new_lap_==true))
    {
        new_lap_=false;
    }


  }



  void LapsPlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_ERROR("Error: %s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void LapsPlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_INFO("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void LapsPlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_WARN("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  QWidget* LapsPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool LapsPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;
    
    DrawIcon();

    return true;
  }

  void LapsPlugin::Draw(double x, double y, double scale)
  {
    if(draw_style_==LINES)
    {
     DrawLaps();
    }
    else if(draw_style_==ARROWS)
    {
     DrawArrows();
    }


  }
  void LapsPlugin::DrawLaps()
  {

      QColor color = ui_.color->color();
      glColor4f(color.redF(), color.greenF(), color.blueF(), 0.5);
      glLineWidth(3);

      if(laps_.size()!=0)
      {
          for(int i=0;i<laps_.size();i++)
          { glBegin(GL_LINE_STRIP);
              int hue=color.hue()+i*10*3.1415926;
              int sat=color.saturation();
              int v=color.value();
              color.setHsv(hue,sat,v);
               glColor4f(color.redF(), color.greenF(), color.blueF(), 0.5);
              std::list<StampedPoint>::iterator it= laps_[i].begin();
              for (; it != laps_[i].end(); it++)
              {

                  if (it->transformed)
                  {

                      glVertex2f(it->transformed_point.getX(), it->transformed_point.getY());

                  }
              }
            glEnd();

          }

          int hue=color.hue()+laps_.size()*10*3.1415926;
          int sat=color.saturation();
          int v=color.value();
          color.setHsv(hue,sat,v);
           glColor4f(color.redF(), color.greenF(), color.blueF(), 0.5);
      }

      glBegin(GL_LINE_STRIP);
      if(points_.size()>0)
      {
          std::list<StampedPoint>::iterator it= points_.begin();
          for (; it != points_.end(); ++it)
          {
              if (it->transformed)
              {
                  glVertex2f(it->transformed_point.getX(), it->transformed_point.getY());
              }
          }
          if(cur_point_.transformed)
          {
              glVertex2f(cur_point_.transformed_point.getX(),cur_point_.transformed_point.getY());
          }
      }


      glEnd();
  }



  bool LapsPlugin::DrawArrows()
  {
    bool transformed = false;
    QColor color = ui_.color->color();
    glColor4f(color.redF(), color.greenF(), color.blueF(), 0.5);
    glLineWidth(2);
    if(laps_.size()!=0)
    {
        for(int i=0;i<laps_.size();i++)
        {
            int hue=color.hue()+i*10*3.1415926;
            int sat=color.saturation();
            int v=color.value();
            color.setHsv(hue,sat,v);
             glColor4f(color.redF(), color.greenF(), color.blueF(), 0.5);
             std::list<StampedPoint>::iterator it= laps_[i].begin();
             for (; it != laps_[i].end(); ++it)
             {glBegin(GL_LINE_STRIP);
               if (it->transformed)
               {
                 glVertex2f(it->transformed_point.getX(), it->transformed_point.getY());
                 glVertex2f(it->transformed_arrow_point.getX(), it->transformed_arrow_point.getY());

                 glVertex2f(it->transformed_arrow_point.getX(), it->transformed_arrow_point.getY());
                 glVertex2f(it->transformed_arrow_left.getX(), it->transformed_arrow_left.getY());

                 glVertex2f(it->transformed_arrow_point.getX(), it->transformed_arrow_point.getY());
                 glVertex2f(it->transformed_arrow_right.getX(), it->transformed_arrow_right.getY());

                 transformed = true;
               }
               glEnd();
             }



        }glEnd();

        int hue=color.hue()+laps_.size()*10*3.1415926;
        int sat=color.saturation();
        int v=color.value();
        color.setHsv(hue,sat,v);
         glColor4f(color.redF(), color.greenF(), color.blueF(), 0.5);
     }



    if(points_.size()>0)
    {
        std::list<StampedPoint>::iterator it = points_.begin();
        for (; it != points_.end(); ++it)
        {glBegin(GL_LINE_STRIP);
          if (it->transformed)
          {
            glVertex2f(it->transformed_point.getX(), it->transformed_point.getY());
            glVertex2f(it->transformed_arrow_point.getX(), it->transformed_arrow_point.getY());

            glVertex2f(it->transformed_arrow_point.getX(), it->transformed_arrow_point.getY());
            glVertex2f(it->transformed_arrow_left.getX(), it->transformed_arrow_left.getY());

            glVertex2f(it->transformed_arrow_point.getX(), it->transformed_arrow_point.getY());
            glVertex2f(it->transformed_arrow_right.getX(), it->transformed_arrow_right.getY());

           }
        glEnd();
        }

     }

  }

  bool LapsPlugin::TransformPoint(StampedPoint& point)
  {
    swri_transform_util::Transform transform;
    if (GetTransform(point.source_frame, point.stamp, transform))
    {
      point.transformed_point = transform * point.point;

      tf::Transform orientation(tf::Transform(transform.GetOrientation()) * point.orientation);
      point.transformed_arrow_point = point.transformed_point + orientation * tf::Point(1.0, 0.0, 0.0);
      point.transformed_arrow_left = point.transformed_point + orientation * tf::Point(0.75, -0.2, 0.0);
      point.transformed_arrow_right = point.transformed_point + orientation * tf::Point(0.75, 0.2, 0.0);

      point.transformed = true;
      return true;
    }

     point.transformed = false;
     return false;
  }

  void LapsPlugin::Transform()
  {
    bool transformed = false;

    std::list<StampedPoint>::iterator points_it = points_.begin();
    for (; points_it != points_.end(); ++points_it)
    {
      transformed = transformed | TransformPoint(*points_it);
    }
    if(laps_.size()>0)
    {
        for (int i=0;i<laps_.size();i++)
        {
            std::list<StampedPoint>::iterator lap_it = laps_[i].begin();
            for (; lap_it != laps_[i].end(); ++lap_it)
            {
                transformed = transformed | TransformPoint(*lap_it);
            }
        }
    }
    transformed = transformed | TransformPoint(cur_point_);

    if (!points_.empty() && !transformed)
    {
      PrintError("No transform between " + cur_point_.source_frame + " and " + target_frame_);
    }
  }

  void LapsPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["topic"])
    {
      std::string topic;
      node["topic"] >> topic;
      ui_.topic->setText(topic.c_str());
    }

    if (node["color"])
    {
      std::string color;
      node["color"] >> color;
      ui_.color->setColor(QColor(color.c_str()));
    }

    if (node["draw_style"])
    {
      std::string draw_style;
      node["draw_style"] >> draw_style;

      if (draw_style == "lines")
      {
        draw_style_ = LINES;
        ui_.drawstyle->setCurrentIndex(0);
      }
      else if (draw_style == "arrows")
      {
        draw_style_ = ARROWS;
        ui_.drawstyle->setCurrentIndex(2);
      }
    }
    TopicEdited();
  }

  void LapsPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "topic" << YAML::Value << topic;

    std::string color = ui_.color->color().name().toStdString();
    emitter << YAML::Key << "color" << YAML::Value << color;

    std::string draw_style = ui_.drawstyle->currentText().toStdString();
    emitter << YAML::Key << "draw_style" << YAML::Value << draw_style;

  }
}

