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

#include <mapviz_plugins/odometry_plugin.h>

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
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::OdometryPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  OdometryPlugin::OdometryPlugin() : config_widget_(new QWidget())
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

    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this,
                     SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this,
                     SLOT(TopicEdited()));
    QObject::connect(ui_.positiontolerance, SIGNAL(valueChanged(double)), this,
                     SLOT(PositionToleranceChanged(double)));
    QObject::connect(ui_.buffersize, SIGNAL(valueChanged(int)), this,
                     SLOT(BufferSizeChanged(int)));
    QObject::connect(ui_.drawstyle, SIGNAL(activated(QString)), this,
                     SLOT(SetDrawStyle(QString)));
    QObject::connect(ui_.static_arrow_sizes, SIGNAL(clicked(bool)),
                     this, SLOT(SetStaticArrowSizes(bool)));
    QObject::connect(ui_.arrow_size, SIGNAL(valueChanged(int)),
                     this, SLOT(SetArrowSize(int)));
    QObject::connect(ui_.color, SIGNAL(colorEdited(const QColor&)), this,
                     SLOT(SetColor(const QColor&)));
    QObject::connect(ui_.show_laps, SIGNAL(toggled(bool)), this,
                     SLOT(LapToggled(bool)));
    QObject::connect(ui_.show_covariance, SIGNAL(toggled(bool)), this,
                     SLOT(CovariancedToggled(bool)));
    QObject::connect(ui_.buttonResetBuffer, SIGNAL(pressed()), this,
                     SLOT(ClearPoints()));
  }

  OdometryPlugin::~OdometryPlugin()
  {
  }

  void OdometryPlugin::SelectTopic()
  {
    ros::master::TopicInfo topic =
        mapviz::SelectTopicDialog::selectTopic("nav_msgs/Odometry");

    if (!topic.name.empty())
    {
      ui_.topic->setText(QString::fromStdString(topic.name));
      TopicEdited();
    }
  }

  void OdometryPlugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();
    if (topic != topic_)
    {
      initialized_ = false;
      ClearPoints();
      has_message_ = false;
      PrintWarning("No messages received.");

      odometry_sub_.shutdown();

      topic_ = topic;
      if (!topic.empty())
      {
        odometry_sub_ = node_.subscribe(
                    topic_, 1, &OdometryPlugin::odometryCallback, this);

        ROS_INFO("Subscribing to %s", topic_.c_str());
      }
    }
  }

  void OdometryPlugin::odometryCallback(
      const nav_msgs::OdometryConstPtr odometry)
  {
    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }

    // Note that unlike some plugins, this one does not store nor rely on the
    // source_frame_ member variable.  This one can potentially store many
    // messages with different source frames, so we need to store and transform
    // them individually.
    StampedPoint stamped_point;
    stamped_point.stamp = odometry->header.stamp;
    stamped_point.source_frame = odometry->header.frame_id;

    stamped_point.point = tf::Point(odometry->pose.pose.position.x,
                                    odometry->pose.pose.position.y,
                                    odometry->pose.pose.position.z);

    stamped_point.orientation = tf::Quaternion(
        odometry->pose.pose.orientation.x,
        odometry->pose.pose.orientation.y,
        odometry->pose.pose.orientation.z,
        odometry->pose.pose.orientation.w);

    if ( ui_.show_covariance->isChecked() )
    {
      tf::Matrix3x3 tf_cov =
          swri_transform_util::GetUpperLeft(odometry->pose.covariance);

      if (tf_cov[0][0] < 100000 && tf_cov[1][1] < 100000)
      {
        cv::Mat cov_matrix_3d(3, 3, CV_32FC1);
        for (int32_t r = 0; r < 3; r++)
        {
          for (int32_t c = 0; c < 3; c++)
          {
            cov_matrix_3d.at<float>(r, c) = tf_cov[r][c];
          }
        }

        cv::Mat cov_matrix_2d = swri_image_util::ProjectEllipsoid(cov_matrix_3d);

        if (!cov_matrix_2d.empty())
        {
          stamped_point.cov_points = swri_image_util::GetEllipsePoints(
              cov_matrix_2d, stamped_point.point, 3, 32);

          stamped_point.transformed_cov_points = stamped_point.cov_points;
        }
        else
        {
          ROS_ERROR("Failed to project x, y, z covariance to xy-plane.");
        }
      }
    }

    pushPoint( std::move(stamped_point) );

  }

  void OdometryPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void OdometryPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void OdometryPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  QWidget* OdometryPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool OdometryPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;
    SetColor(ui_.color->color());

    return true;
  }

  void OdometryPlugin::Draw(double x, double y, double scale)
  {
    if (ui_.show_covariance->isChecked())
    {
      DrawCovariance();
    }
    if (DrawPoints(scale))
    {
      PrintInfo("OK");
    }
  }
  

  void OdometryPlugin::Paint(QPainter* painter, double x, double y, double scale)
  {
    //dont render any timestamps if the show_timestamps is set to 0
    int interval = ui_.show_timestamps->value();
    if (interval == 0)
    {
      return;
    }

    QTransform tf = painter->worldTransform();
    QFont font("Helvetica", 10);
    painter->setFont(font);
    painter->save();
    painter->resetTransform();

    //set the draw color for the text to be the same as the rest
    QPen pen(QBrush(ui_.color->color()), 1);
    painter->setPen(pen);

    int counter = 0;//used to alternate between rendering text on some points
    for (const StampedPoint& point: points())
    {
      if (point.transformed && counter % interval == 0)//this renders a timestamp every 'interval' points
      {
        QPointF qpoint = tf.map(QPointF(point.transformed_point.getX(),
                                        point.transformed_point.getY()));
        QString time;
        time.setNum(point.stamp.toSec(), 'g', 12);
        painter->drawText(qpoint, time);
      }
      counter++;
    }

    painter->restore();
  }

  void OdometryPlugin::LoadConfig(const YAML::Node& node,
                                  const std::string& path)
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
      QColor qcolor(color.c_str());
      SetColor(qcolor);
      ui_.color->setColor(qcolor);
    }

    if (node["draw_style"])
    {
      std::string draw_style;
      node["draw_style"] >> draw_style;

      if (draw_style == "lines")
      {
        ui_.drawstyle->setCurrentIndex(0);
        SetDrawStyle( LINES );
      }
      else if (draw_style == "points")
      {
        ui_.drawstyle->setCurrentIndex(1);
        SetDrawStyle( POINTS );
      }
      else if (draw_style == "arrows")
      {
        ui_.drawstyle->setCurrentIndex(2);
        SetDrawStyle( ARROWS );
      }
    }

    if (node["position_tolerance"])
    {
      double position_tolerance;
      node["position_tolerance"] >> position_tolerance;
      ui_.positiontolerance->setValue(position_tolerance);
      PositionToleranceChanged(position_tolerance);
    }

    if (node["buffer_size"])
    {
      double buffer_size;
      node["buffer_size"] >> buffer_size;
      ui_.buffersize->setValue(buffer_size);
      BufferSizeChanged(buffer_size);
    }

    if (node["show_covariance"])
    {
      bool show_covariance = false;
      node["show_covariance"] >> show_covariance;
      ui_.show_covariance->setChecked(show_covariance);
      CovariancedToggled(show_covariance);
    }
    if (node["show_laps"])
    {
      bool show_laps = false;
      node["show_laps"] >> show_laps;
      ui_.show_laps->setChecked(show_laps);
      LapToggled(show_laps);
    }

    if (node["static_arrow_sizes"])
    {
      bool static_arrow_sizes = node["static_arrow_sizes"].as<bool>();
      ui_.static_arrow_sizes->setChecked(static_arrow_sizes);
      SetStaticArrowSizes(static_arrow_sizes);
    }

    if (node["arrow_size"])
    {
      double arrow_size = node["arrow_size"].as<int>();
      ui_.arrow_size->setValue(arrow_size);
      SetArrowSize(arrow_size);
    }

    if (node["show_timestamps"])
    {
      ui_.show_timestamps->setValue(node["show_timestamps"].as<int>());
    }

    TopicEdited();
  }

  void OdometryPlugin::SaveConfig(YAML::Emitter& emitter,
                                  const std::string& path)
  {
    std::string topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "topic" << YAML::Value << topic;

    std::string color = ui_.color->color().name().toStdString();
    emitter << YAML::Key << "color" << YAML::Value << color;

    std::string draw_style = ui_.drawstyle->currentText().toStdString();
    emitter << YAML::Key << "draw_style" << YAML::Value << draw_style;

    emitter << YAML::Key << "position_tolerance" <<
               YAML::Value << positionTolerance();

    emitter << YAML::Key << "buffer_size" << YAML::Value << bufferSize();

    bool show_laps = ui_.show_laps->isChecked();
    emitter << YAML::Key << "show_laps" << YAML::Value << show_laps;

    bool show_covariance = ui_.show_covariance->isChecked();
    emitter << YAML::Key << "show_covariance" << YAML::Value << show_covariance;

    emitter << YAML::Key << "static_arrow_sizes" << YAML::Value << ui_.static_arrow_sizes->isChecked();

    emitter << YAML::Key << "arrow_size" << YAML::Value << ui_.arrow_size->value();

    emitter << YAML::Key << "show_timestamps" << YAML::Value << ui_.show_timestamps->value();
  }
}


