// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-58058A
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Steve Dellenback <sdellenback@swri.org> (210) 522-3914
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <mapviz_plugins/odometry_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QDialog>
#include <QColorDialog>
#include <QGLWidget>
#include <QPalette>

#include <opencv2/core/core.hpp>

// ROS libraries
#include <ros/master.h>

#include <image_util/geometry_util.h>
#include <transform_util/transform_util.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
    mapviz_plugins,
    odometry,
    mapviz_plugins::OdometryPlugin,
    mapviz::MapvizPlugin);

namespace mapviz_plugins
{
  OdometryPlugin::OdometryPlugin() :
    config_widget_(new QWidget()),
    color_(Qt::green),
    draw_style_(LINES)
  {
    ui_.setupUi(config_widget_);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    // Initialize color selector color
    ui_.selectcolor->setStyleSheet("background: " + color_.name() + ";");

    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.selectcolor, SIGNAL(clicked()), this, SLOT(SelectColor()));
    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this, SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this, SLOT(TopicEdited()));
    QObject::connect(ui_.positiontolerance, SIGNAL(valueChanged(double)), this, SLOT(PositionToleranceChanged(double)));
    QObject::connect(ui_.buffersize, SIGNAL(valueChanged(int)), this, SLOT(BufferSizeChanged(int)));
    QObject::connect(ui_.drawstyle, SIGNAL(activated(QString)), this, SLOT(SetDrawStyle(QString)));
  }

  OdometryPlugin::~OdometryPlugin()
  {
  }

  void OdometryPlugin::SetDrawStyle(QString style)
  {
    if (style == "lines")
    {
      draw_style_ = LINES;
    }
    else if (style == "points")
    {
      draw_style_ = POINTS;
    }
    else if (style == "arrows")
    {
      draw_style_ = ARROWS;
    }

    canvas_->update();
  }

  void OdometryPlugin::SelectTopic()
  {
    QDialog dialog;
    Ui::topicselect ui;
    ui.setupUi(&dialog);

    std::vector<ros::master::TopicInfo> topics;
    ros::master::getTopics(topics);

    for (unsigned int i = 0; i < topics.size(); i++)
    {
      if (topics[i].datatype == "nav_msgs/Odometry")
      {
        ui.displaylist->addItem(topics[i].name.c_str());
      }
    }
    ui.displaylist->setCurrentRow(0);

    dialog.exec();

    if (dialog.result() == QDialog::Accepted && ui.displaylist->selectedItems().count() == 1)
    {
      ui_.topic->setText(ui.displaylist->selectedItems().first()->text());
      TopicEdited();
    }
  }

  void OdometryPlugin::SelectColor()
  {
    QColorDialog dialog(color_, config_widget_);
    dialog.exec();

    if (dialog.result() == QDialog::Accepted)
    {
      color_ = dialog.selectedColor();
      ui_.selectcolor->setStyleSheet("background: " + color_.name() + ";");
      canvas_->update();
    }
  }

  void OdometryPlugin::TopicEdited()
  {
    if (ui_.topic->text().toStdString() != topic_)
    {
      initialized_ = false;
      points_.clear();
      has_message_ = false;
      topic_ = ui_.topic->text().toStdString();
      PrintWarning("No messages received.");

      odometry_sub_.shutdown();
      odometry_sub_ = node_.subscribe(topic_, 1, &OdometryPlugin::odometryCallback, this);

      ROS_INFO("Subscribing to %s", topic_.c_str());
    }
  }

  void OdometryPlugin::odometryCallback(const nav_msgs::OdometryConstPtr odometry)
  {
    if (!has_message_)
    {
      source_frame_ = odometry->header.frame_id;
      initialized_ = true;
      has_message_ = true;
    }

    StampedPoint stamped_point;
    stamped_point.stamp = odometry->header.stamp;

    stamped_point.point = tf::Point(
        odometry->pose.pose.position.x,
        odometry->pose.pose.position.y,
        odometry->pose.pose.position.z);

    stamped_point.orientation = tf::Quaternion(
        odometry->pose.pose.orientation.x,
        odometry->pose.pose.orientation.y,
        odometry->pose.pose.orientation.z,
        odometry->pose.pose.orientation.w);

    if (points_.empty() || stamped_point.point.distance(points_.back().point) >= position_tolerance_)
    {
      points_.push_back(stamped_point);
    }

    if (buffer_size_ > 0)
    {
      while (static_cast<int>(points_.size()) > buffer_size_)
      {
        points_.pop_front();
      }
    }

    cur_point_ = stamped_point;

    if (ui_.show_covariance->isChecked())
    {
      tf::Matrix3x3 tf_cov =
          transform_util::GetUpperLeft(odometry->pose.covariance);

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

        cv::Mat cov_matrix_2d = image_util::ProjectEllipsoid(cov_matrix_3d);

        if (!cov_matrix_2d.empty())
        {
          cur_point_.cov_points = image_util::GetEllipsePoints(
              cov_matrix_2d, cur_point_.point, 3, 32);

          cur_point_.transformed_cov_points = cur_point_.cov_points;
        }
        else
        {
          ROS_ERROR("Failed to project x, y, z covariance to xy-plane.");
        }
      }
    }

    canvas_->update();
  }

  void OdometryPlugin::PositionToleranceChanged(double value)
  {
    position_tolerance_ = value;
  }

  void OdometryPlugin::BufferSizeChanged(int value)
  {
    buffer_size_ = value;

    if (buffer_size_ > 0)
    {
      while (static_cast<int>(points_.size()) > buffer_size_)
      {
        points_.pop_front();
      }
    }

    canvas_->update();
  }

  void OdometryPlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_ERROR("Error: %s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void OdometryPlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_INFO("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void OdometryPlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_WARN("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  QWidget* OdometryPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool OdometryPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;

    return true;
  }

  void OdometryPlugin::Draw(double x, double y, double scale)
  {
    glColor4f(color_.redF(), color_.greenF(), color_.blueF(), 0.5);

    bool transformed = false;

    if (ui_.show_covariance->isChecked())
    {
      DrawCovariance();
    }

    glColor4f(color_.redF(), color_.greenF(), color_.blueF(), 1.0);

    if (draw_style_ == ARROWS)
    {
      transformed = DrawArrows();
    }
    else
    {
      if (draw_style_ == LINES)
      {
        glLineWidth(3);
        glBegin(GL_LINE_STRIP);
      }
      else
      {
        glPointSize(6);
        glBegin(GL_POINTS);
      }

      std::list<StampedPoint>::iterator it = points_.begin();
      for (; it != points_.end(); ++it)
      {
        if (it->transformed)
        {
          glVertex2f(it->transformed_point.getX(), it->transformed_point.getY());

          transformed = true;
        }
      }

      if (cur_point_.transformed)
      {
        glVertex2f(
          cur_point_.transformed_point.getX(),
          cur_point_.transformed_point.getY());

        transformed = true;
      }

      glEnd();
    }

    if (transformed)
    {
      PrintInfo("OK");
    }
  }

  void OdometryPlugin::DrawCovariance()
  {
    glLineWidth(4);

    if (cur_point_.transformed && !cur_point_.transformed_cov_points.empty())
    {
      glBegin(GL_LINE_STRIP);

      for (uint32_t i = 0; i < cur_point_.transformed_cov_points.size(); i++)
      {
        glVertex2f(
            cur_point_.transformed_cov_points[i].getX(),
            cur_point_.transformed_cov_points[i].getY());
      }

      glVertex2f(
          cur_point_.transformed_cov_points.front().getX(),
          cur_point_.transformed_cov_points.front().getY());

      glEnd();
    }
  }

  bool OdometryPlugin::DrawArrows()
  {
    bool transformed = false;
    glLineWidth(2);
    glBegin(GL_LINES);

    std::list<StampedPoint>::iterator it = points_.begin();
    for (; it != points_.end(); ++it)
    {
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
    }

    if (cur_point_.transformed)
    {
      glVertex2f(
        cur_point_.transformed_point.getX(),
        cur_point_.transformed_point.getY());
      glVertex2f(
        cur_point_.transformed_arrow_point.getX(),
        cur_point_.transformed_arrow_point.getY());

      glVertex2f(
        cur_point_.transformed_arrow_point.getX(),
        cur_point_.transformed_arrow_point.getY());
      glVertex2f(
        cur_point_.transformed_arrow_left.getX(),
        cur_point_.transformed_arrow_left.getY());

      glVertex2f(
        cur_point_.transformed_arrow_point.getX(),
        cur_point_.transformed_arrow_point.getY());
      glVertex2f(
        cur_point_.transformed_arrow_right.getX(),
        cur_point_.transformed_arrow_right.getY());

      transformed = true;
    }


    glEnd();

    return transformed;
  }

  bool OdometryPlugin::TransformPoint(StampedPoint& point)
  {
    tf::StampedTransform transform;
    if (GetTransform(point.stamp, transform))
    {
      point.transformed_point = transform * point.point;

      tf::Transform orientation(transform * point.orientation);
      point.transformed_arrow_point = point.transformed_point + orientation * tf::Point(1.0, 0.0, 0.0);
      point.transformed_arrow_left = point.transformed_point + orientation * tf::Point(0.75, -0.2, 0.0);
      point.transformed_arrow_right = point.transformed_point + orientation * tf::Point(0.75, 0.2, 0.0);

      if (ui_.show_covariance->isChecked())
      {
        for (uint32_t i = 0; i < point.cov_points.size(); i++)
        {
          point.transformed_cov_points[i] = transform * point.cov_points[i];
        }
      }

      point.transformed = true;
      return true;
    }

     point.transformed = false;
     return false;
  }

  void OdometryPlugin::Transform()
  {
    tf::StampedTransform transform;

    bool transformed = false;

    std::list<StampedPoint>::iterator points_it = points_.begin();
    for (; points_it != points_.end(); ++points_it)
    {
      transformed = transformed | TransformPoint(*points_it);
    }

    transformed = transformed | TransformPoint(cur_point_);

    if (!points_.empty() && !transformed)
    {
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }
  }

  void OdometryPlugin::LoadConfiguration(const YAML::Node& node, const std::string& path)
  {
    std::string topic;
    node["topic"] >> topic;
    ui_.topic->setText(topic.c_str());

    std::string color;
    node["color"] >> color;
    color_ = QColor(color.c_str());
    ui_.selectcolor->setStyleSheet("background: " + color_.name() + ";");

    std::string draw_style;
    node["draw_style"] >> draw_style;

    if (draw_style == "lines")
    {
      draw_style_ = LINES;
      ui_.drawstyle->setCurrentIndex(0);
    }
    else if (draw_style == "points")
    {
      draw_style_ = POINTS;
      ui_.drawstyle->setCurrentIndex(1);
    }
    else if (draw_style == "arrows")
    {
      draw_style_ = ARROWS;
      ui_.drawstyle->setCurrentIndex(2);
    }

    node["position_tolerance"] >> position_tolerance_;
    ui_.positiontolerance->setValue(position_tolerance_);

    node["buffer_size"] >> buffer_size_;
    ui_.buffersize->setValue(buffer_size_);

    if (node.FindValue("show_covariance"))
    {
      bool show_covariance = false;
      node["show_covariance"] >> show_covariance;
      ui_.show_covariance->setChecked(show_covariance);
    }

    TopicEdited();
  }

  void OdometryPlugin::SaveConfiguration(YAML::Emitter& emitter, const std::string& path)
  {
    std::string topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "topic" << YAML::Value << topic;

    std::string color = color_.name().toStdString();
    emitter << YAML::Key << "color" << YAML::Value << color;

    std::string draw_style = ui_.drawstyle->currentText().toStdString();
    emitter << YAML::Key << "draw_style" << YAML::Value << draw_style;

    emitter << YAML::Key << "position_tolerance" << YAML::Value << position_tolerance_;

    emitter << YAML::Key << "buffer_size" << YAML::Value << buffer_size_;

    bool show_covariance = ui_.show_covariance->isChecked();
    emitter << YAML::Key << "show_covariance" << YAML::Value << show_covariance;
  }
}

