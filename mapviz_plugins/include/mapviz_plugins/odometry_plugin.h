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

#ifndef ODOMETRY_PLUGIN_H
#define ODOMETRY_PLUGIN_H

// C++ standard libraries
#include <string>
#include <list>

// QT libraries
#include <QGLWidget>
#include <QObject>
#include <QWidget>
#include <QColor>

// ROS libraries
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

#include <mapviz/mapviz_plugin.h>
#include <mapviz/map_canvas.h>

// QT autogenerated files
#include "ui_odometry_config.h"
#include "ui_topic_select.h"

namespace mapviz_plugins
{

  class OdometryPlugin : public mapviz::MapvizPlugin
  {

    Q_OBJECT

  public:

    struct StampedPoint
    {
      tf::Point point;
      tf::Quaternion orientation;
      tf::Point transformed_point;
      tf::Point transformed_arrow_point;
      tf::Point transformed_arrow_left;
      tf::Point transformed_arrow_right;
      bool transformed;
      ros::Time stamp;
    };

    enum DrawStyle { LINES = 0, POINTS, ARROWS };

    OdometryPlugin();
    virtual ~OdometryPlugin();

    bool Initialize(QGLWidget* canvas);
    void Shutdown() {}

    void Draw(double x, double y, double scale);

    void Transform();

    void LoadConfiguration(const YAML::Node& node, const std::string& config_path);
    void SaveConfiguration(YAML::Emitter& emitter, const std::string& config_path);

    QWidget* GetConfigWidget(QWidget* parent);

  protected:
    void PrintError(const std::string& message);
    void PrintInfo(const std::string& message);
    void PrintWarning(const std::string& message);

  protected Q_SLOTS:
    void SelectTopic();
    void SelectColor();
    void TopicEdited();
    void PositionToleranceChanged(double value);
    void AngleToleranceChanged(double value);
    void BufferSizeChanged(int value);
    void SetDrawStyle(QString style);

  private:
    bool DrawArrows();
    bool TransformPoint(StampedPoint& point);

    Ui::odometry_config ui_;
    QWidget* config_widget_;
    QColor color_;

    DrawStyle draw_style_;

    std::string topic_;

    int buffer_size_;
    float position_tolerance_;
    float angle_tolerance_;

    ros::Subscriber odometry_sub_;
    bool has_message_;

    StampedPoint current_point_;
    std::list<StampedPoint> points_;

    void odometryCallback(const nav_msgs::OdometryConstPtr odometry);
  };
}

#endif /* ODOMETRY_PLUGIN_H */
