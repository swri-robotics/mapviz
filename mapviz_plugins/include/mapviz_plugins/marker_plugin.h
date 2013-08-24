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

#ifndef MAPVIZ_PLUGINS_MARKER_PLUGIN_H_
#define MAPVIZ_PLUGINS_MARKER_PLUGIN_H_

// C++ standard libraries
#include <string>
#include <list>
#include <map>

// QT libraries
#include <QGLWidget>
#include <QObject>
#include <QWidget>
#include <QColor>

// ROS libraries
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

#include <GL/glut.h>
#include <mapviz/mapviz_plugin.h>
#include <mapviz/map_canvas.h>

// QT autogenerated files
#include "ui_marker_config.h"
#include "ui_topic_select.h"

namespace mapviz_plugins
{
  class MarkerPlugin : public mapviz::MapvizPlugin
  {
    Q_OBJECT

  public:

    MarkerPlugin();
    virtual ~MarkerPlugin();

    bool Initialize(QGLWidget* canvas);
    void Shutdown() {}

    void Draw(double x, double y, double scale);

    void Transform();

    void LoadConfiguration(const YAML::Node& node, const std::string& path);
    void SaveConfiguration(YAML::Emitter& emitter, const std::string& path);

    QWidget* GetConfigWidget(QWidget* parent);

  protected:
    void PrintError(const std::string& message);
    void PrintInfo(const std::string& message);
    void PrintWarning(const std::string& message);

  protected Q_SLOTS:
    void SelectTopic();
    void TopicEdited();

  private:

    struct StampedPoint
    {
      tf::Point point;
      tf::Point transformed_point;
      QColor color;
    };

    struct MarkerData
    {
      ros::Time stamp;
      ros::Time expire_time;

      int display_type;
      QColor color;

      std::list<StampedPoint> points;
      std::list<std::string> texts;

      float scale_x;
      float scale_y;
      float scale_z;

      bool transformed;
    };

    Ui::marker_config ui_;
    QWidget* config_widget_;

    std::string topic_;

    ros::Subscriber marker_sub_;
    bool has_message_;

    std::map<int, MarkerData> markers_;

    bool is_marker_array_;

    void markerCallback(const visualization_msgs::MarkerConstPtr odometry);
    void markerArrayCallback(const visualization_msgs::MarkerArrayConstPtr markers);
  };
}

#endif  // MAPVIZ_PLUGINS_MARKER_PLUGIN_H_
