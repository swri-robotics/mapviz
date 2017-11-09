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

#ifndef MAPVIZ_PLUGINS_NAVSAT_PLUGIN_H_
#define MAPVIZ_PLUGINS_NAVSAT_PLUGIN_H_

#include <mapviz/mapviz_plugin.h>
#include <mapviz/map_canvas.h>
#include <mapviz_plugins/point_drawing_plugin.h>
// C++ standard libraries
#include <list>
#include <string>
#include <vector>

// QT libraries
#include <QGLWidget>
#include <QObject>
#include <QWidget>

// ROS libraries
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <swri_transform_util/local_xy_util.h>

// QT autogenerated files
#include "ui_navsat_config.h"

namespace mapviz_plugins
{
  class NavSatPlugin : public mapviz_plugins::PointDrawingPlugin
  {
    Q_OBJECT

   public:
    NavSatPlugin();
    virtual ~NavSatPlugin();

    bool Initialize(QGLWidget* canvas);
    void Shutdown()
    {
    }

    void Draw(double x, double y, double scale);

    void LoadConfig(const YAML::Node& node, const std::string& path);
    void SaveConfig(YAML::Emitter& emitter, const std::string& path);

    QWidget* GetConfigWidget(QWidget* parent);

   protected:
    void PrintError(const std::string& message);
    void PrintInfo(const std::string& message);
    void PrintWarning(const std::string& message);

   protected Q_SLOTS:
    void SelectTopic();
    void TopicEdited();
    virtual void SetDrawStyle(QString style) override;

   private:
    Ui::navsat_config ui_;
    QWidget* config_widget_;

    std::string topic_;

    ros::Subscriber navsat_sub_;
    bool has_message_;

    swri_transform_util::LocalXyWgs84Util local_xy_util_;

    void NavSatFixCallback(const sensor_msgs::NavSatFixConstPtr navsat);
  };
}

#endif  // MAPVIZ_PLUGINS_NAVSAT_PLUGIN_H_
