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

#ifndef MAPVIZ_PLUGINS_GRID_PLUGIN_H_
#define MAPVIZ_PLUGINS_GRID_PLUGIN_H_

// C++ standard libraries
#include <string>
#include <list>

// QT libraries
#include <QGLWidget>
#include <QObject>
#include <QWidget>
#include <QTimer>
#include <QColor>

// ROS libraries
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <mapviz/mapviz_plugin.h>
#include <mapviz/map_canvas.h>

// QT autogenerated files
#include "ui_grid_config.h"

namespace mapviz_plugins
{
  class GridPlugin : public mapviz::MapvizPlugin
  {
    Q_OBJECT

  public:
    GridPlugin();
    virtual ~GridPlugin();

    bool Initialize(QGLWidget* canvas);
    void Shutdown();

    void Draw(double x, double y, double scale);

    void Transform();

    void LoadConfig(const YAML::Node& node, const std::string& path);
    void SaveConfig(YAML::Emitter& emitter, const std::string& path);

    QWidget* GetConfigWidget(QWidget* parent);

  protected:
    void PrintError(const std::string& message);
    void PrintInfo(const std::string& message);
    void PrintWarning(const std::string& message);

  protected Q_SLOTS:
    void SelectColor();
    void SetAlpha(double alpha);
    void SetX(double x);
    void SetY(double y);
    void SetSize(double size);
    void SetRows(int rows);
    void SetColumns(int columns);
    void SetFrame(QString frame);
    void UpdateFrames();

  private:
    Ui::grid_config ui_;
    QWidget* config_widget_;
    QTimer frame_timer_;
    QColor color_;

    double alpha_;

    tf::Point top_left_;

    double size_;
    int rows_;
    int columns_;

    bool exit_;

    bool transformed_;

    std::list<tf::Point> top_points_;
    std::list<tf::Point> bottom_points_;
    std::list<tf::Point> left_points_;
    std::list<tf::Point> right_points_;

    std::list<tf::Point> transformed_top_points_;
    std::list<tf::Point> transformed_bottom_points_;
    std::list<tf::Point> transformed_left_points_;
    std::list<tf::Point> transformed_right_points_;

    transform_util::Transform transform_;

    void RecalculateGrid();
    void Transform(std::list<tf::Point>& src, std::list<tf::Point>& dst);
  };
}

#endif  // MAPVIZ_PLUGINS_GRID_PLUGIN_H_
