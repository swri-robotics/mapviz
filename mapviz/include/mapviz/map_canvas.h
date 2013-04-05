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

#ifndef MAP_CANVAS_H
#define MAP_CANVAS_H

// C++ standard libraries
#include <list>
#include <string>

// QT libraries
#include <QGLWidget>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QColor>

// ROS libraries
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <mapviz/mapviz_plugin.h>

class MapCanvas : public QGLWidget
{
  Q_OBJECT

public:
  MapCanvas(QWidget *parent = 0);
  ~MapCanvas();

  void InitializeTf();

  void AddPlugin(mapviz::MapvizPlugin* plugin, int order);
  void RemovePlugin(mapviz::MapvizPlugin* plugin);
  void SetFixedFrame(const std::string& frame);
  void SetTargetFrame(const std::string& frame);
  void ToggleFixOrientation(bool on);
  void ToggleUseLatestTransforms(bool on);
  void UpdateView();
  void ReorderDisplays();

  float ViewScale() const { return view_scale_; }
  float OffsetX() const { return offset_x_; }
  float OffsetY() const { return offset_y_; }

  void SetViewScale(float scale)
  {
    view_scale_ = scale;
    UpdateView();
  }

  void SetOffsetX(float x)
  {
    offset_x_ = x;
    UpdateView();
  }

  void SetOffsetY(float y)
  {
    offset_y_ = y;
    UpdateView();
  }

  void SetBackground(const QColor& color)
  {
    background_ = color;
    update();
  }

protected:
  void initializeGL();
  void resizeGL( int w, int h );
  void paintGL();
  void wheelEvent(QWheelEvent* e);
  void mousePressEvent(QMouseEvent* e);
  void mouseReleaseEvent(QMouseEvent* e);
  void mouseMoveEvent(QMouseEvent* e);

  void Recenter();
  void TransformTarget();

  bool initialized_;
  bool fix_orientation_;

  QColor background_;

  bool mouse_pressed_;
  int mouse_x_;
  int mouse_y_;

  // Offset based on previous mouse drags
  double offset_x_;
  double offset_y_;

  // Offset based on current mouse drag
  double drag_x_;
  double drag_y_;

  // The center of the view
  float view_center_x_;
  float view_center_y_;

  // View scale in meters per pixel
  float view_scale_;

  // The bounds of the view
  float view_left_;
  float view_right_;
  float view_top_;
  float view_bottom_;

  // The bounds of the scene
  float scene_left_;
  float scene_right_;
  float scene_top_;
  float scene_bottom_;

  std::string fixed_frame_;
  std::string target_frame_;

  tf::TransformListener* transform_listener_;
  std::list<mapviz::MapvizPlugin*> plugins_;

};

#endif // MAP_CANVAS_H
