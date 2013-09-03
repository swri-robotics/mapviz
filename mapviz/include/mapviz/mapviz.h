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

#ifndef MAPVIZ_MAPVIZ_H_
#define MAPVIZ_MAPVIZ_H_

// C++ standard libraries
#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

// QT libraries
#include <QtGui/QtGui>
#include <QtGui/QMainWindow>
#include <QDialog>
#include <QTimer>
#include <QString>
#include <QShowEvent>
#include <QCloseEvent>
#include <QListWidgetItem>
#include <QModelIndex>
#include <QColor>

// ROS libraries
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <GL/glut.h>

// Auto-generated UI files
#include "ui_mapviz.h"
#include "ui_pluginselect.h"

#include <mapviz/mapviz_plugin.h>
#include <mapviz/map_canvas.h>

namespace mapviz
{
  class Mapviz : public QMainWindow
  {
    Q_OBJECT

  public:
    Mapviz(int argc, char **argv, QWidget *parent = 0, Qt::WFlags flags = 0);
    ~Mapviz();

    void Initialize();

  public Q_SLOTS:
    void AutoSave();
    void OpenConfig();
    void SaveConfig();
    void SelectNewDisplay();
    void RemoveDisplay();
    void ReorderDisplays();
    void FixedFrameSelected(const QString& text);
    void TargetFrameSelected(const QString& text);
    void ToggleUseLatestTransforms(bool on);
    void UpdateFrames();
    void SpinOnce();
    void UpdateSizeHints();
    void ToggleConfigPanel(bool on);
    void ToggleFixOrientation(bool on);
    void ToggleShowPlugin(QListWidgetItem* item, bool visible);
    void Force720p(bool on);
    void Force480p(bool on);
    void SetResizable(bool on);
    void SelectBackgroundColor();

  protected:
    Ui::mapviz ui_;

    QTimer frame_timer_;
    QTimer spin_timer_;
    QTimer save_timer_;

    int    argc_;
    char** argv_;

    bool initialized_;
    bool force_720p_;
    bool force_480p_;
    bool resizable_;
    QColor background_;

    ros::NodeHandle* node_;
    boost::shared_ptr<tf::TransformListener> tf_;

    pluginlib::ClassLoader<MapvizPlugin>* loader_;
    MapCanvas* canvas_;
    std::map<QListWidgetItem*, MapvizPluginPtr> plugins_;

    void Open(const std::string& filename);
    void Save(const std::string& filename);

    MapvizPluginPtr CreateNewDisplay(
        const std::string& name,
        const std::string& type,
        bool visible,
        bool collapsed);

    void ClearDisplays();
    void AdjustWindowSize();

    virtual void showEvent(QShowEvent* event);
    virtual void closeEvent(QCloseEvent* event);
  };
}

#endif  // MAPVIZ_MAPVIZ_H_
