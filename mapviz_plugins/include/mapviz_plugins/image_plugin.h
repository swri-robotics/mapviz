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

#ifndef MAPVIZ_PLUGINS_IMAGE_PLUGIN_H_
#define MAPVIZ_PLUGINS_IMAGE_PLUGIN_H_

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
#include <sensor_msgs/Image.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <mapviz/mapviz_plugin.h>
#include <mapviz/map_canvas.h>

// QT autogenerated files
#include "ui_image_config.h"
#include "ui_topic_select.h"

namespace mapviz_plugins
{
  class ImagePlugin : public mapviz::MapvizPlugin
  {
    Q_OBJECT

  public:
    enum Anchor {
      TOP_LEFT,
      TOP_CENTER,
      TOP_RIGHT,
      CENTER_LEFT,
      CENTER,
      CENTER_RIGHT,
      BOTTOM_LEFT,
      BOTTOM_CENTER,
      BOTTOM_RIGHT};

    enum Units {PIXELS, PERCENT};

    ImagePlugin();
    virtual ~ImagePlugin();

    bool Initialize(QGLWidget* canvas);
    void Shutdown() {}

    void Draw(double x, double y, double scale);

    void Transform() {}

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
    void SetAnchor(QString anchor);
    void SetUnits(QString units);
    void SetOffsetX(int offset);
    void SetOffsetY(int offset);
    void SetWidth(int width);
    void SetHeight(int height);

  private:
    Ui::image_config ui_;
    QWidget* config_widget_;

    std::string topic_;
    Anchor anchor_;
    Units units_;
    int offset_x_;
    int offset_y_;
    int width_;
    int height_;

    bool has_image_;

    int last_width_;
    int last_height_;

    ros::Subscriber image_sub_;
    bool has_message_;

    sensor_msgs::Image image_;

    cv_bridge::CvImagePtr cv_image_;
    cv::Mat scaled_image_;

    void imageCallback(const sensor_msgs::ImageConstPtr image);

    void ScaleImage(int width, int height);
    void DrawIplImage(cv::Mat *image);

    std::string AnchorToString(Anchor anchor);
    std::string UnitsToString(Units units);
  };
}

#endif  // MAPVIZ_PLUGINS_IMAGE_PLUGIN_H_
