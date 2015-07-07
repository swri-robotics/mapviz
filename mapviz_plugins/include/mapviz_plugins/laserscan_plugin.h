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

#ifndef MAPVIZ_PLUGINS_LASERSCAN_PLUGIN_H_
#define MAPVIZ_PLUGINS_LASERSCAN_PLUGIN_H_

// C++ standard libraries
#include <deque>
#include <string>
#include <vector>

#include <mapviz/mapviz_plugin.h>
#include <GL/glut.h>

// QT libraries
#include <QGLWidget>
#include <QObject>
#include <QWidget>
#include <QColor>

// ROS libraries
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/ColorRGBA.h>

#include <mapviz/map_canvas.h>

// QT autogenerated files
#include "ui_laserscan_config.h"
#include "ui_topic_select.h"

namespace mapviz_plugins
{
  class LaserScanPlugin : public mapviz::MapvizPlugin
  {
    Q_OBJECT

  public:
    LaserScanPlugin();
    virtual ~LaserScanPlugin();

    bool Initialize(QGLWidget* canvas);
    void Shutdown() {}

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
    void SelectTopic();
    void TopicEdited();
    void SelectMinColor();
    void SelectMaxColor();
    void MinIntensityChanged(double value);
    void MaxIntensityChanged(double value);
    void PointSizeChanged(int value);
    void BufferSizeChanged(int value);

  private:
    struct StampedPoint
    {
      tf::Point point;
      tf::Point transformed_point;
      QColor color;
      float intensity;
    };

    struct Scan
    {
      ros::Time stamp;
      std::vector<StampedPoint> points;
      bool transformed;
      bool has_intensity;
    };

    Ui::laserscan_config ui_;
    QWidget* config_widget_;

    std::string topic_;
    QColor min_color_;
    QColor max_color_;
    double min_intensity_;
    double max_intensity_;
    int32_t point_size_;
    int32_t buffer_size_;

    ros::Subscriber laserscan_sub_;
    bool has_message_;

    std::deque<Scan> scans_;

    void laserScanCallback(const sensor_msgs::LaserScanConstPtr scan);
    
    void UpdateColors();
    void DrawIcon();
    QColor InterpolateColors(const QColor& c1, const QColor& c2, double weight) const;
  };
}

#endif  // MAPVIZ_PLUGINS_LASERSCAN_PLUGIN_H_
