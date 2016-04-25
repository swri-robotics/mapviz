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

#ifndef MAPVIZ_PLUGINS_MARKER_PLUGIN_H_
#define MAPVIZ_PLUGINS_MARKER_PLUGIN_H_

// C++ standard libraries
#include <string>
#include <list>
#include <map>

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
#include <topic_tools/shape_shifter.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

#include <mapviz/map_canvas.h>

// QT autogenerated files
#include "ui_marker_config.h"

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
    void Paint(QPainter* painter, double x, double y, double scale);

    void Transform();

    virtual void UpdateConfig(std::map<std::string, std::string>& params);
    void LoadConfig(const YAML::Node& node, const std::string& path);
    void SaveConfig(YAML::Emitter& emitter, const std::string& path);

    QWidget* GetConfigWidget(QWidget* parent);

    bool SupportsPainting() {
      return true;
    }

  protected:
    void PrintError(const std::string& message);
    void PrintInfo(const std::string& message);
    void PrintWarning(const std::string& message);
    void timerEvent(QTimerEvent *);

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
      std::string text;

      float scale_x;
      float scale_y;
      float scale_z;

      std::string source_frame_;
      swri_transform_util::Transform local_transform;
      
      bool transformed;
    };

    Ui::marker_config ui_;
    QWidget* config_widget_;

    std::string topic_;

    ros::Subscriber marker_sub_;
    bool connected_;
    bool has_message_;

    std::map<std::string, std::map<int, MarkerData> > markers_;

    void handleMessage(const topic_tools::ShapeShifter::ConstPtr& msg);
    void handleMarker(const visualization_msgs::Marker &marker);
    void handleMarkerArray(const visualization_msgs::MarkerArray &markers);
  };
}

#endif  // MAPVIZ_PLUGINS_MARKER_PLUGIN_H_
