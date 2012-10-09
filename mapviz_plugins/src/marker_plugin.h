#ifndef MARKER_PLUGIN_H
#define MARKER_PLUGIN_H

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

    void LoadConfiguration(const YAML::Node& node, const std::string& config_path);
    void SaveConfiguration(YAML::Emitter& emitter, const std::string& config_path);

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

#endif /* MARKER_PLUGIN_H */
