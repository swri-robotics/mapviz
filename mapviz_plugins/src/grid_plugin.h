#ifndef GRID_PLUGIN_H
#define GRID_PLUGIN_H

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
    
    void LoadConfiguration(const YAML::Node& node);
    void SaveConfiguration(YAML::Emitter& emitter);

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
    QGLWidget* canvas_;
  
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

    std::list<tf::Point> top_points_;
    std::list<tf::Point> bottom_points_;
    std::list<tf::Point> left_points_;
    std::list<tf::Point> right_points_;

    std::list<tf::Point> transformed_top_points_;
    std::list<tf::Point> transformed_bottom_points_;
    std::list<tf::Point> transformed_left_points_;
    std::list<tf::Point> transformed_right_points_;

    void RecalculateGrid();
    void Transform(std::list<tf::Point>& src, std::list<tf::Point>& dst);
  };
}

#endif /* GRID_PLUGIN_H */
