#ifndef MAPVIZ_H
#define MAPVIZ_H

// C++ standard libraries
#include <string>
#include <vector>
#include <map>

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

// Auto-generated UI files
#include "ui_mapviz.h"
#include "ui_pluginselect.h"

#include "mapviz_plugin.h"
#include "map_canvas.h"

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
  tf::TransformListener* tf_;

  pluginlib::ClassLoader<mapviz::MapvizPlugin>* loader_;
  MapCanvas* canvas_;
  std::map<QListWidgetItem*,mapviz::MapvizPlugin*> plugins_;

  void Open(const std::string& filename);
  void Save(const std::string& filename);

  mapviz::MapvizPlugin* CreateNewDisplay(const std::string& name, const std::string& type, bool visible, bool collapsed);

  void ClearDisplays();
  void AdjustWindowSize();

  virtual void showEvent(QShowEvent* event);
  virtual void closeEvent(QCloseEvent* event);
};

#endif /* MAPVIZ_H */
