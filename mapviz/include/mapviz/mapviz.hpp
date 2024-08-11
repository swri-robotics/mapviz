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

#ifndef MAPVIZ__MAPVIZ_HPP_
#define MAPVIZ__MAPVIZ_HPP_

// QT libraries
#include <QtGui/QtGui>
#include <QDialog>
#include <QMenu>
#include <QTimer>
#include <QString>
#include <QShowEvent>
#include <QCloseEvent>
#include <QListWidgetItem>
#include <QModelIndex>
#include <QColor>
#include <QWidget>
#include <QStringList>
#include <QMainWindow>
#include <QShortcut>

#include <swri_transform_util/transform_manager.h>
#include <mapviz_interfaces/srv/add_mapviz_display.hpp>  // Service
#include <mapviz/mapviz_plugin.h>
#include <mapviz/map_canvas.h>
#include <mapviz/video_writer.h>

// ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <std_srvs/srv/empty.hpp>

// C++ standard libraries
#include <string>
#include <vector>
#include <map>
#include <memory>

// Auto-generated UI files
#include "ui_mapviz.h"
#include "ui_pluginselect.h"


#include "mapviz/stopwatch.h"

namespace mapviz
{
class Mapviz : public QMainWindow
{
  Q_OBJECT

public:
  Mapviz(bool is_standalone,
    int argc,
    char** argv,
    QWidget *parent = 0,
    Qt::WindowFlags flags = Qt::WindowFlags());
  ~Mapviz();

  rclcpp::Node::SharedPtr GetNode();

  void Initialize();

public Q_SLOTS:
  void AutoSave();
  void OpenConfig();
  void SaveConfig();
  void ClearConfig();
  void SelectNewDisplay();
  void RemoveDisplay();
  void RemoveDisplay(QListWidgetItem* item);
  void RenameDisplay();
  void RenameDisplay(QListWidgetItem* item);
  void ReorderDisplays();
  void FixedFrameSelected(const QString& text);
  void TargetFrameSelected(const QString& text);
  void ToggleUseLatestTransforms(bool on);
  void UpdateFrames();
  void SpinOnce();
  void UpdateSizeHints();
  void ToggleConfigPanel(bool on);
  void ToggleStatusBar(bool on);
  void ToggleCaptureTools(bool on);
  void ToggleFixOrientation(bool on);
  void ToggleRotate90(bool on);
  void ToggleEnableAntialiasing(bool on);
  void ToggleShowPlugin(QListWidgetItem* item, bool visible);
  void ToggleRecord(bool on);
  void SetImageTransport(QAction* transport_action);
  void UpdateImageTransportMenu();
  void CaptureVideoFrame();
  void StopRecord();
  void Screenshot();
  void Force720p(bool on);
  void Force480p(bool on);
  void SetResizable(bool on);
  void SelectBackgroundColor(const QColor &color);
  void SetCaptureDirectory();
  void Hover(double x, double y, double scale);
  void Recenter();
  void HandleProfileTimer();
  void ClearHistory();

Q_SIGNALS:
  /**
   * Emitted every time a frame is grabbed when Mapviz is in video recording
   * mode, typically at a rate of 30 FPS.
   * Note that the QImage emitted says its format is ARGB, but its pixel
   * order is actually BGRA.
   */
  void FrameGrabbed(QImage);
  void ImageTransportChanged();

protected:
  void Open(const std::string& filename);
  void Save(const std::string& filename);

  MapvizPluginPtr CreateNewDisplay(
      const std::string& name,
      const std::string& type,
      bool visible,
      bool collapsed,
      int draw_order = 0);

  void AddDisplay(
    const mapviz_interfaces::srv::AddMapvizDisplay::Request::SharedPtr req,
    mapviz_interfaces::srv::AddMapvizDisplay::Response::SharedPtr resp);

  void ClearDisplays();
  void AdjustWindowSize();

  QString GetDefaultConfigPath();

  virtual void showEvent(QShowEvent* event);
  virtual void closeEvent(QCloseEvent* event);

  static const QString ROS_WORKSPACE_VAR;
  static const QString MAPVIZ_CONFIG_FILE;
  static const char IMAGE_TRANSPORT_PARAM[];

  Ui::mapviz ui_;

  QMenu* image_transport_menu_;

  QTimer frame_timer_;
  QTimer spin_timer_;
  QTimer save_timer_;
  QTimer record_timer_;
  QTimer profile_timer_;

  QLabel* xy_pos_label_;
  QLabel* lat_lon_pos_label_;

  QWidget* spacer1_;
  QWidget* spacer2_;
  QWidget* spacer3_;
  QPushButton* recenter_button_;
  QPushButton* rec_button_;
  QPushButton* stop_button_;
  QPushButton* screenshot_button_;

  int    argc_;
  char** argv_;

  bool is_standalone_;
  bool initialized_;
  bool force_720p_;
  bool force_480p_;
  bool resizable_;
  QColor background_;

  std::string capture_directory_;
  QThread video_thread_;
  VideoWriter* vid_writer_;

  bool updating_frames_;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Service<mapviz_interfaces::srv::AddMapvizDisplay>::SharedPtr add_display_srv_;
  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_;
  swri_transform_util::TransformManagerPtr tf_manager_;

  pluginlib::ClassLoader<MapvizPlugin>* loader_;
  MapCanvas* canvas_;
  std::map<QListWidgetItem*, MapvizPluginPtr> plugins_;

  Stopwatch meas_spin_;
};
}   // namespace mapviz

#endif  // MAPVIZ__MAPVIZ_HPP_
