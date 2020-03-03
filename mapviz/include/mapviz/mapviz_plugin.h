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

#ifndef MAPVIZ__MAPVIZ_PLUGIN_H_
#define MAPVIZ__MAPVIZ_PLUGIN_H_

// ROS libraries
#include <swri_transform_util/transform.h>
#include <swri_transform_util/transform_manager.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <mapviz/widgets.h>
#include <yaml-cpp/yaml.h>

// Boost libraries
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

// QT libraries
#include <QWidget>
#include <QGLWidget>
#include <QObject>

// C++ standard libraries
#include <memory>
#include <string>


#include "mapviz/stopwatch.h"

namespace mapviz
{
class MapvizPlugin : public QObject
{
  Q_OBJECT;
public:
  virtual ~MapvizPlugin() {}

  virtual bool Initialize(
      std::shared_ptr<tf2_ros::Buffer> tf_buffer,
      std::shared_ptr<tf2_ros::TransformListener> tf_listener,
      swri_transform_util::TransformManagerPtr tf_manager,
      QGLWidget* canvas)
  {
    tf_buf_ = tf_buffer;
    tf_ = tf_listener;
    tf_manager_ = tf_manager;
    return Initialize(canvas);
  }

  virtual void Shutdown() = 0;

  virtual void ClearHistory() {}

  /**
   * Draws on the Mapviz canvas using OpenGL commands; this will be called
   * before Paint();
   */
  virtual void Draw(double x, double y, double scale) = 0;

  /**
   * Draws on the Mapviz canvas using a QPainter; this is called after Draw().
   * You only need to implement this if you're actually using a QPainter.
   */
  virtual void Paint(QPainter* painter, double x, double y, double scale) {}

  void SetUseLatestTransforms(bool value)
  {
    if (value != use_latest_transforms_) {
      use_latest_transforms_ = value;
      Q_EMIT UseLatestTransformsChanged(use_latest_transforms_);
    }
  }

  void SetName(const std::string& name) { name_ = name; }

  std::string Name() const { return name_; }

  void SetType(const std::string& type) { type_ = type; }

  std::string Type() const { return type_; }

  int DrawOrder() const { return draw_order_; }

  void SetDrawOrder(int order)
  {
    if (draw_order_ != order) {
      draw_order_ = order;
      Q_EMIT DrawOrderChanged(draw_order_);
    }
  }

  virtual void SetNode(rclcpp::Node& node)
  {
    // node_ = node;
    node_ = node.shared_from_this();
  }

  void DrawPlugin(double x, double y, double scale)
  {
    if (visible_ && initialized_) {
      meas_transform_.start();
      Transform();
      meas_transform_.stop();

      meas_draw_.start();
      Draw(x, y, scale);
      meas_draw_.stop();
    }
  }

  void PaintPlugin(QPainter* painter, double x, double y, double scale)
  {
    if (visible_ && initialized_) {
      meas_transform_.start();
      Transform();
      meas_transform_.stop();

      meas_paint_.start();
      Paint(painter, x, y, scale);
      meas_paint_.start();
    }
  }

  void SetTargetFrame(std::string frame_id)
  {
    if (frame_id != target_frame_) {
      target_frame_ = frame_id;

      meas_transform_.start();
      Transform();
      meas_transform_.stop();

      Q_EMIT TargetFrameChanged(target_frame_);
    }
  }

  bool Visible() const { return visible_; }

  void SetVisible(bool visible)
  {
    if (visible_ != visible) {
      visible_ = visible;
      Q_EMIT VisibleChanged(visible_);
    }
  }

  // bool GetTransform(const ros::Time& stamp,
  bool GetTransform(
    const rclcpp::Time& stamp,
    swri_transform_util::Transform& transform,
    bool use_latest_transforms = true)
  {
    if (!initialized_) {
      return false;
    }

    // ros::Time time = stamp;
    rclcpp::Time time = stamp;

    if (use_latest_transforms_ && use_latest_transforms) {
      // time = ros::Time();
      time = rclcpp::Time();
    }

    // ros::Duration elapsed = ros::Time::now() - time;
    rclcpp::Duration elapsed = node_->now() - time;

    // if (time != ros::Time() && elapsed > tf_->getCacheLength())
    if (time != rclcpp::Time() && elapsed > tf_buf_->getCacheLength()) {
      return false;
    }
    if (tf_manager_->GetTransform(
      target_frame_,
      source_frame_,
      tf2::timeFromSec(time.seconds()),
      transform))
    {
      return true;
    } else if (elapsed.seconds() < 0.1) {
      // If the stamped transform failed because it is too recent, find the
      // most recent transform in the cache instead.
      // if (tf_manager_->GetTransform(target_frame_, source_frame_,  ros::Time(), transform))
      if (tf_manager_->GetTransform(
        target_frame_,
        source_frame_,
        tf2::timeFromSec(rclcpp::Time().seconds()),
        transform))
      {
        return true;
      }
    }

    return false;
  }

  bool GetTransform(const std::string& source,
    // const ros::Time& stamp,
    const rclcpp::Time& stamp,
    swri_transform_util::Transform& transform)
  {
    if (!initialized_) {
      return false;
    }

    // ros::Time time = stamp;
    rclcpp::Time time = stamp;

    if (use_latest_transforms_) {
      // time = ros::Time();
      time = rclcpp::Time();
    }

    // ros::Duration elapsed = ros::Time::now() - time;
    rclcpp::Duration elapsed = node_->now() - time;

    // if (time != ros::Time() && elapsed > tf_->getCacheLength())
    if (time != rclcpp::Time() && elapsed > tf_buf_->getCacheLength()) {
      return false;
    }

    if (tf_manager_->GetTransform(
      target_frame_,
      source,
      tf2::timeFromSec(time.seconds()),
      transform))
    {
      return true;
    } else if (elapsed.seconds() < 0.1) {
      // If the stamped transform failed because it is too recent, find the
      // most recent transform in the cache instead.
      // if (tf_manager_->GetTransform(target_frame_, source,  ros::Time(), transform))
      if (tf_manager_->GetTransform(
        target_frame_,
        source,
        tf2::timeFromSec(rclcpp::Time().seconds()),
        transform))
      {
        return true;
      }
    }

    return false;
  }

  virtual void Transform() = 0;

  virtual void LoadConfig(const YAML::Node& load, const std::string& path) = 0;
  virtual void SaveConfig(YAML::Emitter& emitter, const std::string& path) = 0;

  virtual QWidget* GetConfigWidget(QWidget* parent) { return NULL; }

  virtual void PrintError(const std::string& message) = 0;
  virtual void PrintInfo(const std::string& message) = 0;
  virtual void PrintWarning(const std::string& message) = 0;

  void SetIcon(IconWidget* icon) { icon_ = icon; }

  void PrintMeasurements()
  {
    std::string header = type_ + " (" + name_ + ")";
    meas_transform_.printInfo(node_->get_logger(), header + " Transform()");
    meas_paint_.printInfo(node_->get_logger(), header + " Paint()");
    meas_draw_.printInfo(node_->get_logger(), header + " Draw()");
  }

  static void PrintErrorHelper(
    QLabel *status_label,
    const std::string& message,
    double throttle = 0.0);
  static void PrintInfoHelper(
    QLabel *status_label,
    const std::string& message,
    double throttle = 0.0);
  static void PrintWarningHelper(
    QLabel *status_label,
    const std::string& message,
    double throttle = 0.0);

public Q_SLOTS:
  virtual void DrawIcon() {}

  /**
   * Override this to return "true" if you want QPainter support for your
   * plugin.
   */
  virtual bool SupportsPainting()
  {
    return false;
  }

Q_SIGNALS:
  void DrawOrderChanged(int draw_order);
  void SizeChanged();
  void TargetFrameChanged(const std::string& target_frame);
  void UseLatestTransformsChanged(bool use_latest_transforms);
  void VisibleChanged(bool visible);


protected:
  bool initialized_;
  bool visible_;

  QGLWidget* canvas_;
  IconWidget* icon_;

  std::shared_ptr<rclcpp::Node> node_;

  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_;
  swri_transform_util::TransformManagerPtr tf_manager_;

  std::string target_frame_;
  std::string source_frame_;
  std::string type_;
  std::string name_;

  bool use_latest_transforms_;

  int draw_order_;

  virtual bool Initialize(QGLWidget* canvas) = 0;

  MapvizPlugin() :
    initialized_(false),
    visible_(true),
    canvas_(NULL),
    icon_(NULL),
    tf_(),
    target_frame_(""),
    source_frame_(""),
    use_latest_transforms_(false),
    draw_order_(0),
    node_(NULL) {}

  private:
  // Collect basic profiling info to know how much time each plugin
  // spends in Transform(), Paint(), and Draw().
  Stopwatch meas_transform_;
  Stopwatch meas_paint_;
  Stopwatch meas_draw_;
};
typedef std::shared_ptr<MapvizPlugin> MapvizPluginPtr;

// Implementation

inline void MapvizPlugin::PrintErrorHelper(QLabel *status_label, const std::string &message,
                                            double throttle)
{
    if (message == status_label->text().toStdString()) {
      return;
    }

    if (throttle > 0.0) {
        // ROS_ERROR_THROTTLE(throttle, "Error: %s", message.c_str());
        // RCLCPP_ERROR_THROTTLE(
        //  rclcpp::get_logger("mapviz"),
        //  throttle,
        //  "Error: %s",
        //  message.c_str());
        RCLCPP_ERROR(rclcpp::get_logger("mapviz"), "Error: %s", message.c_str());
    } else {
        // ROS_ERROR("Error: %s", message.c_str());
        RCLCPP_ERROR(rclcpp::get_logger("mapviz"), "%s", message.c_str());
    }
    QPalette p(status_label->palette());
    p.setColor(QPalette::Text, Qt::red);
    status_label->setPalette(p);
    status_label->setText(message.c_str());
}

inline void MapvizPlugin::PrintInfoHelper(QLabel *status_label, const std::string &message,
                                          double throttle)
{
    if (message == status_label->text().toStdString()) {
      return;
    }

    if (throttle > 0.0) {
        // ROS_INFO_THROTTLE(throttle, "%s", message.c_str());
        // RCLCPP_INFO_THROTTLE(rclcpp::get_logger("mapviz"), "%s", message.c_str());
        RCLCPP_INFO(rclcpp::get_logger("mapviz"), "%s", message.c_str());
    } else {
        // ROS_INFO("%s", message.c_str());
        RCLCPP_INFO(rclcpp::get_logger("mapviz"), "%s", message.c_str());
    }
    QPalette p(status_label->palette());
    p.setColor(QPalette::Text, Qt::darkGreen);
    status_label->setPalette(p);
    status_label->setText(message.c_str());
}

inline void MapvizPlugin::PrintWarningHelper(QLabel *status_label, const std::string &message,
                                              double throttle)
{
    if (message == status_label->text().toStdString()) {
      return;
    }

    if (throttle > 0.0) {
        // ROS_WARN_THROTTLE(throttle, "%s", message.c_str());
        // RCLCPP_WARN_THROTTLE(rclcpp::get_logger("mapviz"), "%s", message.c_str());
        RCLCPP_WARN(rclcpp::get_logger("mapviz"), "%s", message.c_str());
    } else {
        // ROS_WARN("%s", message.c_str());
        RCLCPP_WARN(rclcpp::get_logger("mapviz"), "%s", message.c_str());
    }
    QPalette p(status_label->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    status_label->setPalette(p);
    status_label->setText(message.c_str());
}

}   // namespace mapviz

#endif  // MAPVIZ__MAPVIZ_PLUGIN_H_

