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

#ifndef MAPVIZ_MAPVIZ_PLUGIN_H_
#define MAPVIZ_MAPVIZ_PLUGIN_H_

// C++ standard libraries
#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

// QT libraries
#include <QWidget>
#include <QGLWidget>
#include <QObject>

// ROS libraries
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <transform_util/transform.h>
#include <transform_util/transform_manager.h>
#include <yaml-cpp/yaml.h>

namespace mapviz
{
  class MapvizPlugin : public QObject
  {
  public:
    virtual ~MapvizPlugin() {}

    virtual bool Initialize(
        boost::shared_ptr<tf::TransformListener> tf_listener,
        QGLWidget* canvas)
    {
      tf_ = tf_listener;
      tf_manager_.Initialize(tf_);
      return Initialize(canvas);
    }

    virtual void Shutdown() = 0;

    virtual void Draw(double x, double y, double scale) = 0;

    void SetUseLatestTransforms(bool value)
    {
      use_latest_transforms_ = value;
    }

    void SetName(const std::string& name) { name_ = name; }

    std::string Name() const { return name_; }

    void SetType(const std::string& type) { type_ = type; }

    std::string Type() const { return type_; }

    int DrawOrder() const { return draw_order_; }

    void SetDrawOrder(int order)
    {
      draw_order_ = order;
    }

    void SetNode(const ros::NodeHandle& node)
    {
      node_ = node;
    }

    void DrawPlugin(double x, double y, double scale)
    {
      if (visible_ && initialized_)
      {
        Transform();

        Draw(x, y, scale);
      }
    }

    void SetTargetFrame(std::string frame_id)
    {
      if (frame_id != target_frame_)
      {
        target_frame_ = frame_id;
        Transform();
      }
    }

    bool Visible() const { return visible_; }

    void SetVisible(bool visible)
    {
      visible_ = visible;
    }

    bool GetTransform(const ros::Time& stamp, transform_util::Transform& transform)
    {
      if (!initialized_)
        return false;

      ros::Time time = stamp;

      if (use_latest_transforms_)
      {
        time = ros::Time();
      }

      ros::Duration elapsed = ros::Time::now() - time;

      if (time != ros::Time() && elapsed > tf_->getCacheLength())
      {
        return false;
      }

      if (tf_manager_.GetTransform(target_frame_, source_frame_, time, transform))
      {
        return true;
      }
      else if (elapsed.toSec() < 0.1)
      {
        // If the stamped transform failed because it is too recent, find the
        // most recent transform in the cache instead.
        if (tf_manager_.GetTransform(target_frame_, source_frame_,  ros::Time(), transform))
        {
          return true;
        }
      }

      return false;
    }
    
    bool GetTransform(const std::string& source, const ros::Time& stamp, transform_util::Transform& transform)
    {
      if (!initialized_)
        return false;

      ros::Time time = stamp;

      if (use_latest_transforms_)
      {
        time = ros::Time();
      }

      ros::Duration elapsed = ros::Time::now() - time;

      if (time != ros::Time() && elapsed > tf_->getCacheLength())
      {
        return false;
      }

      if (tf_manager_.GetTransform(target_frame_, source, time, transform))
      {
        return true;
      }
      else if (elapsed.toSec() < 0.1)
      {
        // If the stamped transform failed because it is too recent, find the
        // most recent transform in the cache instead.
        if (tf_manager_.GetTransform(target_frame_, source,  ros::Time(), transform))
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

  protected:
    bool initialized_;
    bool visible_;

    QGLWidget* canvas_;

    ros::NodeHandle node_;

    boost::shared_ptr<tf::TransformListener> tf_;
    transform_util::TransformManager tf_manager_;
    
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
      tf_(),
      target_frame_(""),
      source_frame_(""),
      use_latest_transforms_(false),
      draw_order_(0) {}
  };
  typedef boost::shared_ptr<MapvizPlugin> MapvizPluginPtr;
}

#endif  // MAPVIZ_MAPVIZ_PLUGIN_H_

