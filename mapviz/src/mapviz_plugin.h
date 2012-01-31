#ifndef MAPVIZ_PLUGIN_H
#define MAPVIZ_PLUGIN_H

// C++ standard libraries
#include <string>

// QT libraries
#include <QWidget>
#include <QGLWidget>
#include <QObject>

// ROS libraries
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>

namespace mapviz
{
  class MapvizPlugin : public QObject
  {
  public:

    virtual ~MapvizPlugin() {}

    virtual bool Initialize(QGLWidget* canvas) = 0;
    virtual void Shutdown() = 0;
    
    virtual void Draw(double x, double y, double scale) = 0;

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
        UpdateTransform(false);

        if (!no_transform_ || ignore_transform_)
        {
          Draw(x, y, scale);
          PrintInfo("OK");
        }
      }
    }

    void SetTargetFrame(std::string frame_id)
    {
      if (frame_id != target_frame_)
      {
        target_frame_ = frame_id;
        UpdateTransform(true);
      }
    }

    bool Visible() const { return visible_; }

    void SetVisible(bool visible)
    {
      visible_ = visible;
    }
    
    void UpdateTransform(bool force)
    {
      if (!initialized_ || ignore_transform_)
        return;

      no_transform_ = true;
      tf::StampedTransform newTransform;
      try
      {
        transform_listener_.lookupTransform(target_frame_, source_frame_, ros::Time(0), newTransform);
      }
      catch (tf::LookupException& e)
      {
        PrintError(e.what());
        return;
      }
      catch (tf::ConnectivityException& e)
      {
        PrintError(e.what());
        return;
      }
      catch (tf::ExtrapolationException& e)
      {
        PrintError(e.what());;
        return;
      }
      catch (...)
      {
        PrintError("Failed to lookup transform");
        return;
      }

      no_transform_ = false;
      if (
        transform_.getBasis() == newTransform.getBasis() &&
        transform_.getOrigin() == newTransform.getOrigin() &&
        transform_.getRotation() == newTransform.getRotation())
      {
        if (force)
        {
          Transform();
        }
      }
      else
      {
       transform_ = newTransform;
       Transform();
      }
    }

    virtual void Transform() = 0;
    
    virtual void LoadConfiguration(const YAML::Node& load, const std::string& config_path) = 0;
    virtual void SaveConfiguration(YAML::Emitter& emitter, const std::string& config_path) = 0;

    virtual QWidget* GetConfigWidget(QWidget* parent) { return NULL; }

    virtual void PrintError(const std::string& message) = 0;
    virtual void PrintInfo(const std::string& message) = 0;
    virtual void PrintWarning(const std::string& message) = 0;

  protected:
    bool initialized_;
    bool visible_;
    bool no_transform_;
    bool ignore_transform_;

    ros::NodeHandle node_;

    tf::Transform transform_;

    tf::TransformListener transform_listener_;
    std::string target_frame_;
    std::string source_frame_;
    std::string type_;
    std::string name_;

    int draw_order_;

    MapvizPlugin() : initialized_(false), visible_(true), no_transform_(true), ignore_transform_(false), target_frame_(""), source_frame_(""), draw_order_(0) {}
  };
}

#endif /* MAPVIZ_PLUGIN_H */

