// *****************************************************************************
//
// Copyright (c) 2014-2020, Southwest Research Institute® (SwRI®)
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

#include <mapviz_plugins/point_click_publisher_plugin.h>
#include <swri_transform_util/frames.h>
#include <tf2/transform_datatypes.h>

// Declare plugin
#include <pluginlib/class_list_macros.hpp>

// C++ Standard Libraries
#include <memory>
#include <string>
#include <vector>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::PointClickPublisherPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  PointClickPublisherPlugin::PointClickPublisherPlugin()
  : MapvizPlugin()
  , ui_()
  , config_widget_(new QWidget())
  , canvas_(nullptr)
  {
    ui_.setupUi(config_widget_);

    connect(&click_filter_, SIGNAL(pointClicked(const QPointF&)),
            this, SLOT(pointClicked(const QPointF&)));
    connect(ui_.topic, SIGNAL(textEdited(const QString&)),
            this, SLOT(topicChanged(const QString&)));

    frame_timer_.start(1000);
    connect(&frame_timer_, SIGNAL(timeout()), this, SLOT(updateFrames()));
  }

  PointClickPublisherPlugin::~PointClickPublisherPlugin()
  {
    if (canvas_)
    {
      canvas_->removeEventFilter(&click_filter_);
    }
  }

  bool PointClickPublisherPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = dynamic_cast<mapviz::MapCanvas*>(canvas);
    canvas_->installEventFilter(&click_filter_);

    PrintInfo("Ready.");

    return true;
  }

  void PointClickPublisherPlugin::Draw(double x, double y, double scale)
  {
  }

  void PointClickPublisherPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    std::string tmp;
    if (node["topic"])
    {
      tmp = node["topic"].as<std::string>();
      ui_.topic->setText(QString(tmp.c_str()));
      topicChanged(ui_.topic->text());
    }

    if (node["output_frame"])
    {
      tmp = node["output_frame"].as<std::string>();
      ui_.outputframe->addItem(QString(tmp.c_str()));
    }
  }

  void PointClickPublisherPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "topic" << YAML::Value << ui_.topic->text().toStdString();
    emitter << YAML::Key
      << "output_frame"
      << YAML::Value
      << ui_.outputframe->currentText().toStdString();
  }

  QWidget* PointClickPublisherPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }


  void PointClickPublisherPlugin::pointClicked(const QPointF& point)
  {
    QPointF transformed = canvas_->MapGlCoordToFixedFrame(point);

    std::string output_frame = ui_.outputframe->currentText().toStdString();

    if (target_frame_ != output_frame)
    {
      swri_transform_util::Transform tf;
      tf2::Vector3 tfPoint(transformed.x(), transformed.y(), 0.0);
      if (tf_manager_->GetTransform(output_frame, target_frame_, tf))
      {
        tfPoint = tf * tfPoint;
      } else {
        std::stringstream error;
        error << "Unable to find transform from " << target_frame_ << " to " << output_frame << ".";
        PrintError(error.str());
        return;
      }
      transformed.setX(tfPoint.x());
      transformed.setY(tfPoint.y());
    }

    std::unique_ptr<geometry_msgs::msg::PointStamped> stamped =
      std::make_unique<geometry_msgs::msg::PointStamped>();
    stamped->header.frame_id = output_frame;
    stamped->header.stamp = node_->get_clock()->now();
    stamped->point.x = transformed.x();
    stamped->point.y = transformed.y();
    stamped->point.z = 0.0;

    std::stringstream ss;
    ss << "Point in " << output_frame.c_str() << ": " << transformed.x() << "," << transformed.y();

    // Only publish if this plugin is visible
    if(this->Visible())
    {
      point_publisher_->publish(*stamped);
    }
    else
    {
      ss << " (but not publishing since plugin is hidden)";
    }
    
    PrintInfo(ss.str());
  }

  void PointClickPublisherPlugin::SetNode(rclcpp::Node& node)
  {
    mapviz::MapvizPlugin::SetNode(node);

    // We override this method so that we can initialize our publisher after
    // our node has been set, ensuring that it's in mapviz's namespace.
    topicChanged(ui_.topic->text());
  }

  void PointClickPublisherPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void PointClickPublisherPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void PointClickPublisherPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }


  void PointClickPublisherPlugin::topicChanged(const QString& topic)
  {
    std::stringstream ss;
    ss << "Publishing points to topic: " << topic.toStdString().c_str();
    PrintInfo(ss.str());

    if (!topic.isEmpty())
    {
      point_publisher_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
          topic.toStdString(), rclcpp::QoS(1000));
    }
  }

  void PointClickPublisherPlugin::updateFrames()
  {
    std::vector<std::string> frames;
    tf_buf_->_getFrameStrings(frames);

    bool supports_wgs84 = tf_manager_->SupportsTransform(
        swri_transform_util::_local_xy_frame,
        swri_transform_util::_wgs84_frame);

    if (supports_wgs84)
    {
      frames.push_back(swri_transform_util::_wgs84_frame);
    }

    if (ui_.outputframe->count() >= 0 &&
        static_cast<size_t>(ui_.outputframe->count()) == frames.size())
    {
      bool changed = false;
      for (size_t i = 0; i < frames.size(); i++)
      {
        if (frames[i] != ui_.outputframe->itemText(static_cast<int>(i)).toStdString())
        {
          changed = true;
        }
      }

      if (!changed)
        return;
    }

    std::string current_output = ui_.outputframe->currentText().toStdString();

    ui_.outputframe->clear();
    for (auto & frame : frames)
    {
      ui_.outputframe->addItem(frame.c_str());
    }

    if (!current_output.empty())
    {
      int index = ui_.outputframe->findText(current_output.c_str());
      if (index < 0)
      {
        ui_.outputframe->addItem(current_output.c_str());
      }

      index = ui_.outputframe->findText(current_output.c_str());
      ui_.outputframe->setCurrentIndex(index);
    }
  }
}   // namespace mapviz_plugins
