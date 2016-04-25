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

#include "include/mapviz_plugins/point_click_publisher_plugin.h"
#include <geometry_msgs/PointStamped.h>
#include <swri_yaml_util/yaml_util.h>

#include <boost/shared_ptr.hpp>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
    mapviz_plugins,
    point_click_publisher,
    mapviz_plugins::PointClickPublisherPlugin,
    mapviz::MapvizPlugin)

namespace mapviz_plugins {

  PointClickPublisherPlugin::PointClickPublisherPlugin() :
  config_widget_(new QWidget()),
  canvas_(NULL)
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
    if (canvas_) {
      canvas_->removeEventFilter(&click_filter_);
    }
  }

  bool PointClickPublisherPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = static_cast<mapviz::MapCanvas*>(canvas);
    canvas_->installEventFilter(&click_filter_);

    PrintInfo("Ready.");

    return true;
  }

  void PointClickPublisherPlugin::Draw(double x, double y, double scale)
  {
  }

  void PointClickPublisherPlugin::UpdateConfig(std::map<std::string, std::string>& params)
  {
    if (params.count("topic") > 0)
    {
      ui_.topic->setText(boost::trim_copy(params["topic"]).c_str());
    }
    topicChanged(ui_.topic->text());
    
    if (params.count("output_frame") > 0)
    {
      ui_.outputframe->addItem(QString(params["output_frame"].c_str()));
    }
  }

  void PointClickPublisherPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    std::string tmp;
    if (swri_yaml_util::FindValue(node, "topic"))
    {
      node["topic"] >> tmp;
      ui_.topic->setText(QString(tmp.c_str()));
    }

    topicChanged(ui_.topic->text());

    if (swri_yaml_util::FindValue(node, "output_frame"))
    {
      node["output_frame"] >> tmp;
      ui_.outputframe->addItem(QString(tmp.c_str()));
    }
  }

  void PointClickPublisherPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << "topic" << YAML::Value << ui_.topic->text().toStdString();
    emitter << YAML::Key << "output_frame" << YAML::Value << ui_.outputframe->currentText().toStdString();
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

    if (target_frame_ != output_frame) {
      swri_transform_util::Transform tf;
      tf::Point tfPoint(transformed.x(), transformed.y(), 0.0);
      if (tf_manager_.GetTransform(output_frame, target_frame_, tf)) {
        tfPoint = tf * tfPoint;
      }
      else {
        std::stringstream error;
        error << "Unable to find transform from " << target_frame_ << " to " << output_frame << ".";
        PrintError(error.str());
        return;
      }
      transformed.setX(tfPoint.x());
      transformed.setY(tfPoint.y());
    }

    std::stringstream ss;
    ss << "Point in " << output_frame.c_str() << ": " << transformed.x() << "," << transformed.y();
    PrintInfo(ss.str());

    boost::shared_ptr<geometry_msgs::PointStamped> stamped = boost::make_shared<geometry_msgs::PointStamped>();
    stamped->header.frame_id = output_frame;
    stamped->header.stamp = ros::Time::now();
    stamped->point.x = transformed.x();
    stamped->point.y = transformed.y();
    stamped->point.z = 0.0;

    point_publisher_.publish(stamped);
  }

  void PointClickPublisherPlugin::SetNode(const ros::NodeHandle& node)
  {
    mapviz::MapvizPlugin::SetNode(node);

    // We override this method so that we can initialize our publisher after
    // our node has been set, ensuring that it's in mapviz's namespace.
    topicChanged(ui_.topic->text());
  }

  void PointClickPublisherPlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
    {
      return;
    }

    ROS_ERROR("Error: %s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void PointClickPublisherPlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
    {
      return;
    }

    ROS_INFO("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void PointClickPublisherPlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
    {
      return;
    }

    ROS_WARN("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }


  void PointClickPublisherPlugin::topicChanged(const QString& topic)
  {
    std::stringstream ss;
    ss << "Publishing points to topic: " << topic.toStdString().c_str();
    PrintInfo(ss.str());

    if (!topic.isEmpty())
    {
      point_publisher_ = node_.advertise<geometry_msgs::PointStamped>(topic.toStdString(), 1000);
    }
  }

  void PointClickPublisherPlugin::updateFrames()
  {
    std::vector<std::string> frames;
    tf_->getFrameStrings(frames);

    if (ui_.outputframe->count() >= 0 &&
        static_cast<size_t>(ui_.outputframe->count()) == frames.size())
    {
      bool changed = false;
      for (size_t i = 0; i < frames.size(); i++)
      {
        if (frames[i] != ui_.outputframe->itemText(i).toStdString())
        {
          changed = true;
        }
      }

      if (!changed)
        return;
    }

    std::string current_output = ui_.outputframe->currentText().toStdString();

    ui_.outputframe->clear();
    for (size_t i = 0; i < frames.size(); i++)
    {
      ui_.outputframe->addItem(frames[i].c_str());
    }

    if (current_output != "")
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
}
