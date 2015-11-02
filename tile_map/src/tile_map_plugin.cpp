// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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

#include <tile_map/tile_map_plugin.h>

// QT libraries
#include <QGLWidget>
#include <QPalette>

// ROS libraries
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <swri_transform_util/frames.h>
#include <swri_yaml_util/yaml_util.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(mapviz_plugins, tile_map, tile_map::TileMapPlugin, mapviz::MapvizPlugin)

namespace tile_map
{
  TileMapPlugin::TileMapPlugin() :
    config_widget_(new QWidget()),
    transformed_(false)
  {
    ui_.setupUi(config_widget_);

    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    QPalette p2(ui_.status->palette());
    p2.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p2);

    source_frame_ = swri_transform_util::_wgs84_frame;
    
    QObject::connect(ui_.source_combo, SIGNAL(activated(QString)), this, SLOT(SelectSource(QString)));
  }

  TileMapPlugin::~TileMapPlugin()
  {
  }

  void TileMapPlugin::SelectSource(QString source)
  {
    if (source == "MapQuest (satellite)")
    {
      tile_map_.SetBaseUrl("http://otile1.mqcdn.com/tiles/1.0.0/sat/");
      tile_map_.SetExtension(".jpg");
      tile_map_.SetMaxLevel(18);
    }
    else if (source == "MapQuest (roads)")
    {
      tile_map_.SetBaseUrl("http://otile1.mqcdn.com/tiles/1.0.0/map/");
      tile_map_.SetExtension(".jpg");
      tile_map_.SetMaxLevel(19);
    }
    else if (source == "OpenStreetMap")
    {
      tile_map_.SetBaseUrl("http://tile.openstreetmap.org/");
      tile_map_.SetExtension(".png");
      tile_map_.SetMaxLevel(19);
    }
    else if (source == "Stamen (watercolor)")
    {
      tile_map_.SetBaseUrl("http://tile.stamen.com/watercolor/");
      tile_map_.SetExtension(".jpg");
      tile_map_.SetMaxLevel(19);
      
    }
    else if (source == "Stamen (terrain)")
    {
      tile_map_.SetBaseUrl("http://tile.stamen.com/terrain/");
      tile_map_.SetExtension(".jpg");
      tile_map_.SetMaxLevel(15);
    }
    else if (source == "Stamen (toner)")
    {
      tile_map_.SetBaseUrl("http://tile.stamen.com/toner/");
      tile_map_.SetExtension(".png");
      tile_map_.SetMaxLevel(19);
    }
    else
    {
      tile_map_.SetBaseUrl(source.toStdString());
      tile_map_.SetExtension(".jpg");
      tile_map_.SetMaxLevel(19);
    }

    initialized_ = true;

    canvas_->update();
  }

  void TileMapPlugin::PrintError(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_ERROR("Error: %s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void TileMapPlugin::PrintInfo(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_INFO("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::green);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  void TileMapPlugin::PrintWarning(const std::string& message)
  {
    if (message == ui_.status->text().toStdString())
      return;

    ROS_WARN("%s", message.c_str());
    QPalette p(ui_.status->palette());
    p.setColor(QPalette::Text, Qt::darkYellow);
    ui_.status->setPalette(p);
    ui_.status->setText(message.c_str());
  }

  QWidget* TileMapPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool TileMapPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;

    SelectSource("MapQuest (satellite)");

    return true;
  }

  void TileMapPlugin::Draw(double x, double y, double scale)
  {
    //ROS_ERROR("Draw(%lf, %lf, %lf)", x, y, scale);
    swri_transform_util::Transform to_wgs84;
    if (tf_manager_.GetTransform(source_frame_, target_frame_, to_wgs84))
    {
      //ROS_ERROR("%s -> %s", target_frame_.c_str(), source_frame_.c_str());
      tf::Vector3 center(x, y, 0);
      center = to_wgs84 * center;
      tile_map_.SetView(center.y(), center.x(), scale, canvas_->width(), canvas_->height());
      tile_map_.Draw();
    }
  }

  void TileMapPlugin::Transform()
  {
    swri_transform_util::Transform to_target;
    if (tf_manager_.GetTransform(target_frame_, source_frame_, to_target))
    {
      tile_map_.SetTransform(to_target);
      PrintInfo("OK");
    }
    else
    {
      PrintError("No transform between " + source_frame_ + " and " + target_frame_);
    }
  }

  void TileMapPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (swri_yaml_util::FindValue(node, "custom_sources"))
    {
      const YAML::Node& sources = node["custom_sources"];
      for (uint32_t i = 0; i < sources.size(); i++)
      {
        std::string url;
        sources[i] >> url;
        ui_.source_combo->addItem(QString::fromStdString(url));
      }
    }
    
    if (swri_yaml_util::FindValue(node, "source"))
    {
      std::string source;
      node["source"] >> source;
      
      int index = ui_.source_combo->findText(QString::fromStdString(source), Qt::MatchExactly);
      if (index < 0)
      {
        ui_.source_combo->addItem(QString::fromStdString(source));
        index = ui_.source_combo->findText(QString::fromStdString(source), Qt::MatchExactly);
      }
      
      if (index >= 0)
      {
        ui_.source_combo->setCurrentIndex(index);
      }
      
      SelectSource(QString::fromStdString(source));
    }
  }

  void TileMapPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    if (ui_.source_combo->count() > 5)
    {
      emitter << YAML::Key << "custom_sources" << YAML::Value << YAML::BeginSeq;
      for (int32_t i = 5; i < ui_.source_combo->count(); i++)
      {
        emitter << YAML::Value << ui_.source_combo->itemText(i).toStdString();
      }
      emitter << YAML::EndSeq;
    }
  
    emitter << YAML::Key << "source" << YAML::Value << boost::trim_copy(ui_.source_combo->currentText().toStdString());
  }
}

