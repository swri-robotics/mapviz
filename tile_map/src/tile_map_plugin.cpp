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
#include <tile_map/tile_source.h>

// QT libraries
#include <QGLWidget>
#include <QInputDialog>
#include <QMessageBox>
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
  std::string TileMapPlugin::BASE_URL_KEY = "base_url";
  std::string TileMapPlugin::COORD_ORDER_KEY = "coord_order";
  std::string TileMapPlugin::CUSTOM_SOURCES_KEY = "custom_sources";
  std::string TileMapPlugin::MAX_ZOOM_KEY = "max_zoom";
  std::string TileMapPlugin::NAME_KEY = "name";
  std::string TileMapPlugin::SOURCE_KEY = "source";
  std::string TileMapPlugin::SUFFIX_KEY = "suffix";
  QString TileMapPlugin::STAMEN_TERRAIN_NAME = "Stamen (terrain)";
  QString TileMapPlugin::STAMEN_TONER_NAME = "Stamen (toner)";
  QString TileMapPlugin::STAMEN_WATERCOLOR_NAME = "Stamen (watercolor)";

  TileMapPlugin::TileMapPlugin() :
    config_widget_(new QWidget()),
    transformed_(false)
  {
    ui_.setupUi(config_widget_);

    tile_sources_[STAMEN_TERRAIN_NAME] = TileSource(STAMEN_TERRAIN_NAME,
                                                    "http://tile.stamen.com/terrain/",
                                                    TileSource::ZXY,
                                                    false,
                                                    15,
                                                    ".png");
    tile_sources_[STAMEN_TONER_NAME] = TileSource(STAMEN_TONER_NAME,
                                                  "http://tile.stamen.com/toner/",
                                                  TileSource::ZXY,
                                                  false,
                                                  19,
                                                  ".png");
    tile_sources_[STAMEN_WATERCOLOR_NAME] = TileSource(STAMEN_WATERCOLOR_NAME,
                                                       "http://tile.stamen.com/watercolor/",
                                                       TileSource::ZXY,
                                                       false,
                                                       19,
                                                       ".jpg");

    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    QPalette p2(ui_.status->palette());
    p2.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p2);

    source_frame_ = swri_transform_util::_wgs84_frame;

    QObject::connect(ui_.delete_button, SIGNAL(clicked()), this, SLOT(DeleteTileSource()));
    QObject::connect(ui_.source_combo, SIGNAL(activated(QString)), this, SLOT(SelectSource(QString)));
    QObject::connect(ui_.save_button, SIGNAL(clicked()), this, SLOT(SaveCustomSource()));
    QObject::connect(ui_.reset_cache_button, SIGNAL(clicked()), this, SLOT(ResetTileCache()));
  }

  TileMapPlugin::~TileMapPlugin()
  {
  }

  void TileMapPlugin::DeleteTileSource()
  {
    int source_index = ui_.source_combo->currentIndex();
    QString current_name = ui_.source_combo->currentText();

    QMessageBox mbox;
    mbox.setText("Are you sure you want to delete the source \"" + current_name + "\"?");
    mbox.setIcon(QMessageBox::Warning);
    mbox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    mbox.setDefaultButton(QMessageBox::Cancel);
    int ret = mbox.exec();

    if (ret == QMessageBox::Ok)
    {
      ui_.source_combo->removeItem(source_index);
      tile_sources_.erase(current_name);
      ui_.source_combo->setCurrentIndex(0);
      SelectSource(ui_.source_combo->currentText());
    }
  }

  void TileMapPlugin::SelectSource(QString source)
  {
    if (source == STAMEN_TERRAIN_NAME ||
        source == STAMEN_WATERCOLOR_NAME ||
        source == STAMEN_TONER_NAME)
    {
      stopCustomEditing();
    }
    else
    {
      startCustomEditing();
    }

    if (tile_sources_.find(source) != tile_sources_.end())
    {
      selectTileSource(tile_sources_[source]);
      initialized_ = true;
    }
    else
    {
      ui_.delete_button->setEnabled(false);
    }
  }

  void TileMapPlugin::SaveCustomSource()
  {
    // If the user is editing a custom source, we want to fill in the default
    // name for it with its current name.
    // Otherwise, they're creating a new custom source, in which case we
    // should leave the default blank.
    QString current_source = ui_.source_combo->currentText();
    QString default_name = "";
    if (tile_sources_.find(current_source) != tile_sources_.end())
    {
      if (tile_sources_[current_source].IsCustom())
      {
        default_name = current_source;
      }
    }
    bool ok;
    QString name = QInputDialog::getText(config_widget_,
                                         tr("Save New Tile Source"),
                                         tr("Tile Source Name:"),
                                         QLineEdit::Normal,
                                         default_name,
                                         &ok);
    name = name.trimmed();
    if (ok && !name.isEmpty())
    {
      TileSource source(name,
                        ui_.base_url_text->text(),
                        static_cast<TileSource::COORD_ORDER>(ui_.coord_order_combo_box->currentIndex()),
                        true,
                        ui_.max_zoom_spin_box->value(),
                        ui_.suffix_text->text());
      int existing_index = ui_.source_combo->findText(name);
      if (existing_index != -1)
      {
        ui_.source_combo->removeItem(existing_index);
      }
      tile_sources_[name] = source;
      ui_.source_combo->addItem(name);
      int new_index = ui_.source_combo->findText(name);
      ui_.source_combo->setCurrentIndex(new_index);
      SelectSource(name);
    }
  }

  void TileMapPlugin::ResetTileCache()
  {
    tile_map_.ResetCache();
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

    SelectSource(STAMEN_TERRAIN_NAME);

    return true;
  }

  void TileMapPlugin::Draw(double x, double y, double scale)
  {
    swri_transform_util::Transform to_wgs84;
    if (tf_manager_.GetTransform(source_frame_, target_frame_, to_wgs84))
    {
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
    if (swri_yaml_util::FindValue(node, CUSTOM_SOURCES_KEY))
    {
      const YAML::Node& sources = node[CUSTOM_SOURCES_KEY];
      YAML::Node::const_iterator source_iter;
      for (source_iter = sources.begin(); source_iter != sources.end(); source_iter++)
      {
        TileSource source(QString::fromStdString(((*source_iter)[NAME_KEY]).as<std::string>()),
                          QString::fromStdString((*source_iter)[BASE_URL_KEY].as<std::string>()),
                          static_cast<TileSource::COORD_ORDER>((*source_iter)[COORD_ORDER_KEY].as<int32_t>()),
                          true,
                          (*source_iter)[MAX_ZOOM_KEY].as<int>(),
                          QString::fromStdString((*source_iter)[SUFFIX_KEY].as<std::string>())
                          );
        tile_sources_[source.GetName()] = source;
        ui_.source_combo->addItem(source.GetName());
      }
    }
    
    if (node[SOURCE_KEY])
    {
      std::string source = node[SOURCE_KEY].as<std::string>();

      int index = ui_.source_combo->findText(QString::fromStdString(source), Qt::MatchExactly);

      if (index >= 0)
      {
        ui_.source_combo->setCurrentIndex(index);
      }
      
      SelectSource(QString::fromStdString(source));
    }
  }

  void TileMapPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    emitter << YAML::Key << CUSTOM_SOURCES_KEY << YAML::Value << YAML::BeginSeq;

    std::map<QString, TileSource>::iterator iter;
    for (iter = tile_sources_.begin(); iter != tile_sources_.end(); iter++)
    {
      if (iter->second.IsCustom())
      {
        emitter << YAML::BeginMap;
        emitter << YAML::Key << BASE_URL_KEY << YAML::Value << iter->second.GetBaseUrl().toStdString();
        emitter << YAML::Key << COORD_ORDER_KEY << YAML::Value << (int) iter->second.GetCoordOrder();
        emitter << YAML::Key << MAX_ZOOM_KEY << YAML::Value << iter->second.GetMaxZoom();
        emitter << YAML::Key << NAME_KEY << YAML::Value << iter->second.GetName().toStdString();
        emitter << YAML::Key << SUFFIX_KEY << YAML::Value << iter->second.GetSuffix().toStdString();
        emitter << YAML::EndMap;
      }
    }
    emitter << YAML::EndSeq;

    emitter << YAML::Key << SOURCE_KEY << YAML::Value << boost::trim_copy(ui_.source_combo->currentText().toStdString());
  }

  void TileMapPlugin::selectTileSource(const TileSource& tile_source)
  {
    tile_map_.SetTileSource(tile_source);
    ui_.base_url_text->setText(tile_source.GetBaseUrl());
    ui_.suffix_text->setText(tile_source.GetSuffix());
    ui_.coord_order_combo_box->setCurrentIndex((int) tile_source.GetCoordOrder());
    ui_.max_zoom_spin_box->setValue(tile_source.GetMaxZoom());
  }

  void TileMapPlugin::startCustomEditing()
  {
    ui_.base_url_text->setEnabled(true);
    ui_.suffix_text->setEnabled(true);
    ui_.coord_order_combo_box->setEnabled(true);
    ui_.delete_button->setEnabled(true);
    ui_.max_zoom_spin_box->setEnabled(true);
    ui_.save_button->setEnabled(true);
  }

  void TileMapPlugin::stopCustomEditing()
  {
    ui_.base_url_text->setEnabled(false);
    ui_.suffix_text->setEnabled(false);
    ui_.coord_order_combo_box->setEnabled(false);
    ui_.delete_button->setEnabled(false);
    ui_.max_zoom_spin_box->setEnabled(false);
    ui_.save_button->setEnabled(false);
  }
}

