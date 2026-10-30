// *****************************************************************************
//
// Copyright (c) 2015-2020, Southwest Research Institute® (SwRI®)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <tile_map/tile_map_plugin.hpp>

// QT libraries
#include <QOpenGLWidget>
#include <QInputDialog>
#include <QMessageBox>
#include <QUrl>
#include <QNetworkRequest>
#include <QDateTime>
#include <QFontMetrics>
#include <QPalette>

#include <algorithm>
#include <cmath>

#include <mapviz/mapviz_plugin.hpp>
#include <tile_map/tile_source.hpp>
#include <tile_map/bing_source.hpp>
#include <tile_map/carto_source.hpp>
#include <tile_map/stadia_source.hpp>
#include <tile_map/wmts_source.hpp>

// ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.hpp>
#include "swri_math_util/constants.h"
#include "swri_transform_util/earth_constants.h"
#include "swri_transform_util/frames.h"

// Declare plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tile_map::TileMapPlugin, mapviz::MapvizPlugin)

namespace tile_map
{
std::string TileMapPlugin::BASE_URL_KEY = "base_url";  // NOLINT(runtime/string)
std::string TileMapPlugin::BING_API_KEY = "bing_api_key";  // NOLINT(runtime/string)
std::string TileMapPlugin::CARTO_API_KEY = "carto_api_key";  // NOLINT(runtime/string)
std::string TileMapPlugin::STADIA_API_KEY = "stadia_api_key";  // NOLINT(runtime/string)
std::string TileMapPlugin::CUSTOM_SOURCES_KEY = "custom_sources";  // NOLINT(runtime/string)
std::string TileMapPlugin::MAX_ZOOM_KEY = "max_zoom";  // NOLINT(runtime/string)
std::string TileMapPlugin::NAME_KEY = "name";  // NOLINT(runtime/string)
std::string TileMapPlugin::SOURCE_KEY = "source";  // NOLINT(runtime/string)
std::string TileMapPlugin::TYPE_KEY = "type";  // NOLINT(runtime/string)
QString TileMapPlugin::BING_NAME = "Bing Maps (terrain)";
QString TileMapPlugin::CARTO_NAME = "Carto";
QString TileMapPlugin::STAMEN_TERRAIN_NAME = "Stamen (terrain)";
QString TileMapPlugin::STAMEN_TONER_NAME = "Stamen (toner)";
QString TileMapPlugin::STAMEN_WATERCOLOR_NAME = "Stamen (watercolor)";
QString TileMapPlugin::OSM_NAME = "OpenStreetMap";
QString TileMapPlugin::USGS_NAME = "USGS Satellite";

// How long a failed tile request keeps the status label.  There is no
// per-tile success signal, so the message ages out rather than being cleared.
constexpr qint64 TILE_ERROR_TIMEOUT_MS = 5000;

// Size of the attribution text, its distance from the canvas edges, and the
// space between it and the edge of its background.
constexpr int ATTRIBUTION_FONT_PIXELS = 11;
constexpr int ATTRIBUTION_MARGIN = 4;
constexpr int ATTRIBUTION_PADDING = 3;

namespace
{
/// The area within half the canvas diagonal of the center, so the bounds
/// still cover the view when the map is rotated.  Views that cross the
/// antimeridian are clipped at it.
ViewBounds ViewAround(
  double latitude, double longitude, double scale,
  int32_t width, int32_t height, int32_t level)
{
  // scale is in meters per pixel.
  const double meters_per_degree = swri_transform_util::_earth_equator_circumference / 360.0;
  const double radius = 0.5 * scale * std::hypot(width, height) / meters_per_degree;
  const double cos_lat = std::cos(latitude / swri_math_util::_rad_2_deg);

  ViewBounds view;
  view.level = level;
  view.south = std::max(-90.0, latitude - radius);
  view.north = std::min(90.0, latitude + radius);
  if (cos_lat > 0.0 && radius / cos_lat < 180.0) {
    view.west = std::max(-180.0, longitude - radius / cos_lat);
    view.east = std::min(180.0, longitude + radius / cos_lat);
  }
  return view;
}
}  // namespace

TileMapPlugin::TileMapPlugin()
: MapvizPlugin()
  , ui_()
  , config_widget_(new QWidget())
  , tile_error_time_(0)
  , transform_ok_(false)
  , dirty_(false)
  , transformed_(false)
  , last_center_x_(0.0)
  , last_center_y_(0.0)
  , last_scale_(0.0)
  , last_height_(0)
  , last_width_(0)
{
  ui_.setupUi(config_widget_);

  // The attribution for each built-in source is the credit line that the
  // provider's terms of use require on the map; Bing works out its own from
  // the imagery on screen.
  tile_sources_[CARTO_NAME] =
    std::make_shared<CartoSource>(
    CARTO_NAME,
    "https://basemaps.cartocdn.com/rastertiles/voyager/{level}/{x}/{y}.png",
    false,
    19);
  tile_sources_[CARTO_NAME]->SetAttribution("© OpenStreetMap contributors, © CARTO");
  std::shared_ptr<StadiaSource> stamen_terrain = std::make_shared<StadiaSource>(
    STAMEN_TERRAIN_NAME,
    "https://tiles.stadiamaps.com/tiles/stamen_terrain/{level}/{x}/{y}.png",
    false,
    19);
  stamen_terrain->SetAttribution(
    "© Stadia Maps © Stamen Design © OpenMapTiles © OpenStreetMap");
  tile_sources_[STAMEN_TERRAIN_NAME] = stamen_terrain;
  std::shared_ptr<StadiaSource> stamen_toner = std::make_shared<StadiaSource>(
    STAMEN_TONER_NAME,
    "https://tiles.stadiamaps.com/tiles/stamen_toner/{level}/{x}/{y}.png",
    false,
    19);
  stamen_toner->SetAttribution(
    "© Stadia Maps © Stamen Design © OpenMapTiles © OpenStreetMap");
  tile_sources_[STAMEN_TONER_NAME] = stamen_toner;
  std::shared_ptr<StadiaSource> stamen_watercolor = std::make_shared<StadiaSource>(
    STAMEN_WATERCOLOR_NAME,
    "https://tiles.stadiamaps.com/tiles/stamen_watercolor/{level}/{x}/{y}.jpg",
    false,
    19);
  // Unlike the other Stamen styles, Watercolor isn't built from OpenMapTiles.
  stamen_watercolor->SetAttribution("© Stadia Maps © Stamen Design © OpenStreetMap");
  tile_sources_[STAMEN_WATERCOLOR_NAME] = stamen_watercolor;
  tile_sources_[OSM_NAME] =
    std::make_shared<WmtsSource>(
    OSM_NAME,
    "https://tile.openstreetmap.org/{level}/{x}/{y}.png",
    false,
    19);
  tile_sources_[OSM_NAME]->SetAttribution("© OpenStreetMap contributors");
  tile_sources_[USGS_NAME] =
    std::make_shared<WmtsSource>(
    USGS_NAME,
    "https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryOnly/MapServer/WMTS/tile/"
    "1.0.0/USGSImageryOnly/default/default028mm/{level}/{y}/{x}.png",
    false,
    19);
  // USGS imagery is public domain; this is the credit the service asks for.
  tile_sources_[USGS_NAME]->SetAttribution("USDA, USGS The National Map: Orthoimagery");
  std::shared_ptr<BingSource> bing = std::make_shared<BingSource>(BING_NAME);
  tile_sources_[BING_NAME] = bing;

  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Window, Qt::white);
  config_widget_->setPalette(p);

  QPalette p2(ui_.status->palette());
  p2.setColor(QPalette::Text, Qt::red);
  ui_.status->setPalette(p2);

  source_frame_ = swri_transform_util::_wgs84_frame;

  QObject::connect(
    bing.get(), SIGNAL(ErrorMessage(const std::string&)),
    this, SLOT(PrintError(const std::string&)));
  QObject::connect(
    bing.get(), SIGNAL(InfoMessage(const std::string&)),
    this, SLOT(PrintInfo(const std::string&)));
  QObject::connect(ui_.delete_button, SIGNAL(clicked()), this, SLOT(DeleteTileSource()));
  QObject::connect(
    ui_.source_combo, SIGNAL(textActivated(const QString&)), this,
    SLOT(SelectSource(const QString&)));
  QObject::connect(ui_.save_button, SIGNAL(clicked()), this, SLOT(SaveCustomSource()));
  QObject::connect(ui_.reset_cache_button, SIGNAL(clicked()), this, SLOT(ResetTileCache()));
  QObject::connect(ui_.test_button, SIGNAL(clicked()), this, SLOT(TestTileSource()));
  // Without these, typing a URL into an enabled-looking field did nothing at
  // all and gave no sign that Save was required to apply it.
  QObject::connect(
    ui_.base_url_text, SIGNAL(textEdited(const QString&)),
    this, SLOT(SourceEdited()));
  QObject::connect(
    ui_.max_zoom_spin_box, SIGNAL(valueChanged(int)),
    this, SLOT(SourceEdited()));
  QObject::connect(
    &test_network_manager_, SIGNAL(finished(QNetworkReply*)),
    this, SLOT(HandleTestReply(QNetworkReply*)));
  // The image cache is created once by TileMapView and outlives every source
  // set on it, so this connection is made a single time.
  QObject::connect(
    tile_map_.GetImageCache().get(),
    SIGNAL(RequestFailed(QString,QString)),  // NOLINT(whitespace/comma)
    this, SLOT(HandleTileFailure(QString,QString)));  // NOLINT(whitespace/comma)

  UpdateControlState();
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

  if (ret == QMessageBox::Ok) {
    ui_.source_combo->removeItem(source_index);
    tile_sources_.erase(current_name);
    ui_.source_combo->setCurrentIndex(0);
    SelectSource(ui_.source_combo->currentText());
  }
}

void TileMapPlugin::SelectSource(const QString & source)
{
  std::map<QString, std::shared_ptr<TileSource>>::iterator iter = tile_sources_.find(source);

  if (iter != tile_sources_.end()) {
    selectTileSource(iter->second);
    initialized_ = true;
  }

  // Switching sources discards whatever was typed but not saved.
  dirty_ = false;
  tile_error_.clear();
  tile_error_time_ = 0;

  UpdateControlState();
  UpdateStatus();
}

std::shared_ptr<TileSource> TileMapPlugin::CurrentSource() const
{
  auto iter = tile_sources_.find(ui_.source_combo->currentText());
  if (iter == tile_sources_.end()) {
    return {};
  }
  return iter->second;
}

bool TileMapPlugin::IsDirty() const
{
  return dirty_;
}

void TileMapPlugin::SourceEdited()
{
  std::shared_ptr<TileSource> source = CurrentSource();

  // Editing an API key applies on Save and is not a "source" edit, but it
  // still has to enable Save so the key can be submitted.
  dirty_ = true;

  if (source && !source->IsCustom() &&
    source->GetType() != BingSource::BING_TYPE &&
    source->GetType() != CartoSource::CARTO_TYPE &&
    source->GetType() != StadiaSource::STADIA_TYPE)
  {
    // Built-in WMTS sources are read-only; nothing to apply.
    dirty_ = false;
  }

  UpdateControlState();
  UpdateStatus();
}

void TileMapPlugin::UpdateControlState()
{
  std::shared_ptr<TileSource> source = CurrentSource();
  // No entry in tile_sources_ means the "Custom WMTS Source..." placeholder is
  // selected: the user is composing a brand new source.
  const bool is_new_custom = !source;
  const bool is_custom = source && source->IsCustom();
  const bool is_bing = source && source->GetType() == BingSource::BING_TYPE;
  const bool is_carto = source && source->GetType() == CartoSource::CARTO_TYPE;
  const bool is_stadia = source && source->GetType() == StadiaSource::STADIA_TYPE;
  const bool is_key = is_bing || is_carto || is_stadia;
  const bool editable = is_new_custom || is_custom || is_key;

  ui_.url_label->setText(
    is_bing ? "Bing API Key:" :
    (is_carto ? "Carto API Key:" :
    (is_stadia ? "Stadia API Key:" : "Base URL:")));
  // The placeholder and tooltip from the .ui describe a URL template, which
  // is misleading when the field holds an API key.
  ui_.base_url_text->setPlaceholderText(
    is_key ? "Paste API key, then click Save" :
    "http://localhost/osm_tiles/{level}/{x}/{y}.png");
  ui_.base_url_text->setToolTip(
    is_carto ? "Carto basemaps API key, from carto.com/basemaps/apikey." :
    (is_key ? "API key for this tile source." :
    "Tile URL template.  {level} (or {z}), {x} and {y} are replaced with the tile "
    "coordinates.  file:// URLs work for tiles on disk."));
  // The ellipsis is the convention for "this opens a dialog", which saving a
  // custom source does and submitting an API key does not.
  ui_.save_button->setText(is_key ? "Save" : "Save...");

  ui_.base_url_text->setEnabled(editable);
  ui_.max_zoom_spin_box->setEnabled(is_new_custom || is_custom);
  ui_.delete_button->setEnabled(is_custom);
  // Save is the only thing that applies a custom URL, so leave it live while
  // there is something to apply and grey it out once there is not.
  ui_.save_button->setEnabled(editable && (IsDirty() || is_new_custom));
  // Testing a source the user has not applied yet would probe the old URL.
  ui_.test_button->setEnabled(source && !IsDirty());
}

void TileMapPlugin::UpdateStatus()
{
  // A tile error is the most specific thing we know, but it must not stick
  // around after the source starts working; there is no per-tile success
  // signal, so age it out instead.
  if (!tile_error_.empty()) {
    if (QDateTime::currentMSecsSinceEpoch() - tile_error_time_ < TILE_ERROR_TIMEOUT_MS) {
      PrintError(tile_error_);
      return;
    }
    tile_error_.clear();
  }

  if (IsDirty()) {
    PrintWarning("Unsaved changes.  Click Save to apply them.");
    return;
  }

  if (!transform_status_.empty() && !transform_ok_) {
    PrintError(transform_status_);
    return;
  }

  if (!CurrentSource()) {
    PrintWarning("Enter a tile URL and click Save to create this source.");
    return;
  }

  PrintInfo("OK");
}

void TileMapPlugin::HandleTileFailure(QString url, QString error_string)
{
  tile_error_ = "Tile request failed: " + error_string.toStdString() +
    " (" + url.toStdString() + ")";
  tile_error_time_ = QDateTime::currentMSecsSinceEpoch();
  UpdateStatus();
}

void TileMapPlugin::TestTileSource()
{
  std::shared_ptr<TileSource> source = CurrentSource();
  if (!source) {
    return;
  }

  // Probe the tile the view actually wants; falling back to the top of the
  // pyramid only when nothing has been drawn yet.
  int32_t level = 0;
  int64_t x = 0;
  int64_t y = 0;
  if (!tile_map_.GetCenterTile(level, x, y)) {
    level = 1;
  }

  QString url = source->GenerateTileUrl(level, x, y);
  ui_.test_button->setEnabled(false);
  PrintWarning("Testing " + url.toStdString() + " ...");
  test_network_manager_.get(QNetworkRequest(QUrl(url)));
}

void TileMapPlugin::HandleTestReply(QNetworkReply * reply)
{
  const QString url = reply->url().toString();
  const QVariant status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute);
  const qint64 size = reply->bytesAvailable();

  if (reply->error() == QNetworkReply::NoError) {
    std::string detail = status.isValid() ?
      "HTTP " + std::to_string(status.toInt()) :
      std::string("ok");
    // A server that answers 200 with an error page is a common failure for
    // hand-rolled tile servers, so report the size rather than just success.
    tile_error_.clear();
    PrintInfo(
      "Test succeeded: " + detail + ", " +
      std::to_string(size) + " bytes from " + url.toStdString());
  } else {
    std::string detail = status.isValid() ?
      "HTTP " + std::to_string(status.toInt()) + ": " :
      std::string();
    PrintError(
      "Test failed: " + detail + reply->errorString().toStdString() +
      " (" + url.toStdString() + ")");
  }

  reply->deleteLater();
  ui_.test_button->setEnabled(true);
}

void TileMapPlugin::SaveCustomSource()
{
  // If the user is editing a custom source, we want to fill in the default
  // name for it with its current name.
  // Otherwise, they're creating a new custom source, in which case we
  // should leave the default blank.
  QString current_source = ui_.source_combo->currentText();
  QString default_name = "";

  auto iter = tile_sources_.find(current_source);
  if (iter != tile_sources_.end()) {
    if (iter->second->IsCustom()) {
      default_name = current_source;
    } else if (iter->second->GetType() == BingSource::BING_TYPE) {
      // If the user has picked Bing as their source, we're not actually
      // saving a custom map source, just updating the API key
      BingSource * bing_source = dynamic_cast<BingSource *>(iter->second.get());
      bing_source->SetApiKey(ui_.base_url_text->text());
      dirty_ = false;
      UpdateControlState();
      UpdateStatus();
      return;
    } else if (iter->second->GetType() == CartoSource::CARTO_TYPE) {
      // If the user has picked Carto as their source, we're not actually
      // saving a custom map source, just updating the API key
      CartoSource * carto_source = dynamic_cast<CartoSource *>(iter->second.get());
      carto_source->SetApiKey(ui_.base_url_text->text());
      dirty_ = false;
      UpdateControlState();
      UpdateStatus();
      return;
    } else if (iter->second->GetType() == StadiaSource::STADIA_TYPE) {
      // If the user has picked a Stadia source (Stamen), we're not actually
      // saving a custom map source, just updating the API key for all Stadia sources
      QString api_key = ui_.base_url_text->text();
      for (auto & source_pair : tile_sources_) {
        if (source_pair.second->GetType() == StadiaSource::STADIA_TYPE) {
          StadiaSource * stadia_source = dynamic_cast<StadiaSource *>(source_pair.second.get());
          stadia_source->SetApiKey(api_key);
        }
      }
      dirty_ = false;
      UpdateControlState();
      UpdateStatus();
      return;
    }
  }

  QString problem = WmtsSource::ValidateBaseUrl(ui_.base_url_text->text());
  if (!problem.isEmpty()) {
    QMessageBox mbox;
    mbox.setText(problem);
    mbox.setIcon(QMessageBox::Warning);
    mbox.setStandardButtons(QMessageBox::Save | QMessageBox::Cancel);
    mbox.setDefaultButton(QMessageBox::Cancel);
    if (mbox.exec() != QMessageBox::Save) {
      return;
    }
  }

  bool ok;
  QString name = QInputDialog::getText(
    config_widget_,
    tr("Save New Tile Source"),
    tr("Tile Source Name:"),
    QLineEdit::Normal,
    default_name,
    &ok);
  name = name.trimmed();
  if (ok && !name.isEmpty()) {
    std::shared_ptr<WmtsSource> source = std::make_shared<WmtsSource>(
      name,
      ui_.base_url_text->text(),
      true,
      ui_.max_zoom_spin_box->value());
    int existing_index = ui_.source_combo->findText(name);
    if (existing_index != -1) {
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

void TileMapPlugin::PrintError(const std::string & message)
{
  if (message == ui_.status->text().toStdString()) {
    return;
  }

  RCLCPP_ERROR(Logger(), "Error: %s", message.c_str());
  QPalette p(ui_.status->palette());
  p.setColor(QPalette::Text, Qt::red);
  ui_.status->setPalette(p);
  ui_.status->setText(message.c_str());
}

void TileMapPlugin::PrintInfo(const std::string & message)
{
  if (message == ui_.status->text().toStdString()) {
    return;
  }

  RCLCPP_INFO(Logger(), "%s", message.c_str());
  QPalette p(ui_.status->palette());
  p.setColor(QPalette::Text, Qt::green);
  ui_.status->setPalette(p);
  ui_.status->setText(message.c_str());
}

void TileMapPlugin::PrintWarning(const std::string & message)
{
  if (message == ui_.status->text().toStdString()) {
    return;
  }

  RCLCPP_WARN(Logger(), "%s", message.c_str());
  QPalette p(ui_.status->palette());
  p.setColor(QPalette::Text, Qt::darkYellow);
  ui_.status->setPalette(p);
  ui_.status->setText(message.c_str());
}

QWidget * TileMapPlugin::GetConfigWidget(QWidget * parent)
{
  config_widget_->setParent(parent);

  return config_widget_;
}

bool TileMapPlugin::Initialize(QOpenGLWidget * canvas)
{
  canvas_ = canvas;

  SelectSource(CARTO_NAME);

  return true;
}

void TileMapPlugin::Draw(double x, double y, double scale)
{
  if (!tile_map_.IsReady()) {
    return;
  }

  swri_transform_util::Transform to_wgs84;
  if (tf_manager_->GetTransform(source_frame_, target_frame_, to_wgs84)) {
    tf2::Vector3 center(x, y, 0);
    center = to_wgs84 * center;

    if (center.y() != last_center_y_ ||
      center.x() != last_center_x_ ||
      scale != last_scale_ ||
      canvas_->width() != last_width_ ||
      canvas_->height() != last_height_)
    {
      // Draw() is called very frequently, and SetView is a fairly expensive operation, so we
      // can save some CPU time by only calling it when the relevant parameters have changed.
      last_center_y_ = center.y();
      last_center_x_ = center.x();
      last_scale_ = scale;
      last_width_ = canvas_->width();
      last_height_ = canvas_->height();
      tile_map_.SetView(center.y(), center.x(), scale, canvas_->width(), canvas_->height());
      RCLCPP_DEBUG(Logger(), "TileMapPlugin::Draw: Successfully set view");

      int32_t level;
      int64_t tile_x;
      int64_t tile_y;
      if (tile_map_.GetCenterTile(level, tile_x, tile_y)) {
        view_bounds_ = ViewAround(
          center.y(), center.x(), scale, last_width_, last_height_, level);
      }
    }
    tile_map_.Draw();
  }
}

QString TileMapPlugin::CurrentAttribution() const
{
  std::shared_ptr<TileSource> source = tile_map_.GetTileSource();
  if (!source) {
    return {};
  }
  return source->GetAttribution(view_bounds_);
}

void TileMapPlugin::Paint(QPainter * painter, double, double, double)
{
  std::shared_ptr<TileSource> source = tile_map_.GetTileSource();
  if (!source) {
    return;
  }
  const QString text = CurrentAttribution();
  const QImage logo = source->GetAttributionLogo();
  if (text.isEmpty() && logo.isNull()) {
    return;
  }

  // Drawn in screen coordinates, in the bottom-right corner where web maps
  // put their credits, so it stays put however the map is panned or rotated.
  painter->save();
  painter->resetTransform();

  QFont font = painter->font();
  font.setPixelSize(ATTRIBUTION_FONT_PIXELS);
  painter->setFont(font);
  const QFontMetrics metrics(font);

  const int canvas_width = painter->device()->width();
  const int canvas_height = painter->device()->height();

  // The logo is scaled to the height of one line of text.
  int logo_width = 0;
  int logo_height = 0;
  if (!logo.isNull()) {
    logo_height = metrics.height();
    logo_width = logo.width() * logo_height / std::max(1, logo.height());
  }
  const int gap = (logo_width > 0 && !text.isEmpty()) ? ATTRIBUTION_PADDING : 0;

  // Wrap rather than run off the left edge of a narrow canvas.
  const int text_flags = Qt::TextWordWrap | Qt::AlignRight | Qt::AlignVCenter;
  QRect text_bounds;
  if (!text.isEmpty()) {
    const int max_text_width = std::max(
      1, canvas_width - 2 * (ATTRIBUTION_MARGIN + ATTRIBUTION_PADDING) - logo_width - gap);
    text_bounds = metrics.boundingRect(
      QRect(0, 0, max_text_width, canvas_height), text_flags, text);
  }

  const int box_width = logo_width + gap + text_bounds.width() + 2 * ATTRIBUTION_PADDING;
  const int box_height =
    std::max(logo_height, text_bounds.height()) + 2 * ATTRIBUTION_PADDING;
  const QRect box(
    canvas_width - ATTRIBUTION_MARGIN - box_width,
    canvas_height - ATTRIBUTION_MARGIN - box_height,
    box_width,
    box_height);

  painter->setPen(Qt::NoPen);
  painter->setBrush(QColor(255, 255, 255, 200));
  painter->drawRect(box);

  if (logo_width > 0) {
    painter->drawImage(
      QRect(
        box.left() + ATTRIBUTION_PADDING,
        box.top() + (box.height() - logo_height) / 2,
        logo_width,
        logo_height),
      logo);
  }

  if (!text.isEmpty()) {
    painter->setPen(QColor(51, 51, 51));
    painter->drawText(
      QRect(
        box.left() + ATTRIBUTION_PADDING + logo_width + gap,
        box.top() + ATTRIBUTION_PADDING,
        text_bounds.width(),
        box.height() - 2 * ATTRIBUTION_PADDING),
      text_flags,
      text);
  }

  painter->restore();
}

void TileMapPlugin::Transform()
{
  swri_transform_util::Transform to_target;
  if (tf_manager_->GetTransform(target_frame_, source_frame_, to_target)) {
    tile_map_.SetTransform(to_target);
    transform_ok_ = true;
    transform_status_.clear();
  } else {
    transform_ok_ = false;
    transform_status_ = "No transform between " + source_frame_ + " and " + target_frame_;
  }

  // Transform() runs every frame.  Route through UpdateStatus() so that a
  // tile error or an unsaved edit is not overwritten a frame later.
  UpdateStatus();
}

void TileMapPlugin::LoadConfig(const YAML::Node & node, const std::string &)
{
  if (node[CUSTOM_SOURCES_KEY]) {
    const YAML::Node & sources = node[CUSTOM_SOURCES_KEY];
    YAML::Node::const_iterator source_iter;
    for (auto source_yaml : sources) {
      std::string type;
      if (source_yaml[TYPE_KEY]) {
        // If the type isn't set, we'll assume it's WMTS
        type = source_yaml[TYPE_KEY].as<std::string>();
      }
      std::shared_ptr<TileSource> source;
      if (type == "wmts" || type.empty()) {
        std::string name;
        std::string base_url;
        int max_zoom;
        name = source_yaml[NAME_KEY].as<std::string>();
        base_url = source_yaml[BASE_URL_KEY].as<std::string>();
        max_zoom = source_yaml[MAX_ZOOM_KEY].as<int>();
        source = std::make_shared<WmtsSource>(
          QString::fromStdString(name),
          QString::fromStdString(base_url),
          true,
          max_zoom);
      } else if (type == "bing") {
        std::string name;
        name = source_yaml[NAME_KEY].as<std::string>();
        source = std::make_shared<BingSource>(QString::fromStdString(name));
      }
      tile_sources_[source->GetName()] = source;
      ui_.source_combo->addItem(source->GetName());
    }
  }

  if (node[BING_API_KEY]) {
    std::string key = node[BING_API_KEY].as<std::string>();
    BingSource * source = dynamic_cast<BingSource *>(tile_sources_[BING_NAME].get());
    source->SetApiKey(QString::fromStdString(key));
  }

  if (node[CARTO_API_KEY]) {
    std::string key = node[CARTO_API_KEY].as<std::string>();
    CartoSource * source = dynamic_cast<CartoSource *>(tile_sources_[CARTO_NAME].get());
    source->SetApiKey(QString::fromStdString(key));
  }

  if (node[STADIA_API_KEY]) {
    std::string key = node[STADIA_API_KEY].as<std::string>();
    for (auto & source_pair : tile_sources_) {
      if (source_pair.second->GetType() == StadiaSource::STADIA_TYPE) {
        StadiaSource * stadia_source = dynamic_cast<StadiaSource *>(source_pair.second.get());
        stadia_source->SetApiKey(QString::fromStdString(key));
      }
    }
  }

  if (node[SOURCE_KEY]) {
    std::string source = node[SOURCE_KEY].as<std::string>();

    int index = ui_.source_combo->findText(QString::fromStdString(source), Qt::MatchExactly);

    if (index >= 0) {
      ui_.source_combo->setCurrentIndex(index);
    }

    SelectSource(QString::fromStdString(source));
  }
}

void TileMapPlugin::SaveConfig(YAML::Emitter & emitter, const std::string &)
{
  emitter << YAML::Key << CUSTOM_SOURCES_KEY << YAML::Value << YAML::BeginSeq;

  std::map<QString, std::shared_ptr<TileSource>>::iterator iter;
  for (iter = tile_sources_.begin(); iter != tile_sources_.end(); iter++) {
    if (iter->second->IsCustom()) {
      emitter << YAML::BeginMap;
      emitter << YAML::Key << BASE_URL_KEY << YAML::Value <<
        iter->second->GetBaseUrl().toStdString();
      emitter << YAML::Key << MAX_ZOOM_KEY << YAML::Value << iter->second->GetMaxZoom();
      emitter << YAML::Key << NAME_KEY << YAML::Value << iter->second->GetName().toStdString();
      emitter << YAML::Key << TYPE_KEY << YAML::Value << iter->second->GetType().toStdString();
      emitter << YAML::EndMap;
    }
  }
  emitter << YAML::EndSeq;

  BingSource * bing_source = dynamic_cast<BingSource *>(tile_sources_[BING_NAME].get());
  std::string bing_key = TrimString(bing_source->GetApiKey().toStdString());
  emitter << YAML::Key << BING_API_KEY << YAML::Value << bing_key;

  CartoSource * carto_source = dynamic_cast<CartoSource *>(tile_sources_[CARTO_NAME].get());
  std::string carto_key = TrimString(carto_source->GetApiKey().toStdString());
  emitter << YAML::Key << CARTO_API_KEY << YAML::Value << carto_key;

  // Save Stadia API key (all Stadia sources share the same key, so just get it from one)
  StadiaSource * stadia_source =
    dynamic_cast<StadiaSource *>(tile_sources_[STAMEN_TERRAIN_NAME].get());
  std::string stadia_key = TrimString(stadia_source->GetApiKey().toStdString());
  emitter << YAML::Key << STADIA_API_KEY << YAML::Value << stadia_key;

  std::string combo_str = TrimString(ui_.source_combo->currentText().toStdString());
  emitter << YAML::Key << SOURCE_KEY << YAML::Value << combo_str;
}

void TileMapPlugin::selectTileSource(const std::shared_ptr<TileSource> & tile_source)
{
  last_height_ = 0;   // This will force us to recalculate our view
  tile_map_.SetTileSource(tile_source);
  if (tile_source->GetType() == BingSource::BING_TYPE) {
    BingSource * bing_source = dynamic_cast<BingSource *>(tile_source.get());
    ui_.base_url_text->setText(bing_source->GetApiKey());
  } else if (tile_source->GetType() == CartoSource::CARTO_TYPE) {
    CartoSource * carto_source = dynamic_cast<CartoSource *>(tile_source.get());
    ui_.base_url_text->setText(carto_source->GetApiKey());
  } else if (tile_source->GetType() == StadiaSource::STADIA_TYPE) {
    StadiaSource * stadia_source = dynamic_cast<StadiaSource *>(tile_source.get());
    ui_.base_url_text->setText(stadia_source->GetApiKey());
  } else {
    ui_.base_url_text->setText(tile_source->GetBaseUrl());
  }
  ui_.max_zoom_spin_box->setValue(tile_source->GetMaxZoom());
}

void TileMapPlugin::SetNode(rclcpp::Node & node)
{
  MapvizPlugin::SetNode(node);
  tile_map_.SetLogger(node.get_logger());
}
}  // namespace tile_map
