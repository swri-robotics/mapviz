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
#include <QLabel>
#include <QSignalBlocker>
#include <QMessageBox>
#include <QUrl>
#include <QNetworkRequest>
#include <QDateTime>
#include <QFontInfo>
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
std::string TileMapPlugin::RENDER_25D_KEY = "render_25d";  // NOLINT(runtime/string)
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

// Size of the attribution text relative to the user interface font, and its
// distance from the canvas edges and from the edge of its background, in
// lines of that text.  Following the interface font keeps the credits legible
// for people who use larger text, at any display scale.
constexpr double ATTRIBUTION_FONT_SCALE = 0.9;
constexpr double ATTRIBUTION_MARGIN_LINES = 0.3;
constexpr double ATTRIBUTION_PADDING_LINES = 0.2;
// The spacing never drops below these, in pixels, however small the font.
constexpr int ATTRIBUTION_MIN_MARGIN = 4;
constexpr int ATTRIBUTION_MIN_PADDING = 3;
// The smallest the attribution text is shrunk to on a narrow canvas.
constexpr int ATTRIBUTION_MIN_FONT_PIXELS = 7;

// Keys inside the render_25d map of the config.
const char RENDER_25D_ENABLED_KEY[] = "enabled";
const char TERRAIN_SHADING_KEY[] = "terrain_shading";
const char SHADING_STRENGTH_KEY[] = "shading_strength";
const char EXAGGERATION_KEY[] = "exaggeration";
const char BUILDINGS_KEY[] = "buildings";
const char BUILDING_SOURCE_KEY[] = "building_source";
const char LEAN_KEY[] = "lean";
const char TREES_KEY[] = "trees";
const char TREE_SOURCE_KEY[] = "tree_source";
// Values of tree_source, in the order of the Tree data menu.
const char IMAGERY_TREES[] = "imagery";
const char MAP_TREES[] = "osm";
const char OVERPASS_URL_KEY[] = "overpass_url";
// Values of building_source, in the order of the Building data menu.
const char OVERPASS_SOURCE[] = "overpass";
const char STADIA_SOURCE[] = "stadia";

// The 2.5D controls go in rows 6 to 14 of the grid in tile_map_config.ui,
// between the Max Zoom/Test rows and Status, which is on row 15.
constexpr int RENDER_25D_FIRST_ROW = 6;

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
  , render_25d_(false)
  , last_center_x_(0.0)
  , last_center_y_(0.0)
  , last_scale_(0.0)
  , last_height_(0)
  , last_width_(0)
{
  ui_.setupUi(config_widget_);
  SetUp25dControls();

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
    // What the service had cached as of 2026-11; it answers HTTP 400 past
    // that, which leaves the map blank.  RequestServiceLimits() replaces this
    // with the level the service currently reports once the source is used.
    16);
  // USGS imagery is public domain; this is the credit the service asks for.
  tile_sources_[USGS_NAME]->SetAttribution("USDA, USGS The National Map: Orthoimagery");
  std::shared_ptr<BingSource> bing = std::make_shared<BingSource>(BING_NAME);
  tile_sources_[BING_NAME] = bing;

  mapviz::SetStatusLevel(ui_.status, mapviz::StatusLevel::Error);

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
  ui_.max_zoom_spin_box->setMaximum(TileSource::MAX_SUPPORTED_ZOOM);
  QObject::connect(
    &test_network_manager_, SIGNAL(finished(QNetworkReply*)),
    this, SLOT(HandleTestReply(QNetworkReply*)));
  // The image cache is created once by TileMapView and outlives every source
  // set on it, so this connection is made a single time.
  QObject::connect(
    tile_map_.GetImageCache().get(),
    SIGNAL(RequestFailed(QString,QString)),  // NOLINT(whitespace/comma)
    this, SLOT(HandleTileFailure(QString,QString)));  // NOLINT(whitespace/comma)
  QObject::connect(
    map_25d_.GetTerrainImageCache().get(),
    SIGNAL(RequestFailed(QString,QString)),  // NOLINT(whitespace/comma)
    this, SLOT(HandleTileFailure(QString,QString)));  // NOLINT(whitespace/comma)

  UpdateControlState();
}

void TileMapPlugin::SetUp25dControls()
{
  const ShadingParameters defaults;
  int row = RENDER_25D_FIRST_ROW;

  render_25d_check_ = new QCheckBox("2.5D rendering", config_widget_);
  render_25d_check_->setToolTip(
    "Draw depth cues over the imagery, such as shading that makes hills and slopes stand "
    "out.  The map stays top-down, so other displays line up with it as before.");
  ui_.gridLayout->addWidget(render_25d_check_, row++, 0, 1, 4);

  auto add_row = [this, &row](const QString & label, QWidget * control) {
      auto * label_widget = new QLabel(label, config_widget_);
      ui_.gridLayout->addWidget(label_widget, row, 0);
      ui_.gridLayout->addWidget(control, row, 1, 1, 3);
      settings_25d_widgets_.push_back(label_widget);
      settings_25d_widgets_.push_back(control);
      ++row;
    };

  terrain_check_ = new QCheckBox("Shade slopes", config_widget_);
  terrain_check_->setChecked(true);
  terrain_check_->setToolTip("Uses elevation from the free AWS Terrain Tiles data set.");
  add_row("Terrain:", terrain_check_);

  shading_strength_spin_ = new QSpinBox(config_widget_);
  shading_strength_spin_->setRange(0, 100);
  shading_strength_spin_->setSuffix(" %");
  shading_strength_spin_->setValue(static_cast<int>(std::lround(defaults.strength * 100.0)));
  shading_strength_spin_->setToolTip("How dark the steepest slopes get.");
  add_row("Shading:", shading_strength_spin_);

  exaggeration_spin_ = new QDoubleSpinBox(config_widget_);
  exaggeration_spin_->setRange(0.1, 20.0);
  exaggeration_spin_->setSingleStep(0.5);
  exaggeration_spin_->setDecimals(1);
  exaggeration_spin_->setSuffix(" x");
  exaggeration_spin_->setValue(defaults.exaggeration);
  exaggeration_spin_->setToolTip("Multiplies terrain heights, so gentle slopes still show.");
  add_row("Exaggeration:", exaggeration_spin_);

  buildings_check_ = new QCheckBox("Show buildings", config_widget_);
  buildings_check_->setChecked(true);
  buildings_check_->setToolTip(
    "Draws buildings standing up out of the map, when zoomed in to 3 m per pixel or closer.");
  add_row("Buildings:", buildings_check_);

  building_source_combo_ = new QComboBox(config_widget_);
  building_source_combo_->addItem("OpenStreetMap (Overpass)");
  building_source_combo_->addItem("Stadia vector tiles");
  building_source_combo_->setToolTip(
    "OpenStreetMap needs no key but can be slow to load a new area.  Stadia needs the Stadia "
    "API key entered on a Stamen source.  Either way, each area is kept on disk for 30 days.");
  add_row("Building data:", building_source_combo_);

  trees_check_ = new QCheckBox("Show trees", config_widget_);
  trees_check_->setChecked(true);
  trees_check_->setToolTip(
    "Draws trees standing up out of the map, when zoomed in to 1.5 m per pixel or closer.");
  add_row("Trees:", trees_check_);

  tree_source_combo_ = new QComboBox(config_widget_);
  tree_source_combo_->addItem("From the imagery");
  tree_source_combo_->addItem("OpenStreetMap");
  tree_source_combo_->setToolTip(
    "From the imagery finds tree canopy in satellite or aerial photos (map styles have "
    "none).  OpenStreetMap uses mapped woods, forests and single trees, from the same "
    "download as the buildings.");
  add_row("Tree data:", tree_source_combo_);

  lean_spin_ = new QSpinBox(config_widget_);
  lean_spin_->setRange(0, 200);
  lean_spin_->setSingleStep(10);
  lean_spin_->setSuffix(" %");
  lean_spin_->setValue(static_cast<int>(std::lround(map_25d_.Lean() * 100.0)));
  lean_spin_->setToolTip(
    "How far the top of a building or tree moves up the screen, as a percentage of its "
    "height.");
  add_row("Lean:", lean_spin_);

  for (QWidget * widget : settings_25d_widgets_) {
    widget->setVisible(false);
  }

  QObject::connect(
    render_25d_check_, &QCheckBox::toggled, this, &TileMapPlugin::Render25dToggled);
  QObject::connect(
    terrain_check_, &QCheckBox::toggled, this, &TileMapPlugin::Settings25dChanged);
  QObject::connect(
    shading_strength_spin_, qOverload<int>(&QSpinBox::valueChanged),
    this, &TileMapPlugin::Settings25dChanged);
  QObject::connect(
    exaggeration_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged),
    this, &TileMapPlugin::Settings25dChanged);
  QObject::connect(
    buildings_check_, &QCheckBox::toggled, this, &TileMapPlugin::Settings25dChanged);
  QObject::connect(
    building_source_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
    this, &TileMapPlugin::Settings25dChanged);
  QObject::connect(
    trees_check_, &QCheckBox::toggled, this, &TileMapPlugin::Settings25dChanged);
  QObject::connect(
    tree_source_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
    this, &TileMapPlugin::Settings25dChanged);
  QObject::connect(
    lean_spin_, qOverload<int>(&QSpinBox::valueChanged),
    this, &TileMapPlugin::Settings25dChanged);
  // Newly downloaded buildings should appear without waiting for the next
  // reason to redraw.
  QObject::connect(
    &map_25d_.Buildings(), &BuildingLayer::Updated, this, [this]() {
      if (canvas_) {
        canvas_->update();
      }
    });
}

void TileMapPlugin::Render25dToggled(bool checked)
{
  render_25d_ = checked;
  for (QWidget * widget : settings_25d_widgets_) {
    widget->setVisible(checked);
  }
  // The 2.5D layers only follow the view while they are on, so make the next
  // Draw() pass the current view to them.
  last_height_ = 0;
  if (canvas_) {
    canvas_->update();
  }
}

void TileMapPlugin::Settings25dChanged()
{
  ShadingParameters shading;
  shading.strength = shading_strength_spin_->value() / 100.0;
  shading.exaggeration = exaggeration_spin_->value();
  map_25d_.SetShading(shading);
  map_25d_.SetTerrainEnabled(terrain_check_->isChecked());
  map_25d_.SetBuildingsEnabled(buildings_check_->isChecked());
  map_25d_.Buildings().SetSource(
    building_source_combo_->currentIndex() == 1 ? BuildingSource::Stadia :
    BuildingSource::Overpass);
  map_25d_.SetTreesEnabled(trees_check_->isChecked());
  map_25d_.SetTreeSource(
    tree_source_combo_->currentIndex() == 1 ? TreeSource::Map : TreeSource::Imagery);
  map_25d_.SetLean(lean_spin_->value() / 100.0);
  // A layer that was just turned on needs to be told what is in view.
  last_height_ = 0;
  if (canvas_) {
    canvas_->update();
  }
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
    PrintWarning("Unsaved changes: click Save to apply them");
    return;
  }

  if (!transform_status_.empty() && !transform_ok_) {
    PrintError(transform_status_);
    return;
  }

  if (!CurrentSource()) {
    PrintWarning("Enter a tile URL and click Save to create this source");
    return;
  }

  if (render_25d_) {
    bool is_error = false;
    const QString status = map_25d_.Status(is_error);
    if (!status.isEmpty()) {
      if (is_error) {
        PrintError(status.toStdString());
      } else {
        PrintInfo(status.toStdString());
      }
      return;
    }
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
  const std::string text = mapviz::StatusText(message);
  if (text == ui_.status->text().toStdString()) {
    return;
  }

  RCLCPP_ERROR(Logger(), "Error: %s", message.c_str());
  mapviz::SetStatusLevel(ui_.status, mapviz::StatusLevel::Error);
  ui_.status->setText(QString::fromStdString(text));
}

void TileMapPlugin::PrintInfo(const std::string & message)
{
  const std::string text = mapviz::StatusText(message);
  if (text == ui_.status->text().toStdString()) {
    return;
  }

  RCLCPP_INFO(Logger(), "%s", message.c_str());
  mapviz::SetStatusLevel(ui_.status, mapviz::StatusLevel::Info);
  ui_.status->setText(QString::fromStdString(text));
}

void TileMapPlugin::PrintWarning(const std::string & message)
{
  const std::string text = mapviz::StatusText(message);
  if (text == ui_.status->text().toStdString()) {
    return;
  }

  RCLCPP_WARN(Logger(), "%s", message.c_str());
  mapviz::SetStatusLevel(ui_.status, mapviz::StatusLevel::Warning);
  ui_.status->setText(QString::fromStdString(text));
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
      if (render_25d_) {
        map_25d_.SetView(center.y(), center.x(), scale, canvas_->width(), canvas_->height());
      }
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
    if (render_25d_) {
      // All the Stadia sources share one key.
      auto stadia = std::dynamic_pointer_cast<StadiaSource>(tile_sources_[STAMEN_TERRAIN_NAME]);
      if (stadia) {
        map_25d_.Buildings().SetStadiaApiKey(stadia->GetApiKey());
      }
      map_25d_.SetImagery(tile_map_.GetTileSource(), tile_map_.GetImageCache());
      map_25d_.Draw();
    }
  }
}

int TileMapPlugin::CurrentMaxZoom() const
{
  std::shared_ptr<TileSource> source = tile_map_.GetTileSource();
  return source ? static_cast<int>(source->GetMaxZoom()) : -1;
}

QString TileMapPlugin::CurrentAttribution() const
{
  std::shared_ptr<TileSource> source = tile_map_.GetTileSource();
  if (!source) {
    return {};
  }
  QString attribution = source->GetAttribution(view_bounds_);
  const QString extra = render_25d_ ? map_25d_.Attribution() : QString();
  if (!extra.isEmpty()) {
    attribution = attribution.isEmpty() ? extra : attribution + " | " + extra;
  }
  return attribution;
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
  if (font.pointSizeF() > 0) {
    font.setPointSizeF(font.pointSizeF() * ATTRIBUTION_FONT_SCALE);
  } else {
    font.setPixelSize(std::max(1, qRound(font.pixelSize() * ATTRIBUTION_FONT_SCALE)));
  }
  // Work in pixels from here on, so the text can be shrunk a pixel at a time.
  font.setPixelSize(QFontInfo(font).pixelSize());

  const int canvas_width = painter->device()->width();
  const int canvas_height = painter->device()->height();
  const int text_flags = Qt::TextWordWrap | Qt::AlignRight | Qt::AlignVCenter;

  QFontMetrics metrics(font);
  int margin = 0;
  int padding = 0;
  int logo_width = 0;
  int logo_height = 0;
  int gap = 0;
  int max_text_width = 0;
  QRect text_bounds;
  while (true) {
    metrics = QFontMetrics(font);
    margin = std::max(
      ATTRIBUTION_MIN_MARGIN, qRound(metrics.height() * ATTRIBUTION_MARGIN_LINES));
    padding = std::max(
      ATTRIBUTION_MIN_PADDING, qRound(metrics.height() * ATTRIBUTION_PADDING_LINES));

    // The logo is scaled to the height of one line of text.
    if (!logo.isNull()) {
      logo_height = metrics.height();
      logo_width = logo.width() * logo_height / std::max(1, logo.height());
    }
    gap = (logo_width > 0 && !text.isEmpty()) ? padding : 0;

    // Wrap rather than run off the left edge of a narrow canvas.
    max_text_width = std::max(1, canvas_width - 2 * (margin + padding) - logo_width - gap);
    if (!text.isEmpty()) {
      text_bounds = metrics.boundingRect(
        QRect(0, 0, max_text_width, canvas_height), text_flags, text);
    }

    // A single word wider than the canvas can't wrap; make the text smaller
    // until it fits, down to a floor.
    if (text.isEmpty() || text_bounds.width() <= max_text_width ||
      font.pixelSize() <= ATTRIBUTION_MIN_FONT_PIXELS)
    {
      break;
    }
    font.setPixelSize(font.pixelSize() - 1);
  }
  painter->setFont(font);
  // On a canvas too narrow even for that, cut the text off rather than let it
  // run past the left edge.
  text_bounds.setWidth(std::min(text_bounds.width(), max_text_width));

  const int box_width = logo_width + gap + text_bounds.width() + 2 * padding;
  const int box_height =
    std::max(logo_height, text_bounds.height()) + 2 * padding;
  const QRect box(
    canvas_width - margin - box_width,
    canvas_height - margin - box_height,
    box_width,
    box_height);

  painter->setPen(Qt::NoPen);
  painter->setBrush(QColor(255, 255, 255, 200));
  painter->drawRect(box);
  painter->setClipRect(box);

  if (logo_width > 0) {
    painter->drawImage(
      QRect(
        box.left() + padding,
        box.top() + (box.height() - logo_height) / 2,
        logo_width,
        logo_height),
      logo);
  }

  if (!text.isEmpty()) {
    painter->setPen(QColor(51, 51, 51));
    painter->drawText(
      QRect(
        box.left() + padding + logo_width + gap,
        box.top() + padding,
        text_bounds.width(),
        box.height() - 2 * padding),
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
    map_25d_.SetTransform(to_target);
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

  if (node[RENDER_25D_KEY]) {
    const YAML::Node & render_25d = node[RENDER_25D_KEY];
    // Set every control before applying them, so the layers see one change.
    {
      const QSignalBlocker block_terrain(terrain_check_);
      const QSignalBlocker block_strength(shading_strength_spin_);
      const QSignalBlocker block_exaggeration(exaggeration_spin_);
      const QSignalBlocker block_buildings(buildings_check_);
      const QSignalBlocker block_building_source(building_source_combo_);
      const QSignalBlocker block_trees(trees_check_);
      const QSignalBlocker block_tree_source(tree_source_combo_);
      const QSignalBlocker block_lean(lean_spin_);
      if (render_25d[TERRAIN_SHADING_KEY]) {
        terrain_check_->setChecked(render_25d[TERRAIN_SHADING_KEY].as<bool>());
      }
      if (render_25d[SHADING_STRENGTH_KEY]) {
        shading_strength_spin_->setValue(
          static_cast<int>(std::lround(render_25d[SHADING_STRENGTH_KEY].as<double>() * 100.0)));
      }
      if (render_25d[EXAGGERATION_KEY]) {
        exaggeration_spin_->setValue(render_25d[EXAGGERATION_KEY].as<double>());
      }
      if (render_25d[BUILDINGS_KEY]) {
        buildings_check_->setChecked(render_25d[BUILDINGS_KEY].as<bool>());
      }
      if (render_25d[BUILDING_SOURCE_KEY]) {
        building_source_combo_->setCurrentIndex(
          render_25d[BUILDING_SOURCE_KEY].as<std::string>() == STADIA_SOURCE ? 1 : 0);
      }
      if (render_25d[TREES_KEY]) {
        trees_check_->setChecked(render_25d[TREES_KEY].as<bool>());
      }
      if (render_25d[TREE_SOURCE_KEY]) {
        tree_source_combo_->setCurrentIndex(
          render_25d[TREE_SOURCE_KEY].as<std::string>() == MAP_TREES ? 1 : 0);
      }
      if (render_25d[LEAN_KEY]) {
        lean_spin_->setValue(
          static_cast<int>(std::lround(render_25d[LEAN_KEY].as<double>() * 100.0)));
      }
      if (render_25d[OVERPASS_URL_KEY]) {
        map_25d_.Buildings().SetOverpassUrl(
          QString::fromStdString(render_25d[OVERPASS_URL_KEY].as<std::string>()));
      }
    }
    Settings25dChanged();
    if (render_25d[RENDER_25D_ENABLED_KEY]) {
      render_25d_check_->setChecked(render_25d[RENDER_25D_ENABLED_KEY].as<bool>());
    }
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

  emitter << YAML::Key << RENDER_25D_KEY << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << RENDER_25D_ENABLED_KEY << YAML::Value << render_25d_;
  emitter << YAML::Key << TERRAIN_SHADING_KEY << YAML::Value << terrain_check_->isChecked();
  emitter << YAML::Key << SHADING_STRENGTH_KEY << YAML::Value <<
    shading_strength_spin_->value() / 100.0;
  emitter << YAML::Key << EXAGGERATION_KEY << YAML::Value << exaggeration_spin_->value();
  emitter << YAML::Key << BUILDINGS_KEY << YAML::Value << buildings_check_->isChecked();
  emitter << YAML::Key << BUILDING_SOURCE_KEY << YAML::Value <<
    (building_source_combo_->currentIndex() == 1 ? STADIA_SOURCE : OVERPASS_SOURCE);
  emitter << YAML::Key << TREES_KEY << YAML::Value << trees_check_->isChecked();
  emitter << YAML::Key << TREE_SOURCE_KEY << YAML::Value <<
    (tree_source_combo_->currentIndex() == 1 ? MAP_TREES : IMAGERY_TREES);
  emitter << YAML::Key << LEAN_KEY << YAML::Value << lean_spin_->value() / 100.0;
  emitter << YAML::Key << OVERPASS_URL_KEY << YAML::Value <<
    map_25d_.Buildings().OverpassUrl().toStdString();
  emitter << YAML::EndMap;
}

void TileMapPlugin::selectTileSource(const std::shared_ptr<TileSource> & tile_source)
{
  last_height_ = 0;   // This will force us to recalculate our view
  tile_map_.SetTileSource(tile_source);
  QObject::connect(
    tile_source.get(), &TileSource::MaxZoomChanged,
    this, &TileMapPlugin::SourceMaxZoomChanged, Qt::UniqueConnection);
  tile_source->RequestServiceLimits();
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

void TileMapPlugin::SourceMaxZoomChanged()
{
  std::shared_ptr<TileSource> source = CurrentSource();
  if (!source || source.get() != sender()) {
    return;
  }

  last_height_ = 0;   // Recalculate the view, which picks the zoom level
  ui_.max_zoom_spin_box->setValue(source->GetMaxZoom());
}

void TileMapPlugin::SetNode(rclcpp::Node & node)
{
  MapvizPlugin::SetNode(node);
  tile_map_.SetLogger(node.get_logger());
  map_25d_.SetLogger(node.get_logger());
}
}  // namespace tile_map
