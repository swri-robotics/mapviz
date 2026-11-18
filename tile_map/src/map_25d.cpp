// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
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

#include <tile_map/map_25d.hpp>

#include <QStringList>

#include <memory>
#include <utility>
#include <vector>

#include <tile_map/wmts_source.hpp>

namespace tile_map
{
Map25d::Map25d(rclcpp::Logger logger)
: terrain_source_(
    std::make_shared<WmtsSource>(
      "Terrain Tiles", terrarium::URL_TEMPLATE, false, terrarium::MAX_ZOOM)),
  terrain_view_(logger, "/tmp/tile_map_terrain"),
  buildings_(std::make_unique<BuildingLayer>(logger))
{
  terrain_view_.SetTileSource(terrain_source_);
  // The shading is translucent; drawing the coarser level under it would
  // darken the map twice wherever both levels have loaded.
  terrain_view_.SetDrawPrecache(false);
  terrain_view_.SetImageFilter(
    [this](const QImage & image, const QString & url) {
      return ShadeTerrariumTile(image, url, shading_);
    });
}

void Map25d::SetLogger(rclcpp::Logger logger)
{
  terrain_view_.SetLogger(logger);
  buildings_->SetLogger(logger);
}

void Map25d::SetBuildingsEnabled(bool enabled)
{
  buildings_enabled_ = enabled;
}

void Map25d::SetImagery(const std::shared_ptr<TileSource> & source, const ImageCachePtr & cache)
{
  // A new source has different tiles to look in; the view usually hasn't
  // changed, so it won't be passed in again on its own.
  if (trees_.SetImagery(source, cache) && view_ &&
    trees_enabled_ && tree_source_ == TreeSource::Imagery)
  {
    trees_.SetView(view_->latitude, view_->longitude, view_->scale, view_->width, view_->height);
  }
}

void Map25d::SetDrawnTiles(std::vector<ImageryTile> tiles, uint64_t revision)
{
  drawn_tiles_ = std::move(tiles);
  drawn_tiles_revision_ = revision;
}

bool Map25d::MapDataNeeded() const
{
  return buildings_enabled_ || (trees_enabled_ && tree_source_ == TreeSource::Map);
}

void Map25d::SetShading(const ShadingParameters & shading)
{
  if (shading != shading_) {
    shading_ = shading;
    refresh_textures_ = true;
  }
}

void Map25d::SetTransform(const swri_transform_util::Transform & wgs84_to_target)
{
  terrain_view_.SetTransform(wgs84_to_target);
  buildings_->SetTransform(wgs84_to_target);
  trees_.SetTransform(wgs84_to_target);
}

void Map25d::SetView(
  double latitude, double longitude, double scale, int32_t width, int32_t height)
{
  view_ = View{latitude, longitude, scale, width, height};
  if (terrain_enabled_) {
    terrain_view_.SetView(latitude, longitude, scale, width, height);
  }
  // Only fetch data that is shown.
  if (MapDataNeeded()) {
    buildings_->SetView(latitude, longitude, scale, width, height);
  }
  if (trees_enabled_ && tree_source_ == TreeSource::Imagery) {
    trees_.SetView(latitude, longitude, scale, width, height);
  }
}

void Map25d::Draw()
{
  if (terrain_enabled_) {
    if (refresh_textures_) {
      terrain_view_.RefreshTextures();
      refresh_textures_ = false;
    }
    terrain_view_.Draw();
  }

  const bool map_trees = trees_enabled_ && tree_source_ == TreeSource::Map;
  const bool imagery_trees = trees_enabled_ && tree_source_ == TreeSource::Imagery;
  std::vector<const ProjectedBuilding *> buildings;
  std::vector<const ProjectedCrown *> crowns;
  if (MapDataNeeded()) {
    buildings_->Collect(buildings_enabled_ ? &buildings : nullptr, map_trees ? &crowns : nullptr);
  }
  if (imagery_trees) {
    trees_.Update();
    trees_.Collect(&crowns);
  }
  if (buildings.empty() && crowns.empty()) {
    return;
  }
  // Anything that changes what is drawn changes the revision, so the
  // renderer rebuilds its triangles only when it has to.  That includes the
  // imagery tiles: the triangles refer to their textures.
  static const std::vector<ImageryTile> no_tiles;
  const uint64_t revision =
    buildings_->Revision() * 1000003ULL + trees_.Revision() * 8191ULL +
    (imagery_textures_ ? drawn_tiles_revision_ * 131ULL + 8u : 0u) +
    (buildings_enabled_ ? 1u : 0u) + (map_trees ? 2u : 0u) + (imagery_trees ? 4u : 0u);
  renderer_.Draw(
    buildings, crowns, imagery_textures_ ? drawn_tiles_ : no_tiles, lean_, revision);
}

QString Map25d::Attribution() const
{
  QStringList credits;
  if (terrain_enabled_) {
    credits << terrarium::ATTRIBUTION;
  }
  // Trees found in the imagery come from the imagery, which tile_map
  // already credits.
  const bool map_trees = trees_enabled_ && tree_source_ == TreeSource::Map;
  if (buildings_enabled_ || map_trees) {
    const QString what = buildings_enabled_ && map_trees ? "Buildings and trees" :
      buildings_enabled_ ? "Buildings" : "Trees";
    credits << what + ": " + buildings_->Attribution();
  }
  return credits.join(" | ");
}

QString Map25d::Status(bool & is_error) const
{
  is_error = false;
  if (MapDataNeeded()) {
    const QString status = buildings_->Status(is_error);
    if (!status.isEmpty()) {
      return status;
    }
  }
  if (trees_enabled_ && tree_source_ == TreeSource::Imagery && trees_.TooFarOut()) {
    return "Zoom in closer to see trees";
  }
  return {};
}

ImageCachePtr Map25d::GetTerrainImageCache()
{
  return terrain_view_.GetImageCache();
}

QImage Map25d::ShadeTerrariumTile(
  const QImage & image, const QString & url, const ShadingParameters & shading)
{
  int32_t level;
  int64_t x;
  int64_t y;
  if (image.isNull() || !terrarium::ParseUrl(url, level, x, y)) {
    return QImage();
  }

  // Cell size at the middle of the tile; a 256 pixel tile spans little enough
  // latitude that the difference across it doesn't show in the shading.
  const double latitude = TileRowLatitude(level, static_cast<double>(y) + 0.5);
  const double cell_size = MetersPerPixel(level, latitude) * 256.0 / image.width();
  const std::vector<float> heights = terrarium::DecodeImage(image);
  return SlopeShade(heights, image.width(), image.height(), cell_size, shading);
}
}  // namespace tile_map
