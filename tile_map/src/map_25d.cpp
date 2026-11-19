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

#include <cxxabi.h>

#include <QList>
#include <QStringList>
#include <QtGlobal>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <system_error>
#include <type_traits>
#include <utility>
#include <vector>

#include "swri_transform_util/transform.h"
#include <tile_map/building_layer.hpp>
#include <tile_map/scene_25d.hpp>
#include <tile_map/terrain.hpp>
#include <tile_map/tile_map_view.hpp>
#include <tile_map/tree_layer.hpp>
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

Map25d::~Map25d()
{
  // The build only uses its own copies, so it is safe to let it finish.
  if (build_.valid()) {
    build_.wait();
  }
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
  imagery_cache_ = cache;
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
  transform_ = wgs84_to_target;
  terrain_view_.SetTransform(wgs84_to_target);
  buildings_->SetTransform(wgs84_to_target);
  trees_.SetTransform(wgs84_to_target);
}

void Map25d::UpdateCullArea()
{
  if (!view_) {
    return;
  }
  const tf2::Vector3 center =
    transform_ * tf2::Vector3(view_->longitude, view_->latitude, 0.0);
  const double view_radius = 0.5 * view_->scale * std::hypot(view_->width, view_->height);
  auto planar = [](const tf2::Vector3 & a, const tf2::Vector3 & b) {
      return std::hypot(a.x() - b.x(), a.y() - b.y());
    };
  // Move the kept area only when the view nears its edge or zooms, so what
  // is drawn (and so the triangles) stays the same while panning a little.
  if (!cull_ || planar(center, cull_->center) > 0.25 * view_radius ||
    view_->scale > 1.25 * cull_->scale || view_->scale < 0.8 * cull_->scale)
  {
    // Tall things just outside can lean into view.
    const double margin = lean_ * 60.0;
    cull_ = CullArea{center, 1.5 * view_radius + margin, view_radius, view_->scale};
    ++cull_generation_;
  }
}

void Map25d::Cull(
  std::vector<const ProjectedBuilding *> & buildings,
  std::vector<const ProjectedCrown *> & crowns)
{
  if (!cull_) {
    return;
  }
  auto planar = [](const tf2::Vector3 & a, const tf2::Vector3 & b) {
      return std::hypot(a.x() - b.x(), a.y() - b.y());
    };
  const CullArea & area = *cull_;
  buildings.erase(
    std::remove_if(
      buildings.begin(), buildings.end(), [&](const ProjectedBuilding * building) {
        return planar(building->center, area.center) > area.radius + building->extent;
      }), buildings.end());
  crowns.erase(
    std::remove_if(
      crowns.begin(), crowns.end(), [&](const ProjectedCrown * crown) {
        return planar(crown->center, area.center) > area.radius + crown->radius;
      }), crowns.end());
  if (crowns.size() > MAX_CROWNS) {
    std::nth_element(
      crowns.begin(), crowns.begin() + MAX_CROWNS, crowns.end(),
      [&](const ProjectedCrown * a, const ProjectedCrown * b) {
        return planar(a->center, area.center) < planar(b->center, area.center);
      });
    crowns.resize(MAX_CROWNS);
  }
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
  if (imagery_trees) {
    trees_.Update();
  }
  UpdateCullArea();

  // Collecting and culling every building and tree costs more than drawing
  // them, so do it again only when something it depends on has changed.
  const uint64_t source_revision =
    buildings_->Revision() * 1000003ULL + trees_.Revision() * 8191ULL +
    cull_generation_ * 524287ULL +
    (buildings_enabled_ ? 1u : 0u) + (map_trees ? 2u : 0u) + (imagery_trees ? 4u : 0u);
  if (source_revision != collected_revision_) {
    collected_buildings_.clear();
    collected_crowns_.clear();
    if (MapDataNeeded()) {
      buildings_->Collect(
        buildings_enabled_ ? &collected_buildings_ : nullptr,
        map_trees ? &collected_crowns_ : nullptr);
    }
    if (imagery_trees) {
      trees_.Collect(&collected_crowns_);
    }
    Cull(collected_buildings_, collected_crowns_);
    collected_revision_ = source_revision;
  }
  // Upload a finished build.
  if (build_.valid() && build_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    BuildResult result = build_.get();
    if (result.atlas) {
      atlas_ = std::move(result.atlas);
      atlas_revision_ = result.atlas_revision;
    }
    renderer_.Upload(std::move(result.mesh));
  }

  if (collected_buildings_.empty() && collected_crowns_.empty()) {
    return;
  }

  // Anything that changes what is drawn changes the revision.  That includes
  // the imagery tiles, which the atlas is packed from.
  const uint64_t revision = source_revision +
    (imagery_textures_ ? drawn_tiles_revision_ * 131ULL + 8u : 0u);
  // While data arrives the revision changes every few frames; rebuilding
  // that often would keep a core busy, so wait a little between rebuilds.
  const bool due = !since_build_.isValid() ||
    since_build_.elapsed() >= static_cast<qint64>(1000.0 / REBUILDS_PER_SECOND);
  if (!build_.valid() && revision != started_revision_ && due) {
    StartBuild(collected_buildings_, collected_crowns_);
    started_revision_ = revision;
    since_build_.restart();
  }
  renderer_.Draw(lean_);
}

void Map25d::StartBuild(
  const std::vector<const ProjectedBuilding *> & buildings,
  const std::vector<const ProjectedCrown *> & crowns)
{
  // The layers own what the pointers point to and may change it while the
  // worker runs, so it gets copies.
  std::vector<ProjectedBuilding> building_copies;
  building_copies.reserve(buildings.size());
  for (const ProjectedBuilding * building : buildings) {
    building_copies.push_back(*building);
  }
  std::vector<ProjectedCrown> crown_copies;
  crown_copies.reserve(crowns.size());
  for (const ProjectedCrown * crown : crowns) {
    crown_copies.push_back(*crown);
  }

  // The atlas is packed again only when the drawn tiles have changed.  The
  // images are shared with the cache, not copied.
  std::shared_ptr<const ImageryAtlas> atlas = imagery_textures_ ? atlas_ : nullptr;
  const bool repack = imagery_textures_ && imagery_cache_ &&
    (!atlas_ || atlas_revision_ != drawn_tiles_revision_);
  std::vector<std::pair<ImageryTile, QImage>> images;
  if (repack) {
    images.reserve(drawn_tiles_.size());
    for (const ImageryTile & tile : drawn_tiles_) {
      const ImagePtr image = imagery_cache_->GetImage(tile.url_hash, tile.url);
      const std::shared_ptr<QImage> pixels = image ? image->GetImage() : nullptr;
      images.emplace_back(tile, pixels ? *pixels : QImage());
    }
  }

  const double scale = cull_ ? cull_->scale : 0.1;
  const uint64_t tiles_revision = drawn_tiles_revision_;
  build_ = std::async(
    std::launch::async,
    [buildings = std::move(building_copies), crowns = std::move(crown_copies),
    images = std::move(images), atlas, repack, scale, tiles_revision]() {
      BuildResult result;
      std::shared_ptr<const ImageryAtlas> drape = atlas;
      if (repack) {
        auto packed = std::make_shared<ImageryAtlas>();
        packed->Build(images);
        drape = packed;
        result.atlas = packed;
        result.atlas_revision = tiles_revision;
      }
      result.mesh = BuildMesh25d(buildings, crowns, drape, scale);
      return result;
    });
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
