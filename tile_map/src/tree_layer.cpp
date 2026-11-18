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

#include <tile_map/tree_layer.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

#include <tile_map/terrain.hpp>
#include <tile_map/tile_source.hpp>
#include "swri_math_util/constants.h"
#include "swri_transform_util/earth_constants.h"

namespace tile_map
{
namespace
{
/// Priority of the imagery requests made for trees: below the tiles on
/// screen, above the coarser level tile_map loads in advance.
constexpr int32_t TREE_TILE_PRIORITY = 5000;
}  // namespace

int32_t TreeLayer::AnalysisLevel(const TileSource & source)
{
  return std::min(source.GetMaxZoom(), MAX_ANALYSIS_LEVEL);
}

bool TreeLayer::SetImagery(const std::shared_ptr<TileSource> & source, const ImageCachePtr & cache)
{
  if (source == source_ && cache == cache_) {
    return false;
  }
  source_ = source;
  cache_ = cache;
  tiles_.clear();
  view_tiles_.clear();
  ++revision_;
  return true;
}

void TreeLayer::SetView(
  double latitude, double longitude, double scale, int32_t width, int32_t height)
{
  std::vector<TileKey> view_tiles;
  too_far_out_ = scale <= 0.0 || scale > MAX_SCALE;
  if (source_ && !too_far_out_) {
    const int32_t level = AnalysisLevel(*source_);
    const double radius = 0.5 * scale * std::hypot(width, height);
    for (const auto & tile : TilesAround(level, latitude, longitude, radius, MAX_TILES)) {
      view_tiles.emplace_back(level, tile.first, tile.second);
    }
  }
  if (view_tiles == view_tiles_) {
    return;
  }
  view_tiles_ = std::move(view_tiles);
  ++revision_;

  // Keep what has been worked out near the view; forget the rest.
  if (tiles_.size() > 2 * MAX_TILES) {
    for (auto it = tiles_.begin(); it != tiles_.end(); ) {
      if (std::find(view_tiles_.begin(), view_tiles_.end(), it->first) == view_tiles_.end()) {
        it = tiles_.erase(it);
      } else {
        ++it;
      }
    }
  }
}

void TreeLayer::SetTransform(const swri_transform_util::Transform & wgs84_to_target)
{
#ifdef TILE_MAP_HAVE_TRANSFORM_EQUALITY
  if (wgs84_to_target == transform_) {
    return;
  }
#endif
  transform_ = wgs84_to_target;
  for (auto & entry : tiles_) {
    if (entry.second.done) {
      Project(entry.second);
    }
  }
}

void TreeLayer::Update()
{
  if (!source_ || !cache_ || !source_->IsReady()) {
    return;
  }
  int worked = 0;
  for (const TileKey & key : view_tiles_) {
    TileData & tile = tiles_[key];
    if (tile.done) {
      continue;
    }
    const auto [level, x, y] = key;
    const ImagePtr image = cache_->GetImage(
      source_->GenerateTileHash(level, x, y), source_->GenerateTileUrl(level, x, y),
      TREE_TILE_PRIORITY);
    if (!image) {
      // The cache gave up on this tile: no trees there.
      tile.done = true;
      continue;
    }
    const std::shared_ptr<QImage> pixels = image->GetImage();
    if (!pixels || pixels->isNull()) {
      continue;
    }
    if (worked >= TILES_PER_FRAME) {
      // Leave the rest for the next frames.
      break;
    }
    tile.crowns = CrownsFromImage(*pixels, level, x, y);
    tile.done = true;
    Project(tile);
    ++worked;
  }
}

std::vector<Crown> TreeLayer::CrownsFromImage(
  const QImage & image, int32_t level, int64_t x, int64_t y)
{
  std::vector<Crown> crowns;
  if (image.isNull()) {
    return crowns;
  }
  const int width = image.width();
  const int height = image.height();
  const double latitude = TileRowLatitude(level, static_cast<double>(y) + 0.5);
  const double cell_size = MetersPerPixel(level, latitude) * 256.0 / width;
  const uint64_t seed = (static_cast<uint64_t>(x) << 32) ^ static_cast<uint64_t>(y) ^
    (static_cast<uint64_t>(level) << 58);
  const std::vector<PixelCrown> placed =
    PlaceCrowns(CanopyMask(image), width, height, cell_size, seed);

  const double n = std::pow(2.0, level);
  for (const PixelCrown & crown : placed) {
    const double fx = static_cast<double>(x) + crown.x / width;
    const double fy = static_cast<double>(y) + crown.y / height;
    crowns.push_back(
      {{TileRowLatitude(level, fy), fx / n * 360.0 - 180.0}, crown.radius, crown.height});
  }
  return crowns;
}

ProjectedCrown TreeLayer::ProjectCrown(
  const Crown & crown, const swri_transform_util::Transform & wgs84_to_target)
{
  ProjectedCrown projected;
  projected.center =
    wgs84_to_target * tf2::Vector3(crown.center.longitude, crown.center.latitude, 0.0);
  projected.center.setZ(0.0);
  projected.radius = crown.radius;
  projected.height = crown.height;

  // For draping the imagery over the top: where the crown is on the
  // mercator map, the map's scale there, and which way east is when drawn.
  LatLonToMercator(
    crown.center.latitude, crown.center.longitude, projected.mercator_x, projected.mercator_y);
  const double cos_lat = std::cos(
    crown.center.latitude * static_cast<double>(swri_math_util::_deg_2_rad));
  projected.mercator_per_meter =
    1.0 / (swri_transform_util::_earth_equator_circumference * std::max(cos_lat, 1e-6));
  tf2::Vector3 east =
    wgs84_to_target * tf2::Vector3(crown.center.longitude + 1e-5, crown.center.latitude, 0.0) -
    wgs84_to_target * tf2::Vector3(crown.center.longitude, crown.center.latitude, 0.0);
  east.setZ(0.0);
  if (east.length() > 0.0) {
    projected.east = east.normalized();
  }

  const double hash = std::fmod(
    std::abs(crown.center.latitude * 7919.0 + crown.center.longitude * 104729.0), 1.0);
  projected.shade = static_cast<float>(0.88 + 0.24 * hash);
  return projected;
}

void TreeLayer::Project(TileData & tile)
{
  ++revision_;
  tile.projected.clear();
  tile.projected.reserve(tile.crowns.size());
  for (const Crown & crown : tile.crowns) {
    tile.projected.push_back(ProjectCrown(crown, transform_));
  }
}

void TreeLayer::Collect(std::vector<const ProjectedCrown *> * crowns) const
{
  for (const TileKey & key : view_tiles_) {
    auto found = tiles_.find(key);
    if (found == tiles_.end()) {
      continue;
    }
    for (const ProjectedCrown & crown : found->second.projected) {
      crowns->push_back(&crown);
    }
  }
}
}  // namespace tile_map
