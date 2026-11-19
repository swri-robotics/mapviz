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

#ifndef TILE_MAP__TREE_LAYER_HPP_
#define TILE_MAP__TREE_LAYER_HPP_

#include <QImage>

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <tuple>
#include <vector>

#include <tile_map/image_cache.hpp>
#include <tile_map/scene_25d.hpp>
#include <tile_map/trees.hpp>
#include "swri_transform_util/transform.h"
#include <tile_map/tile_source.hpp>

namespace tile_map
{
class TileSource;  // IWYU pragma: keep

/**
 * Trees found in the imagery itself, for tile_map's 2.5D rendering.
 *
 * Canopy is picked out of the source's tiles at a fixed level (so the trees
 * don't change as the view zooms) and filled with round crowns.  The tiles
 * come through the same image cache tile_map draws from, so with most
 * sources nothing extra is downloaded.  Map-style sources have no canopy to
 * find, so they give no trees.
 */
class TreeLayer
{
public:
  /// Trees are not looked for when zoomed out past this many meters per
  /// pixel.
  static constexpr double MAX_SCALE = 1.5;
  /// The deepest level canopy is worked out at; about 1 m per pixel.
  static constexpr int32_t MAX_ANALYSIS_LEVEL = 17;
  /// The most imagery tiles looked at for one view.
  static constexpr size_t MAX_TILES = 64;
  /// Canopy is worked out for at most this many tiles per frame, so a new
  /// area fills in over a few frames instead of stalling one.
  static constexpr int TILES_PER_FRAME = 2;

  using TileKey = std::tuple<int32_t, int64_t, int64_t>;

  /// The imagery to look for trees in.  Returns true if it changed, in
  /// which case the view has to be set again.
  bool SetImagery(const std::shared_ptr<TileSource> & source, const ImageCachePtr & cache);

  /// The area being drawn.
  void SetView(double latitude, double longitude, double scale, int32_t width, int32_t height);

  /// The transform from WGS84 into the frame being drawn.
  void SetTransform(const swri_transform_util::Transform & wgs84_to_target);

  /// Looks at imagery tiles that have arrived since the last call.
  void Update();

  /// Adds the crowns in view.
  void Collect(std::vector<const ProjectedCrown *> * crowns) const;

  /// Changes whenever what Collect() returns does.
  uint64_t Revision() const {return revision_;}

  /// True when zoomed out too far to look for trees.
  bool TooFarOut() const {return too_far_out_;}

  /// The level canopy is worked out at for a source.
  static int32_t AnalysisLevel(const TileSource & source);

  /// The trees in one imagery tile.
  static std::vector<Crown> CrownsFromImage(
    const QImage & image, int32_t level, int64_t x, int64_t y);

  /// A crown in the frame being drawn, with a slight variation in color
  /// fixed by where the tree is.
  static ProjectedCrown ProjectCrown(
    const Crown & crown, const swri_transform_util::Transform & wgs84_to_target);

private:
  struct TileData
  {
    bool done = false;
    std::vector<Crown> crowns;
    std::vector<ProjectedCrown> projected;
  };

  void Project(TileData & tile);

  std::shared_ptr<TileSource> source_;
  ImageCachePtr cache_;
  std::map<TileKey, TileData> tiles_;
  std::vector<TileKey> view_tiles_;
  swri_transform_util::Transform transform_;
  uint64_t revision_ = 1;
  bool too_far_out_ = false;
};
}  // namespace tile_map

#endif  // TILE_MAP__TREE_LAYER_HPP_
