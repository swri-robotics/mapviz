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

#ifndef TILE_MAP__LOCAL_TERRAIN_VIEW_HPP_
#define TILE_MAP__LOCAL_TERRAIN_VIEW_HPP_

#include <QOpenGLFunctions_1_1>
#include <QOpenGLTexture>

#include <cstdint>
#include <map>
#include <memory>
#include <tuple>
#include <vector>

#include <tile_map/contour_layer.hpp>
#include <tile_map/elevation_set.hpp>
#include <tile_map/terrain.hpp>
#include "swri_transform_util/transform.h"
#include <tf2/LinearMath/Vector3.hpp>

namespace tile_map
{
/**
 * Slope shading drawn from an elevation set on disk, for 2.5D rendering
 * without a network connection: the local counterpart of the TileMapView
 * that shades the online elevation tiles.
 *
 * Each tile of the set in view is shaded (see SlopeShade()) into a texture
 * and drawn as a grid of quads whose corners are projected into the frame
 * being drawn, so sets in UTM or in longitude and latitude both line up
 * with the imagery.  Runs on the render thread.
 */
class LocalTerrainView : protected QOpenGLFunctions_1_1
{
public:
  /// Tiles shaded per frame, so a new area fills in over a few frames
  /// instead of stalling one.
  static constexpr int TILES_PER_FRAME = 4;
  /// Quads along each side of a tile.
  static constexpr int SUBDIVISIONS = 4;

  LocalTerrainView() = default;
  ~LocalTerrainView() = default;
  LocalTerrainView(const LocalTerrainView &) = delete;
  LocalTerrainView & operator=(const LocalTerrainView &) = delete;

  /// The set to draw, or null for none.
  void SetSet(const std::shared_ptr<const ElevationSet> & set);
  const std::shared_ptr<const ElevationSet> & Set() const {return set_;}

  void SetShading(const ShadingParameters & shading);

  void SetTransform(const swri_transform_util::Transform & wgs84_to_target);

  void SetView(double latitude, double longitude, double scale, int32_t width, int32_t height);

  /// Draws the shading.  Needs the canvas's GL context.
  void Draw();

private:
  using TileKey = std::tuple<int32_t, int64_t, int64_t>;

  struct Tile
  {
    /// Null for a tile with no data.
    std::unique_ptr<QOpenGLTexture> texture;
    /// (SUBDIVISIONS + 1)^2 corners in the frame being drawn, row by row.
    std::vector<tf2::Vector3> corners;
  };

  /// Projects a tile's corners with the current transform.
  void Project(const TileKey & key, Tile & tile) const;

  std::shared_ptr<const ElevationSet> set_;
  ShadingParameters shading_;
  swri_transform_util::Transform transform_;
  ContourLayer::Block block_;
  std::map<TileKey, Tile> tiles_;
  /// Set when the textures have to be made again, in Draw().
  bool refresh_ = false;
  bool gl_initialized_ = false;
};
}  // namespace tile_map

#endif  // TILE_MAP__LOCAL_TERRAIN_VIEW_HPP_
