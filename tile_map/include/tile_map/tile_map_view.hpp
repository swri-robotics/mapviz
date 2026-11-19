// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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

#ifndef TILE_MAP__TILE_MAP_VIEW_HPP_
#define TILE_MAP__TILE_MAP_VIEW_HPP_

#include <QOpenGLFunctions_1_1>
#include <QString>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <tile_map/tile_source.hpp>
#include <tile_map/texture_cache.hpp>
#include "swri_transform_util/transform.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tile_map/image_cache.hpp>

namespace tile_map
{
class TileSource;

struct Tile
{
public:
  QString url;
  size_t url_hash;
  int32_t level;
  int64_t x = 0;
  int64_t y = 0;
  int32_t subdiv_count;
  double subwidth;

  TexturePtr texture;

  std::vector<tf2::Vector3> points;
  std::vector<tf2::Vector3> points_t;
};

class TileMapViewTest;

class TileMapView : protected QOpenGLFunctions_1_1
{
  /// The tile geometry and the transform applied to it belong to the render
  /// thread; they are deliberately not reachable through the public API. The
  /// unit test fixture is a friend so that it can check what SetTransform()
  /// applied without opening them up to anything else.
  friend class TileMapViewTest;

public:
  /// @param cache_dir Where downloaded tiles are kept on disk.  Views drawing
  ///   different kinds of tiles should use different directories.
  explicit TileMapView(
    rclcpp::Logger logger = rclcpp::get_logger("tile_map::TileMapView"),
    const QString & cache_dir = "/tmp/tile_map");

  bool IsReady();

  void ResetCache();

  void SetLogger(rclcpp::Logger logger);

  void SetTileSource(const std::shared_ptr<TileSource> & tile_source);

  std::shared_ptr<TileSource> GetTileSource() const;

  void SetTransform(const swri_transform_util::Transform & transform);

  void SetView(
    double latitude,
    double longitude,
    double scale,
    int32_t width,
    int32_t height);

  void Draw();

  /// See TextureCache::SetImageFilter().
  void SetImageFilter(TextureCache::ImageFilter filter);

  /// Rebuilds every tile's texture from its downloaded image, e.g. after the
  /// image filter's settings change.
  void RefreshTextures();

  /// Whether to draw the next coarser level under the current one while its
  /// tiles load (on by default).  Turn it off for translucent overlays, which
  /// would otherwise be drawn twice wherever both levels have loaded.
  void SetDrawPrecache(bool draw_precache) {draw_precache_ = draw_precache;}

  /// The tiles being drawn that have their textures, finer level first.
  /// The textures stay valid until TextureRevision() changes.
  std::vector<ImageryTile> DrawnTiles() const;

  /// Changes whenever a drawn tile gets, loses or changes its texture.
  uint64_t TextureRevision() const {return texture_revision_;}

  /// The image cache is created once and outlives every tile source set on
  /// this view, so its signals can be connected a single time.
  ImageCachePtr GetImageCache();

  /// The tile the view is currently centred on, so a source can be probed
  /// without guessing coordinates.  False before the first SetView().
  bool GetCenterTile(int32_t & level, int64_t & x, int64_t & y) const;

private:
  void DrawTiles(std::vector<Tile> & tiles, int priority);

  std::shared_ptr<TileSource> tile_source_;

  swri_transform_util::Transform transform_;

  int32_t level_;

  int64_t center_x_;
  int64_t center_y_;

  int64_t size_;

  int32_t width_;
  int32_t height_;

  std::vector<Tile> tiles_;
  std::vector<Tile> precache_;

  TextureCachePtr tile_cache_;

  bool gl_initialized_ = false;

  bool draw_precache_ = true;

  uint64_t texture_revision_ = 1;

  rclcpp::Logger logger_;

  void ToLatLon(int32_t level, double x, double y, double & latitude, double & longitude);

  void InitializeTile(int32_t level, int64_t x, int64_t y, Tile & tile, int priority);
};
}  // namespace tile_map

#endif  // TILE_MAP__TILE_MAP_VIEW_HPP_
