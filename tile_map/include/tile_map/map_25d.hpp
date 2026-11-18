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

#ifndef TILE_MAP__MAP_25D_HPP_
#define TILE_MAP__MAP_25D_HPP_

#include <QImage>
#include <QString>

#include <memory>
#include <optional>
#include <vector>

#include <rclcpp/logger.hpp>
#include <tile_map/building_layer.hpp>
#include <tile_map/scene_25d.hpp>
#include <tile_map/terrain.hpp>
#include <tile_map/tree_layer.hpp>
#include <tile_map/tile_map_view.hpp>
#include "swri_transform_util/transform.h"

namespace tile_map
{
class WmtsSource;

/// Where trees come from.
enum class TreeSource
{
  /// Canopy picked out of the imagery being drawn.
  Imagery,
  /// Woods, forests and single trees mapped in OpenStreetMap, from the same
  /// download as the buildings.
  Map,
};

/**
 * The layers tile_map draws over its imagery when "2.5D rendering" is on.
 *
 * Everything stays in mapviz's top-down view and is lit evenly from every
 * direction, with no light source and no shadows.  Depth comes from shading
 * steep ground and from buildings and trees whose tops lean up the screen in
 * proportion to their height, so the imagery and every other plugin stay
 * where they are on the map.
 *
 * Runs on the render thread: the setters only record what changed, and
 * anything that touches OpenGL waits for Draw().
 */
class Map25d
{
public:
  explicit Map25d(rclcpp::Logger logger = rclcpp::get_logger("tile_map::Map25d"));

  void SetLogger(rclcpp::Logger logger);

  /// Relief shading from elevation tiles.
  void SetTerrainEnabled(bool enabled) {terrain_enabled_ = enabled;}
  bool TerrainEnabled() const {return terrain_enabled_;}

  void SetShading(const ShadingParameters & shading);
  const ShadingParameters & Shading() const {return shading_;}

  /// Buildings standing up out of the map.
  void SetBuildingsEnabled(bool enabled);
  bool BuildingsEnabled() const {return buildings_enabled_;}

  /// Trees standing up out of the map.
  void SetTreesEnabled(bool enabled) {trees_enabled_ = enabled;}
  bool TreesEnabled() const {return trees_enabled_;}
  void SetTreeSource(TreeSource source) {tree_source_ = source;}
  TreeSource GetTreeSource() const {return tree_source_;}

  /// The imagery tile_map is drawing, for finding trees in.
  void SetImagery(const std::shared_ptr<TileSource> & source, const ImageCachePtr & cache);

  /// The imagery tiles tile_map has drawn this frame and their revision (see
  /// TileMapView::DrawnTiles()), for draping over roofs and treetops.
  void SetDrawnTiles(std::vector<ImageryTile> tiles, uint64_t revision);

  /// Whether roofs and treetops show the imagery that was under them, rather
  /// than plain colours.
  void SetImageryTextures(bool enabled) {imagery_textures_ = enabled;}
  bool ImageryTextures() const {return imagery_textures_;}

  /// How far the top of a building or tree moves up the screen per metre of
  /// height (0 draws them flat).
  void SetLean(double lean) {lean_ = lean;}
  double Lean() const {return lean_;}

  /// Where building data comes from, and what that needs.
  BuildingLayer & Buildings() {return *buildings_;}
  const BuildingLayer & Buildings() const {return *buildings_;}

  /// The transform from WGS84 into the frame being drawn.
  void SetTransform(const swri_transform_util::Transform & wgs84_to_target);

  /// The area being drawn: its centre, metres per pixel and size in pixels.
  void SetView(double latitude, double longitude, double scale, int32_t width, int32_t height);

  /// Draws the enabled layers.  Needs the canvas's GL context.
  void Draw();

  /// Credit lines for the data the enabled layers draw.
  QString Attribution() const;

  /// Progress or a problem to show in the status line; empty when there is
  /// nothing to say.
  QString Status(bool & is_error) const;

  /// The cache that downloads elevation tiles, to connect to its errors.
  ImageCachePtr GetTerrainImageCache();

  /// Turns a downloaded Terrarium tile into its shading overlay.  Returns a
  /// null image if `url` isn't a Terrarium tile URL.
  static QImage ShadeTerrariumTile(
    const QImage & image, const QString & url, const ShadingParameters & shading);

private:
  std::shared_ptr<WmtsSource> terrain_source_;
  TileMapView terrain_view_;
  bool terrain_enabled_ = true;
  ShadingParameters shading_;
  /// Set when the shading changes; the textures are rebuilt in Draw(), where
  /// the GL context is current.
  bool refresh_textures_ = false;

  /// True when the building layer's data is needed: for buildings, or for
  /// the trees OpenStreetMap maps.
  bool MapDataNeeded() const;

  /// The last view passed to SetView(), for layers that need it again.
  struct View
  {
    double latitude;
    double longitude;
    double scale;
    int32_t width;
    int32_t height;
  };
  std::optional<View> view_;

  std::unique_ptr<BuildingLayer> buildings_;
  bool buildings_enabled_ = true;
  TreeLayer trees_;
  bool trees_enabled_ = true;
  TreeSource tree_source_ = TreeSource::Imagery;
  double lean_ = 0.6;
  bool imagery_textures_ = true;
  std::vector<ImageryTile> drawn_tiles_;
  uint64_t drawn_tiles_revision_ = 0;
  Renderer25d renderer_;
};
}  // namespace tile_map

#endif  // TILE_MAP__MAP_25D_HPP_
