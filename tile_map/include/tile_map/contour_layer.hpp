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

#ifndef TILE_MAP__CONTOUR_LAYER_HPP_
#define TILE_MAP__CONTOUR_LAYER_HPP_

#include <QColor>
#include <QElapsedTimer>
#include <QOpenGLFunctions_1_1>
#include <QString>

#include <cstddef>
#include <cstdint>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <tuple>
#include <vector>

#include <tile_map/contours.hpp>
#include <tile_map/image_cache.hpp>
#include "swri_transform_util/transform.h"

class QPainter;

namespace tile_map
{
/**
 * Contour lines for tile_map's topographic map mode.
 *
 * Heights come from the same elevation tiles (AWS Terrain Tiles) that 2.5D
 * rendering shades the terrain with, through the same image cache, so tiles
 * both need are downloaded once.  The tiles in view are joined into one grid
 * and traced on a worker thread, so lines run on across tile edges.  Lines
 * are drawn as lines, not into textures, so they stay thin at any zoom, and
 * the index contours' height labels are painted upright in screen space.
 *
 * Runs on the render thread, like Map25d.
 */
class ContourLayer : protected QOpenGLFunctions_1_1
{
public:
  /// The most elevation tiles joined into one grid.
  static constexpr size_t MAX_TILES = 100;
  /// The most line segments traced for one view; past this the layer asks
  /// for a larger interval instead of drawing a solid smear.
  static constexpr size_t MAX_SEGMENTS = 1500000;
  /// While tiles arrive, rebuilds start at most this often.
  static constexpr double REBUILDS_PER_SECOND = 4.0;
  /// Lines are drawn in squares of this many grid cells, each placed with
  /// its own linear fit to the transform into the frame being drawn.
  static constexpr int CHUNK_CELLS = 32;

  ContourLayer();
  /// Waits for a build in progress.
  ~ContourLayer();

  /// Where elevation tiles come from (see Map25d::GetTerrainImageCache()).
  void SetImageCache(const ImageCachePtr & cache);

  /// Height between lines in meters, or 0 to pick one from the zoom (see
  /// AutoContourInterval()).
  void SetInterval(double interval);
  double Interval() const {return interval_;}

  /// The color of the lines; labels use a darker shade.
  void SetColor(const QColor & color) {color_ = color;}
  const QColor & Color() const {return color_;}

  /// Whether index contours are labeled with their height.
  void SetLabels(bool labels) {labels_ = labels;}
  bool Labels() const {return labels_;}

  /// The transform from WGS84 into the frame being drawn.
  void SetTransform(const swri_transform_util::Transform & wgs84_to_target);

  /// The area being drawn: its center, meters per pixel and size in pixels.
  void SetView(double latitude, double longitude, double scale, int32_t width, int32_t height);

  /// Draws the lines.  Needs the canvas's GL context.
  /// @param pixel_ratio  Device pixels per logical pixel, for line widths
  void Draw(double pixel_ratio);

  /// Paints the height labels.  The painter's world transform maps the frame
  /// being drawn onto the canvas, as mapviz sets it up for Paint().
  void Paint(QPainter * painter);

  /// A problem to show in the status line; empty when there is none.
  QString Status() const;

  /// The interval of the lines being drawn, in meters (0 before any are).
  double DrawnInterval() const {return drawn_ ? drawn_->interval : 0.0;}

  /// Where in view the heights are: the tiles at `level` in a block of
  /// `columns` x `rows` starting at (`x`, `y`).
  struct Block
  {
    int32_t level = -1;
    int64_t x = 0;
    int64_t y = 0;
    int64_t columns = 0;
    int64_t rows = 0;

    bool operator==(const Block & other) const
    {
      return level == other.level && x == other.x && y == other.y &&
             columns == other.columns && rows == other.rows;
    }
    bool operator!=(const Block & other) const {return !(*this == other);}
  };

  /// The elevation tiles to draw a view with: every tile within half the
  /// canvas diagonal of the center, at the deepest level whose pixels are
  /// no smaller than the screen's (capped at what the data set has), or a
  /// coarser one if that would need more than MAX_TILES.
  static Block BlockFor(
    double latitude, double longitude, double scale, int32_t width, int32_t height);

  /// Latitude and longitude of a point on a block's grid.
  static void GridToLatLon(
    const Block & block, double x, double y, double & latitude, double & longitude);

  /// Joins a block's tiles into one grid of heights, NaN where a tile is
  /// missing.  `tiles` holds the block's tiles row by row, null for missing.
  static std::vector<float> JoinTiles(
    const Block & block, const std::vector<std::shared_ptr<const std::vector<float>>> & tiles);

private:
  /// Lines in one square of the grid, relative to its middle, in cells.
  struct Chunk
  {
    /// The middle of the square, in cells from the grid's corner.
    GridPoint center;
    /// x, y pairs, two points per segment: first the ordinary lines, then
    /// from `index_start` the index contours.
    std::vector<float> vertices;
    size_t index_start = 0;
    /// Column-major 4x4 matrix from the square's cells into the frame being
    /// drawn, refitted whenever the transform changes.
    double placement[16];
  };

  /// A height label: where it goes and which way the line runs there.
  struct Label
  {
    double latitude;
    double longitude;
    double before_latitude;
    double before_longitude;
    double after_latitude;
    double after_longitude;
    double height;
  };

  /// What a build on the worker thread hands back.
  struct Built
  {
    Block block;
    double interval = 0.0;
    bool too_many = false;
    std::vector<Chunk> chunks;
    std::vector<Label> labels;
  };

  /// Starts tracing what is in view on the worker thread.
  void StartBuild(double interval);

  /// Refits each chunk's placement to the current transform.
  void Place(Built & built) const;

  /// Reads elevation tiles in view that have arrived since the last frame.
  /// Returns true if any did.
  bool Update();

  /// The interval to draw the current view with.
  double CurrentInterval() const;

  using TileKey = std::tuple<int32_t, int64_t, int64_t>;

  ImageCachePtr cache_;
  double interval_ = 0.0;
  QColor color_ = QColor(160, 82, 45);
  bool labels_ = true;

  swri_transform_util::Transform transform_;
  bool transform_changed_ = false;

  Block block_;
  double scale_ = 0.0;
  /// Heights of each elevation tile read so far, null for tiles that failed.
  std::map<TileKey, std::shared_ptr<const std::vector<float>>> heights_;
  /// Changes whenever something a build depends on does.
  uint64_t revision_ = 1;

  std::future<Built> build_;
  std::optional<uint64_t> started_revision_;
  QElapsedTimer since_build_;
  /// The last build finished, which is drawn until the next one is ready.
  std::unique_ptr<Built> drawn_;

  bool gl_initialized_ = false;
};
}  // namespace tile_map

#endif  // TILE_MAP__CONTOUR_LAYER_HPP_
