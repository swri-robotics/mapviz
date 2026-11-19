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

#include <tile_map/contour_layer.hpp>

#include <QFont>
#include <QFontMetricsF>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QPointF>
#include <QRectF>
#include <QTransform>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>
#include <vector>

#include "swri_math_util/constants.h"
#include "swri_transform_util/earth_constants.h"
#include <tf2/LinearMath/Vector3.hpp>
#include <tile_map/terrain.hpp>
#include <tile_map/wmts_source.hpp>

namespace tile_map
{
namespace
{
/// Pixels on a side of an elevation tile.
constexpr int TILE_PIXELS = 256;
/// Priority of elevation requests: the same as the tiles on screen.
constexpr int32_t CONTOUR_TILE_PRIORITY = 10000;
/// Roughly how far apart labels on one line are, and how much straight line
/// a label needs on either side of its middle, in screen pixels.
constexpr double LABEL_SPACING_PIXELS = 350.0;
constexpr double LABEL_HALF_LENGTH_PIXELS = 22.0;
/// Labels closer than this to one already painted are left out.
constexpr double LABEL_MIN_GAP_PIXELS = 90.0;
/// Line widths, in pixels.
constexpr double LINE_WIDTH = 1.0;
constexpr double INDEX_LINE_WIDTH = 2.0;

/// Squares of the grid placed with one linear fit each.  Coarse levels cover
/// so much ground per cell that the transform bends within a large square.
int ChunkCells(int32_t level)
{
  return level >= 6 ? 64 : 16;
}

/// The same tiles, under the same URLs, as Map25d's terrain shading, so the
/// two share downloads in one image cache.
std::shared_ptr<WmtsSource> TerrainSource()
{
  static const std::shared_ptr<WmtsSource> source = std::make_shared<WmtsSource>(
    "Terrain Tiles", terrarium::URL_TEMPLATE, false, terrarium::MAX_ZOOM);
  return source;
}
}  // namespace

ContourLayer::ContourLayer() = default;

ContourLayer::~ContourLayer()
{
  // The build only uses its own copies, so it is safe to let it finish.
  if (build_.valid()) {
    build_.wait();
  }
}

void ContourLayer::SetImageCache(const ImageCachePtr & cache)
{
  if (cache != cache_) {
    cache_ = cache;
    heights_.clear();
    ++revision_;
  }
}

void ContourLayer::SetInterval(double interval)
{
  interval = std::max(0.0, interval);
  if (interval != interval_) {
    interval_ = interval;
    ++revision_;
  }
}

void ContourLayer::SetTransform(const swri_transform_util::Transform & wgs84_to_target)
{
#ifdef TILE_MAP_HAVE_TRANSFORM_EQUALITY
  if (wgs84_to_target == transform_) {
    return;
  }
#endif
  transform_ = wgs84_to_target;
  transform_changed_ = true;
}

ContourLayer::Block ContourLayer::BlockFor(
  double latitude, double longitude, double scale, int32_t width, int32_t height)
{
  Block block;
  if (!(scale > 0.0)) {
    return block;
  }
  latitude = std::clamp(latitude, -85.0511, 85.0511);
  longitude = std::clamp(longitude, -180.0, 180.0);
  // The deepest level whose pixels are at least as large as the screen's
  // (no deeper than the data set goes), or a coarser one if the view needs
  // too many of its tiles.
  const double circumference = swri_transform_util::_earth_equator_circumference *
    std::cos(latitude * swri_math_util::_deg_2_rad) / scale;
  const int32_t deepest = std::clamp(
    static_cast<int32_t>(std::floor(std::log2(circumference) - 8.0)), 0, terrarium::MAX_ZOOM);
  const double radius = 0.5 * scale * std::hypot(width, height);
  std::vector<std::pair<int64_t, int64_t>> tiles;
  for (block.level = deepest; block.level >= 0 && tiles.empty(); --block.level) {
    tiles = TilesAround(block.level, latitude, longitude, radius, MAX_TILES);
  }
  ++block.level;
  if (tiles.empty()) {
    return block;
  }
  block.x = tiles.front().first;
  block.y = tiles.front().second;
  block.columns = tiles.back().first - block.x + 1;
  block.rows = tiles.back().second - block.y + 1;
  return block;
}

void ContourLayer::GridToLatLon(
  const Block & block, double x, double y, double & latitude, double & longitude)
{
  // The middle of the grid cell is the middle of the tile pixel.
  const double tile_x = static_cast<double>(block.x) + (x + 0.5) / TILE_PIXELS;
  const double tile_y = static_cast<double>(block.y) + (y + 0.5) / TILE_PIXELS;
  longitude = tile_x / std::pow(2.0, block.level) * 360.0 - 180.0;
  latitude = TileRowLatitude(block.level, tile_y);
}

std::vector<float> ContourLayer::JoinTiles(
  const Block & block, const std::vector<std::shared_ptr<const std::vector<float>>> & tiles)
{
  const size_t width = static_cast<size_t>(block.columns) * TILE_PIXELS;
  const size_t height = static_cast<size_t>(block.rows) * TILE_PIXELS;
  std::vector<float> grid(width * height, std::numeric_limits<float>::quiet_NaN());
  constexpr size_t tile_cells = static_cast<size_t>(TILE_PIXELS) * TILE_PIXELS;
  for (int64_t row = 0; row < block.rows; ++row) {
    for (int64_t col = 0; col < block.columns; ++col) {
      const size_t i = static_cast<size_t>(row * block.columns + col);
      if (i >= tiles.size() || !tiles[i] || tiles[i]->size() != tile_cells) {
        continue;
      }
      const std::vector<float> & tile = *tiles[i];
      for (int r = 0; r < TILE_PIXELS; ++r) {
        std::copy_n(
          tile.begin() + static_cast<size_t>(r) * TILE_PIXELS, TILE_PIXELS,
          grid.begin() + (static_cast<size_t>(row) * TILE_PIXELS + r) * width +
          static_cast<size_t>(col) * TILE_PIXELS);
      }
    }
  }
  return grid;
}

void ContourLayer::SetView(
  double latitude, double longitude, double scale, int32_t width, int32_t height)
{
  const Block block = BlockFor(latitude, longitude, scale, width, height);
  if (block != block_ || scale != scale_) {
    block_ = block;
    scale_ = scale;
    ++revision_;
  }
  // Keep the heights read near the view; forget the rest.
  if (heights_.size() > 4 * MAX_TILES) {
    for (auto it = heights_.begin(); it != heights_.end(); ) {
      const auto [level, x, y] = it->first;
      const bool in_view = level == block_.level && x >= block_.x &&
        x < block_.x + block_.columns && y >= block_.y && y < block_.y + block_.rows;
      it = in_view ? std::next(it) : heights_.erase(it);
    }
  }
}

double ContourLayer::CurrentInterval() const
{
  return interval_ > 0.0 ? interval_ : AutoContourInterval(scale_);
}

bool ContourLayer::Update()
{
  if (!cache_ || block_.columns == 0) {
    return false;
  }
  const std::shared_ptr<WmtsSource> source = TerrainSource();
  bool changed = false;
  for (int64_t y = block_.y; y < block_.y + block_.rows; ++y) {
    for (int64_t x = block_.x; x < block_.x + block_.columns; ++x) {
      const TileKey key{block_.level, x, y};
      if (heights_.count(key)) {
        continue;
      }
      const ImagePtr image = cache_->GetImage(
        source->GenerateTileHash(block_.level, x, y),
        source->GenerateTileUrl(block_.level, x, y), CONTOUR_TILE_PRIORITY);
      if (!image) {
        // The cache gave up on this tile: no lines there.
        heights_[key] = nullptr;
        changed = true;
        continue;
      }
      const std::shared_ptr<QImage> pixels = image->GetImage();
      if (!pixels || pixels->isNull()) {
        continue;
      }
      heights_[key] = std::make_shared<const std::vector<float>>(terrarium::DecodeImage(*pixels));
      changed = true;
    }
  }
  if (changed) {
    ++revision_;
  }
  return changed;
}

void ContourLayer::StartBuild(double interval)
{
  std::vector<std::shared_ptr<const std::vector<float>>> tiles;
  tiles.reserve(static_cast<size_t>(block_.columns * block_.rows));
  for (int64_t y = block_.y; y < block_.y + block_.rows; ++y) {
    for (int64_t x = block_.x; x < block_.x + block_.columns; ++x) {
      const auto it = heights_.find(TileKey{block_.level, x, y});
      tiles.push_back(it == heights_.end() ? nullptr : it->second);
    }
  }

  // Grid cells per screen pixel, for spacing the labels.
  double latitude;
  double longitude;
  GridToLatLon(
    block_, 0.5 * block_.columns * TILE_PIXELS, 0.5 * block_.rows * TILE_PIXELS,
    latitude, longitude);
  const double cells_per_pixel = scale_ / MetersPerPixel(block_.level, latitude);

  build_ = std::async(
    std::launch::async,
    [block = block_, tiles = std::move(tiles), interval, cells_per_pixel]() {
      Built built;
      built.block = block;
      built.interval = interval;
      const int width = static_cast<int>(block.columns * TILE_PIXELS);
      const int height = static_cast<int>(block.rows * TILE_PIXELS);
      std::vector<ContourLine> lines;
      if (!TraceContours(JoinTiles(block, tiles), width, height, interval, MAX_SEGMENTS, lines)) {
        built.too_many = true;
        return built;
      }

      // Sort the segments into squares, ordinary lines and index contours
      // apart, relative to the middle of each square.
      const int size = ChunkCells(block.level);
      const int across = (width + size - 1) / size;
      const int down = (height + size - 1) / size;
      std::vector<std::vector<float>> ordinary(static_cast<size_t>(across) * down);
      std::vector<std::vector<float>> index(ordinary.size());
      for (const ContourLine & line : lines) {
        const size_t count = line.points.size();
        const size_t segments = line.closed ? count : count - 1;
        for (size_t i = 0; i < segments; ++i) {
          const GridPoint & a = line.points[i];
          const GridPoint & b = line.points[(i + 1) % count];
          const int cx = std::clamp(static_cast<int>(0.5 * (a.x + b.x) / size), 0, across - 1);
          const int cy = std::clamp(static_cast<int>(0.5 * (a.y + b.y) / size), 0, down - 1);
          const double ox = (cx + 0.5) * size;
          const double oy = (cy + 0.5) * size;
          std::vector<float> & out = (line.index ? index : ordinary)[cy * across + cx];
          out.insert(
            out.end(), {static_cast<float>(a.x - ox), static_cast<float>(a.y - oy),
              static_cast<float>(b.x - ox), static_cast<float>(b.y - oy)});
        }
      }
      for (size_t i = 0; i < ordinary.size(); ++i) {
        if (ordinary[i].empty() && index[i].empty()) {
          continue;
        }
        Chunk chunk;
        chunk.center = {
          (static_cast<double>(i % across) + 0.5) * size,
          (static_cast<double>(i / across) + 0.5) * size};
        chunk.index_start = ordinary[i].size() / 2;
        chunk.vertices = std::move(ordinary[i]);
        chunk.vertices.insert(chunk.vertices.end(), index[i].begin(), index[i].end());
        built.chunks.push_back(std::move(chunk));
      }

      const double spacing = LABEL_SPACING_PIXELS * cells_per_pixel;
      const double half_length = LABEL_HALF_LENGTH_PIXELS * cells_per_pixel;
      for (const LabelSpot & spot : LabelSpots(lines, spacing, half_length)) {
        Label label;
        label.height = spot.height;
        GridToLatLon(block, spot.position.x, spot.position.y, label.latitude, label.longitude);
        GridToLatLon(
          block, spot.before.x, spot.before.y, label.before_latitude, label.before_longitude);
        GridToLatLon(
          block, spot.after.x, spot.after.y, label.after_latitude, label.after_longitude);
        built.labels.push_back(label);
      }
      return built;
    });
}

void ContourLayer::Place(Built & built) const
{
  const double size = ChunkCells(built.block.level);
  auto to_target = [&](double x, double y) {
      double latitude;
      double longitude;
      GridToLatLon(built.block, x, y, latitude, longitude);
      return transform_ * tf2::Vector3(longitude, latitude, 0.0);
    };
  for (Chunk & chunk : built.chunks) {
    // A linear fit around the middle of the square, from central differences.
    const double h = 0.5 * size;
    const tf2::Vector3 middle = to_target(chunk.center.x, chunk.center.y);
    const tf2::Vector3 along_x =
      (to_target(chunk.center.x + h, chunk.center.y) -
      to_target(chunk.center.x - h, chunk.center.y)) / size;
    const tf2::Vector3 along_y =
      (to_target(chunk.center.x, chunk.center.y + h) -
      to_target(chunk.center.x, chunk.center.y - h)) / size;
    double * m = chunk.placement;
    std::fill(m, m + 16, 0.0);
    m[0] = along_x.x();
    m[1] = along_x.y();
    m[4] = along_y.x();
    m[5] = along_y.y();
    m[10] = 1.0;
    m[12] = middle.x();
    m[13] = middle.y();
    m[15] = 1.0;
  }
}

void ContourLayer::Draw(double pixel_ratio)
{
  if (!gl_initialized_) {
    initializeOpenGLFunctions();
    gl_initialized_ = true;
  }

  Update();

  if (build_.valid() && build_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    drawn_ = std::make_unique<Built>(build_.get());
    Place(*drawn_);
    transform_changed_ = false;
  }
  if (transform_changed_ && drawn_) {
    Place(*drawn_);
  }
  transform_changed_ = false;

  // While tiles arrive the revision changes every few frames; rebuilding
  // that often would keep a core busy, so wait a little between rebuilds.
  const bool due = !since_build_.isValid() ||
    since_build_.elapsed() >= static_cast<qint64>(1000.0 / REBUILDS_PER_SECOND);
  if (!build_.valid() && revision_ != started_revision_ && due && block_.columns > 0) {
    StartBuild(CurrentInterval());
    started_revision_ = revision_;
    since_build_.restart();
  }

  if (!drawn_ || drawn_->chunks.empty()) {
    return;
  }

  glPushAttrib(GL_ENABLE_BIT | GL_LINE_BIT | GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT);
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glMatrixMode(GL_MODELVIEW);
  glEnableClientState(GL_VERTEX_ARRAY);
  for (const bool index : {false, true}) {
    glLineWidth(static_cast<GLfloat>((index ? INDEX_LINE_WIDTH : LINE_WIDTH) * pixel_ratio));
    glColor4d(color_.redF(), color_.greenF(), color_.blueF(), index ? 0.95 : 0.75);
    for (const Chunk & chunk : drawn_->chunks) {
      const size_t total = chunk.vertices.size() / 2;
      const size_t first = index ? chunk.index_start : 0;
      const size_t count = index ? total - chunk.index_start : chunk.index_start;
      if (count == 0) {
        continue;
      }
      glPushMatrix();
      glMultMatrixd(chunk.placement);
      glVertexPointer(2, GL_FLOAT, 0, chunk.vertices.data() + 2 * first);
      glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(count));
      glPopMatrix();
    }
  }
  glDisableClientState(GL_VERTEX_ARRAY);
  glPopAttrib();
}

void ContourLayer::Paint(QPainter * painter)
{
  if (!labels_ || !drawn_ || drawn_->labels.empty() || !painter->device()) {
    return;
  }
  const QTransform world = painter->worldTransform();
  auto to_screen = [&](double latitude, double longitude) {
      const tf2::Vector3 point = transform_ * tf2::Vector3(longitude, latitude, 0.0);
      return world.map(QPointF(point.x(), point.y()));
    };

  painter->save();
  painter->resetTransform();
  painter->setRenderHint(QPainter::Antialiasing, true);
  QFont font = painter->font();
  if (font.pointSizeF() > 0) {
    font.setPointSizeF(font.pointSizeF() * 0.85);
  }
  const QFontMetricsF metrics(font);
  const QRectF canvas(0, 0, painter->device()->width(), painter->device()->height());
  const QColor text_color = color_.darker(160);
  const QPen halo(QColor(255, 255, 255, 210), 3.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

  std::vector<QPointF> placed;
  for (const Label & label : drawn_->labels) {
    const QPointF middle = to_screen(label.latitude, label.longitude);
    if (!canvas.contains(middle)) {
      continue;
    }
    const bool crowded = std::any_of(
      placed.begin(), placed.end(), [&middle](const QPointF & other) {
        return std::hypot(other.x() - middle.x(), other.y() - middle.y()) < LABEL_MIN_GAP_PIXELS;
      });
    if (crowded) {
      continue;
    }
    const QPointF before = to_screen(label.before_latitude, label.before_longitude);
    const QPointF after = to_screen(label.after_latitude, label.after_longitude);
    const QString text = QString::number(label.height, 'f', 0);
    const double text_width = metrics.horizontalAdvance(text);
    const double span = std::hypot(after.x() - before.x(), after.y() - before.y());
    if (span < text_width + 4.0) {
      // Zoomed out since the labels were placed; the line bends too soon.
      continue;
    }
    // Along the line, but never upside down.
    double angle = std::atan2(after.y() - before.y(), after.x() - before.x()) *
      swri_math_util::_rad_2_deg;
    if (angle > 90.0) {
      angle -= 180.0;
    } else if (angle < -90.0) {
      angle += 180.0;
    }

    QPainterPath path;
    path.addText(
      -0.5 * text_width, 0.5 * (metrics.ascent() - metrics.descent()), font, text);
    painter->translate(middle);
    painter->rotate(angle);
    painter->strokePath(path, halo);
    painter->fillPath(path, text_color);
    painter->resetTransform();
    placed.push_back(middle);
  }
  painter->restore();
}

QString ContourLayer::Status() const
{
  if (drawn_ && drawn_->too_many) {
    return "Too many contour lines to draw; zoom in or use a larger interval";
  }
  return {};
}
}  // namespace tile_map
