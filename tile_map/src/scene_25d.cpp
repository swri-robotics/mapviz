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

#include <tile_map/scene_25d.hpp>

#include <QPainter>
#include <QtGlobal>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>

#include "swri_math_util/constants.h"

namespace tile_map
{
namespace
{
/// The most sides drawn on a round crown, and the fewest.
constexpr int MAX_DISC_SEGMENTS = 16;
constexpr int MIN_DISC_SEGMENTS = 6;
/// Roughly how long each side of a crown is on screen, in pixels.
constexpr double PIXELS_PER_SEGMENT = 4.0;
/// Crowns with a radius under this many pixels are drawn as just a plain top.
constexpr double DETAILED_CROWN_PIXELS = 3.0;
/// The crown starts this far up the tree, as a fraction of its height.
constexpr double CROWN_BASE = 0.35;
/// A triangle spanning more tiles than this at one level is drawn plain.
constexpr int64_t MAX_TILES_PER_TRIANGLE = 16;
/// The smallest a tile is shrunk to in the atlas.
constexpr int MIN_SLOT = 16;

uint8_t ToByte(float value)
{
  return static_cast<uint8_t>(std::lround(std::clamp(value, 0.0f, 1.0f) * 255.0f));
}

tf2::Vector3 AtHeight(const tf2::Vector3 & point, double height)
{
  return tf2::Vector3(point.x(), point.y(), height);
}
}  // namespace

// ---------------------------------------------------------------------------
// ImageryAtlas

void ImageryAtlas::Build(const std::vector<std::pair<ImageryTile, QImage>> & tiles)
{
  slots_.clear();
  levels_.clear();

  std::vector<const std::pair<ImageryTile, QImage> *> usable;
  for (const auto & tile : tiles) {
    if (!tile.second.isNull()) {
      usable.push_back(&tile);
    }
  }
  if (usable.empty()) {
    image_ = QImage();
    return;
  }
  // Finer tiles first, so they are the ones kept if not everything fits.
  std::stable_sort(
    usable.begin(), usable.end(),
    [](const auto * a, const auto * b) {return a->first.level > b->first.level;});

  // As large as fits everything, plus one slot kept white.
  int slot = MAX_SLOT;
  while (slot > MIN_SLOT &&
    static_cast<size_t>((SIZE / slot) * (SIZE / slot)) < usable.size() + 1)
  {
    slot /= 2;
  }
  const int per_side = SIZE / slot;
  const size_t capacity = static_cast<size_t>(per_side * per_side) - 1;

  image_ = QImage(SIZE, SIZE, QImage::Format_RGBA8888);
  image_.fill(Qt::white);
  QPainter painter(&image_);
  painter.setCompositionMode(QPainter::CompositionMode_Source);
  painter.setRenderHint(QPainter::SmoothPixmapTransform);

  // Slot 0, in the top left corner, stays white.
  white_u_ = static_cast<float>(0.5 * slot / SIZE);
  white_v_ = static_cast<float>(1.0 - 0.5 * slot / SIZE);

  for (size_t i = 0; i < usable.size() && i < capacity; ++i) {
    const ImageryTile & tile = usable[i]->first;
    const int index = static_cast<int>(i) + 1;
    const int px = (index % per_side) * slot;
    const int py = (index / per_side) * slot;
    painter.drawImage(QRect(px, py, slot, slot), usable[i]->second);
    // Half a pixel in from the edges, so filtering doesn't pull in the
    // neighbouring slot.  GL's v runs bottom to top.
    slots_[{tile.level, tile.x, tile.y}] = {
      static_cast<float>((px + 0.5) / SIZE),
      static_cast<float>(1.0 - (py + slot - 0.5) / SIZE),
      static_cast<float>((px + slot - 0.5) / SIZE),
      static_cast<float>(1.0 - (py + 0.5) / SIZE)};
    if (std::find(levels_.begin(), levels_.end(), tile.level) == levels_.end()) {
      levels_.push_back(tile.level);
    }
  }
  painter.end();
  std::sort(levels_.begin(), levels_.end(), std::greater<int32_t>());
}

bool ImageryAtlas::Find(int32_t level, int64_t x, int64_t y, std::array<float, 4> & rect) const
{
  const auto found = slots_.find({level, x, y});
  if (found == slots_.end()) {
    return false;
  }
  rect = found->second;
  return true;
}

// ---------------------------------------------------------------------------
// Batch25d

void Batch25d::Clear()
{
  vertices_.clear();
  colors_.clear();
  texcoords_.clear();
}

void Batch25d::Vertex(const tf2::Vector3 & point, const Color & color, float u, float v)
{
  vertices_.push_back(point.x());
  vertices_.push_back(point.y());
  vertices_.push_back(point.z());
  colors_.push_back(ToByte(color.r));
  colors_.push_back(ToByte(color.g));
  colors_.push_back(ToByte(color.b));
  colors_.push_back(255);
  texcoords_.push_back(u);
  texcoords_.push_back(v);
}

void Batch25d::Plain(const tf2::Vector3 & point, const Color & color)
{
  // The atlas's white texel leaves the vertex colour as it is.
  if (atlas_ && !atlas_->Empty()) {
    Vertex(point, color, atlas_->WhiteU(), atlas_->WhiteV());
  } else {
    Vertex(point, color, 0.0f, 0.0f);
  }
}

bool Batch25d::Draped(size_t vertex) const
{
  if (!atlas_ || atlas_->Empty() || vertex >= VertexCount()) {
    return false;
  }
  return texcoords_[2 * vertex] != atlas_->WhiteU() ||
         texcoords_[2 * vertex + 1] != atlas_->WhiteV();
}

void Batch25d::Triangle(
  const tf2::Vector3 & a, const tf2::Vector3 & b, const tf2::Vector3 & c, const Color & color)
{
  Plain(a, color);
  Plain(b, color);
  Plain(c, color);
}

void Batch25d::Quad(
  const tf2::Vector3 & a, const tf2::Vector3 & b, const tf2::Vector3 & c,
  const tf2::Vector3 & d, const Color & color)
{
  Triangle(a, b, c, color);
  Triangle(a, c, d, color);
}

void Batch25d::Disc(
  const tf2::Vector3 & center, double radius, int segments, const Color & color)
{
  const double step = static_cast<double>(swri_math_util::_2pi) / segments;
  tf2::Vector3 previous = center + tf2::Vector3(radius, 0.0, 0.0);
  for (int i = 1; i <= segments; ++i) {
    const tf2::Vector3 next =
      center + tf2::Vector3(radius * std::cos(i * step), radius * std::sin(i * step), 0.0);
    Triangle(center, previous, next, color);
    previous = next;
  }
}

bool Batch25d::TexturedTriangle(const MapPoint & a, const MapPoint & b, const MapPoint & c)
{
  if (!atlas_ || atlas_->Empty()) {
    return false;
  }
  auto area = [](const MapPoint * points, int count) {
      double sum = 0.0;
      for (int i = 0; i < count; ++i) {
        const MapPoint & p = points[i];
        const MapPoint & q = points[(i + 1) % count];
        sum += p.mx * q.my - q.mx * p.my;
      }
      return std::abs(sum) / 2.0;
    };
  const MapPoint corners[3] = {a, b, c};
  const double whole = area(corners, 3);
  if (whole <= 0.0) {
    return true;  // Nothing to draw either way.
  }

  // One step of Sutherland-Hodgman clipping against an axis-aligned line in
  // mercator coordinates, carrying the drawn position along.
  auto clip = [](const Polygon & polygon, bool along_x, double limit, bool keep_below) {
      Polygon out;
      auto inside = [&](const MapPoint & p) {
          const double value = along_x ? p.mx : p.my;
          return keep_below ? value <= limit : value >= limit;
        };
      for (int i = 0; i < polygon.count; ++i) {
        const MapPoint & current = polygon.points[i];
        const MapPoint & previous = polygon.points[(i + polygon.count - 1) % polygon.count];
        const bool current_in = inside(current);
        const bool previous_in = inside(previous);
        if (current_in != previous_in && out.count < 8) {
          const double from = along_x ? previous.mx : previous.my;
          const double to = along_x ? current.mx : current.my;
          const double t = (limit - from) / (to - from);
          out.points[out.count++] = {
            previous.position + (current.position - previous.position) * t,
            previous.mx + (current.mx - previous.mx) * t,
            previous.my + (current.my - previous.my) * t};
        }
        if (current_in && out.count < 8) {
          out.points[out.count++] = current;
        }
      }
      return out;
    };

  // A point's place in its tile, in the tile's slot of the atlas.  Draped
  // with white, so the imagery shows in its own colours.
  const Color white{1.0f, 1.0f, 1.0f};
  auto draped_vertex = [&](
    const MapPoint & p, const std::array<float, 4> & rect, double n, int64_t x, int64_t y) {
      const double u = p.mx * n - static_cast<double>(x);
      const double v = 1.0 - (p.my * n - static_cast<double>(y));
      Vertex(
        p.position, white, static_cast<float>(rect[0] + u * (rect[2] - rect[0])),
        static_cast<float>(rect[1] + v * (rect[3] - rect[1])));
    };

  const double min_x = std::min({a.mx, b.mx, c.mx});
  const double max_x = std::max({a.mx, b.mx, c.mx});
  const double min_y = std::min({a.my, b.my, c.my});
  const double max_y = std::max({a.my, b.my, c.my});

  for (int32_t level : atlas_->Levels()) {
    const double n = std::pow(2.0, level);
    const int64_t x0 = static_cast<int64_t>(std::floor(min_x * n));
    const int64_t x1 = static_cast<int64_t>(std::floor(max_x * n));
    const int64_t y0 = static_cast<int64_t>(std::floor(min_y * n));
    const int64_t y1 = static_cast<int64_t>(std::floor(max_y * n));
    std::array<float, 4> rect;

    // Most triangles sit inside one tile and need no cutting.
    if (x0 == x1 && y0 == y1) {
      if (!atlas_->Find(level, x0, y0, rect)) {
        continue;
      }
      for (const MapPoint & corner : corners) {
        draped_vertex(corner, rect, n, x0, y0);
      }
      return true;
    }
    if ((x1 - x0 + 1) * (y1 - y0 + 1) > MAX_TILES_PER_TRIANGLE) {
      continue;
    }

    struct Piece
    {
      std::array<float, 4> rect;
      int64_t x;
      int64_t y;
      Polygon polygon;
    };
    std::vector<Piece> pieces;
    double covered = 0.0;
    for (int64_t y = y0; y <= y1; ++y) {
      for (int64_t x = x0; x <= x1; ++x) {
        Polygon polygon;
        polygon.points[0] = a;
        polygon.points[1] = b;
        polygon.points[2] = c;
        polygon.count = 3;
        polygon = clip(polygon, true, x / n, false);
        polygon = clip(polygon, true, (x + 1) / n, true);
        polygon = clip(polygon, false, y / n, false);
        polygon = clip(polygon, false, (y + 1) / n, true);
        if (polygon.count < 3 || !atlas_->Find(level, x, y, rect)) {
          continue;
        }
        covered += area(polygon.points, polygon.count);
        pieces.push_back({rect, x, y, polygon});
      }
    }
    if (covered < whole * (1.0 - 1e-6)) {
      continue;
    }
    for (const Piece & piece : pieces) {
      for (int i = 1; i + 1 < piece.polygon.count; ++i) {
        draped_vertex(piece.polygon.points[0], piece.rect, n, piece.x, piece.y);
        draped_vertex(piece.polygon.points[i], piece.rect, n, piece.x, piece.y);
        draped_vertex(piece.polygon.points[i + 1], piece.rect, n, piece.x, piece.y);
      }
    }
    return true;
  }
  return false;
}

void Batch25d::AddBuilding(const ProjectedBuilding & building)
{
  const double top = building.height;
  const double bottom = building.min_height;
  const size_t count = building.outline.size();

  // Every wall: the depth test hides the ones at the back.  Their tone
  // depends on which way they face, the same for opposite walls, so corners
  // read without suggesting a light source.
  for (size_t i = 0; i < count; ++i) {
    if (i >= building.walls.size() || !building.walls[i]) {
      continue;
    }
    const tf2::Vector3 & a = building.outline[i];
    const tf2::Vector3 & b = building.outline[(i + 1) % count];
    const double angle = std::atan2(b.y() - a.y(), b.x() - a.x());
    const float tone = static_cast<float>(0.56 + 0.08 * std::cos(2.0 * angle));
    Quad(
      AtHeight(a, bottom), AtHeight(b, bottom), AtHeight(b, top), AtHeight(a, top),
      {tone, tone * 0.98f, tone * 0.95f});
  }

  // The roof shows the imagery that was under it, where there is some.
  const bool drape = atlas_ && !atlas_->Empty() && building.mercator.size() == 2 * count;
  const Color roof{0.84f, 0.83f, 0.80f};
  for (size_t i = 0; i + 2 < building.triangles.size(); i += 3) {
    const size_t corners[3] = {
      static_cast<size_t>(building.triangles[i]),
      static_cast<size_t>(building.triangles[i + 1]),
      static_cast<size_t>(building.triangles[i + 2])};
    if (drape) {
      auto point = [&](size_t index) {
          return MapPoint{AtHeight(building.outline[index], top), building.mercator[2 * index],
            building.mercator[2 * index + 1]};
        };
      if (TexturedTriangle(point(corners[0]), point(corners[1]), point(corners[2]))) {
        continue;
      }
    }
    Triangle(
      AtHeight(building.outline[corners[0]], top), AtHeight(building.outline[corners[1]], top),
      AtHeight(building.outline[corners[2]], top), roof);
  }
}

void Batch25d::AddCrown(const ProjectedCrown & crown)
{
  const float s = crown.shade;
  const double radius_pixels = crown.radius / std::max(scale_, 1e-9);
  const int segments = std::clamp(
    static_cast<int>(std::lround(
      static_cast<double>(swri_math_util::_2pi) * radius_pixels / PIXELS_PER_SEGMENT)),
    MIN_DISC_SEGMENTS, MAX_DISC_SEGMENTS);
  const bool detailed = radius_pixels >= DETAILED_CROWN_PIXELS;
  const double base = crown.height * CROWN_BASE;
  const double top = crown.height;
  const tf2::Vector3 center = AtHeight(crown.center, 0.0);

  if (detailed) {
    // The trunk, as two crossed strips so it shows however the map is
    // turned.
    const double half = std::max(0.15, 0.08 * crown.radius);
    const Color bark{0.36f * s, 0.27f * s, 0.18f * s};
    const tf2::Vector3 strips[2] = {tf2::Vector3(half, 0.0, 0.0), tf2::Vector3(0.0, half, 0.0)};
    for (const tf2::Vector3 & across : strips) {
      Quad(
        center - across, center + across, AtHeight(center + across, base),
        AtHeight(center - across, base), bark);
    }

    // The sides of the crown, as a short cylinder from its bottom to its top.
    const Color dark{0.16f * s, 0.29f * s, 0.12f * s};
    const double step = static_cast<double>(swri_math_util::_2pi) / segments;
    for (int i = 0; i < segments; ++i) {
      const tf2::Vector3 p(
        crown.center.x() + crown.radius * std::cos(i * step),
        crown.center.y() + crown.radius * std::sin(i * step), 0.0);
      const tf2::Vector3 q(
        crown.center.x() + crown.radius * std::cos((i + 1) * step),
        crown.center.y() + crown.radius * std::sin((i + 1) * step), 0.0);
      Quad(AtHeight(p, base), AtHeight(q, base), AtHeight(q, top), AtHeight(p, top), dark);
    }
  }

  // The top shows the canopy in the imagery, where there is some.
  const tf2::Vector3 top_center = AtHeight(crown.center, top);
  if (detailed && atlas_ && !atlas_->Empty() && crown.mercator_per_meter > 0.0) {
    const tf2::Vector3 north(-crown.east.y(), crown.east.x(), 0.0);
    const double k = crown.mercator_per_meter;
    const double step = static_cast<double>(swri_math_util::_2pi) / segments;
    auto rim = [&](int i) {
        const double dx = crown.radius * std::cos(i * step);
        const double dy = crown.radius * std::sin(i * step);
        // Mercator y grows southward.
        return MapPoint{top_center + crown.east * dx + north * dy,
          crown.mercator_x + dx * k, crown.mercator_y - dy * k};
      };
    const MapPoint middle{top_center, crown.mercator_x, crown.mercator_y};
    bool draped = true;
    const size_t before = VertexCount();
    for (int i = 0; i < segments && draped; ++i) {
      draped = TexturedTriangle(middle, rim(i), rim(i + 1));
    }
    if (draped) {
      return;
    }
    // Part of the top had no imagery: draw the whole top plain instead.
    vertices_.resize(3 * before);
    colors_.resize(4 * before);
    texcoords_.resize(2 * before);
  }

  // A plain top, with a lighter middle so it reads as round.  The middle is
  // at the same height and drawn after, which the depth test lets through.
  Disc(top_center, crown.radius, segments, {0.25f * s, 0.43f * s, 0.18f * s});
  if (detailed) {
    Disc(top_center, 0.55 * crown.radius, segments, {0.33f * s, 0.53f * s, 0.24f * s});
  }
}

void LeanMatrix(const tf2::Vector3 & up, double lean, double matrix[16])
{
  // Column major: x' = x + up.x * lean * z, y' = y + up.y * lean * z, and
  // z' = DEPTH_PER_METER * z, which mapviz's projection turns into a smaller
  // depth (nearer) the higher the point.
  const double values[16] = {
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    up.x() * lean, up.y() * lean, DEPTH_PER_METER, 0.0,
    0.0, 0.0, 0.0, 1.0};
  std::copy(values, values + 16, matrix);
}

// ---------------------------------------------------------------------------
// Renderer25d

void Renderer25d::InitializeGl()
{
  if (gl_initialized_) {
    return;
  }
  initializeOpenGLFunctions();
  gl_initialized_ = true;
  GLint depth_bits = 0;
  glGetIntegerv(GL_DEPTH_BITS, &depth_bits);
  if (depth_bits <= 0) {
    qWarning(
      "tile_map: the map canvas has no depth buffer, so 2.5D buildings and trees may be "
      "drawn in the wrong order where they overlap");
  }
}

void Renderer25d::Rebuild(
  const std::vector<const ProjectedBuilding *> & buildings,
  const std::vector<const ProjectedCrown *> & crowns,
  const ImageryAtlas & atlas, double scale)
{
  InitializeGl();

  batch_.Clear();
  batch_.SetAtlas(atlas.Empty() ? nullptr : &atlas);
  batch_.SetScale(scale);
  for (const ProjectedBuilding * building : buildings) {
    batch_.AddBuilding(*building);
  }
  for (const ProjectedCrown * crown : crowns) {
    batch_.AddCrown(*crown);
  }
  batch_.SetAtlas(nullptr);

  // The atlas, flipped so its top row is at v = 1 as GL expects.
  texture_.reset();
  if (!atlas.Empty()) {
#if QT_VERSION >= 0x060000
    const QImage gl_image = atlas.Image().flipped(Qt::Vertical);
#else
    const QImage gl_image = atlas.Image().mirrored(false, true);
#endif
    texture_ = std::make_unique<QOpenGLTexture>(QOpenGLTexture::Target2D);
    texture_->setFormat(QOpenGLTexture::RGBA8_UNorm);
    texture_->setSize(ImageryAtlas::SIZE, ImageryAtlas::SIZE);
    texture_->allocateStorage(QOpenGLTexture::RGBA, QOpenGLTexture::UInt8);
    texture_->setData(QOpenGLTexture::RGBA, QOpenGLTexture::UInt8, gl_image.constBits());
    texture_->setMinificationFilter(QOpenGLTexture::Linear);
    texture_->setMagnificationFilter(QOpenGLTexture::Linear);
    texture_->setWrapMode(QOpenGLTexture::ClampToEdge);
  }

  // Interleaved floats around the middle of the batch.
  const size_t count = batch_.VertexCount();
  const std::vector<double> & positions = batch_.Vertices();
  const std::vector<uint8_t> & colors = batch_.Colors();
  const std::vector<float> & texcoords = batch_.TexCoords();
  double low_x = 1e300;
  double high_x = -1e300;
  double low_y = 1e300;
  double high_y = -1e300;
  for (size_t i = 0; i < count; ++i) {
    low_x = std::min(low_x, positions[3 * i]);
    high_x = std::max(high_x, positions[3 * i]);
    low_y = std::min(low_y, positions[3 * i + 1]);
    high_y = std::max(high_y, positions[3 * i + 1]);
  }
  origin_x_ = count > 0 ? 0.5 * (low_x + high_x) : 0.0;
  origin_y_ = count > 0 ? 0.5 * (low_y + high_y) : 0.0;
  upload_.resize(count);
  for (size_t i = 0; i < count; ++i) {
    GpuVertex & vertex = upload_[i];
    vertex.x = static_cast<float>(positions[3 * i] - origin_x_);
    vertex.y = static_cast<float>(positions[3 * i + 1] - origin_y_);
    vertex.z = static_cast<float>(positions[3 * i + 2]);
    vertex.u = texcoords[2 * i];
    vertex.v = texcoords[2 * i + 1];
    std::copy(&colors[4 * i], &colors[4 * i] + 4, vertex.rgba);
  }
  if (!buffer_.isCreated()) {
    buffer_.create();
    buffer_.setUsagePattern(QOpenGLBuffer::StaticDraw);
  }
  buffer_.bind();
  buffer_.allocate(upload_.data(), static_cast<int>(count * sizeof(GpuVertex)));
  buffer_.release();
  vertex_count_ = static_cast<GLsizei>(count);
  upload_.clear();
  upload_.shrink_to_fit();
  batch_.Clear();
  built_ = true;
}

void Renderer25d::Draw(double lean)
{
  if (!built_ || vertex_count_ == 0) {
    return;
  }
  InitializeGl();

  // "Up" on screen in the frame being drawn, from the modelview matrix
  // (column major), so the lean follows the screen however it is turned.
  GLdouble modelview[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
  tf2::Vector3 up(modelview[1], modelview[5], 0.0);
  if (up.length() < 1e-9) {
    return;
  }
  up.normalize();

  GLdouble lean_matrix[16];
  LeanMatrix(up, lean, lean_matrix);
  GLint depth_function = GL_LESS;
  glGetIntegerv(GL_DEPTH_FUNC, &depth_function);
  const GLboolean depth_test = glIsEnabled(GL_DEPTH_TEST);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glMultMatrixd(lean_matrix);
  // The stored vertices are relative to the batch's middle; moving on the
  // ground doesn't change their height, so this can follow the lean.
  glTranslated(origin_x_, origin_y_, 0.0);
  glEnable(GL_DEPTH_TEST);
  // Equal depth passes, so a crown's lighter middle shows over its top.
  glDepthFunc(GL_LEQUAL);
  glDepthMask(GL_TRUE);

  buffer_.bind();
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_TEXTURE_COORD_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);
  const GLsizei stride = sizeof(GpuVertex);
  glVertexPointer(3, GL_FLOAT, stride, reinterpret_cast<const void *>(offsetof(GpuVertex, x)));
  glTexCoordPointer(2, GL_FLOAT, stride, reinterpret_cast<const void *>(offsetof(GpuVertex, u)));
  glColorPointer(
    4, GL_UNSIGNED_BYTE, stride, reinterpret_cast<const void *>(offsetof(GpuVertex, rgba)));
  if (texture_) {
    glEnable(GL_TEXTURE_2D);
    texture_->bind();
  }
  glDrawArrays(GL_TRIANGLES, 0, vertex_count_);
  if (texture_) {
    texture_->release();
    glDisable(GL_TEXTURE_2D);
  }
  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_TEXTURE_COORD_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
  buffer_.release();

  glDepthFunc(static_cast<GLenum>(depth_function));
  if (!depth_test) {
    glDisable(GL_DEPTH_TEST);
  }
  glPopMatrix();
}
}  // namespace tile_map
