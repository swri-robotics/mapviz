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

#include <algorithm>
#include <cmath>
#include <functional>

#include "swri_math_util/constants.h"

namespace tile_map
{
namespace
{
/// Segments in the polygon drawn for a round crown.
constexpr int DISC_SEGMENTS = 16;
/// The crown starts this far up the tree, as a fraction of its height.
constexpr double CROWN_BASE = 0.35;
/// A triangle spanning more tiles than this at one level is drawn plain.
constexpr int64_t MAX_TILES_PER_TRIANGLE = 16;

uint8_t ToByte(float value)
{
  return static_cast<uint8_t>(std::lround(std::clamp(value, 0.0f, 1.0f) * 255.0f));
}
}  // namespace

void Batch25d::Clear()
{
  vertices_.clear();
  colors_.clear();
  texcoords_.clear();
  runs_.clear();
}

void Batch25d::SetImagery(const std::vector<ImageryTile> & tiles)
{
  imagery_.clear();
  levels_.clear();
  for (const ImageryTile & tile : tiles) {
    if (tile.texture == 0) {
      continue;
    }
    imagery_[{tile.level, tile.x, tile.y}] = tile.texture;
    if (std::find(levels_.begin(), levels_.end(), tile.level) == levels_.end()) {
      levels_.push_back(tile.level);
    }
  }
  std::sort(levels_.begin(), levels_.end(), std::greater<int32_t>());
}

void Batch25d::Vertex(
  const tf2::Vector3 & point, const Color & color, uint32_t texture, float u, float v)
{
  if (runs_.empty() || runs_.back().texture != texture) {
    runs_.push_back({texture, VertexCount(), 0});
  }
  ++runs_.back().count;
  vertices_.push_back(point.x());
  vertices_.push_back(point.y());
  colors_.push_back(ToByte(color.r));
  colors_.push_back(ToByte(color.g));
  colors_.push_back(ToByte(color.b));
  colors_.push_back(255);
  texcoords_.push_back(u);
  texcoords_.push_back(v);
}

void Batch25d::Triangle(
  const tf2::Vector3 & a, const tf2::Vector3 & b, const tf2::Vector3 & c, const Color & color)
{
  Vertex(a, color);
  Vertex(b, color);
  Vertex(c, color);
}

void Batch25d::Quad(
  const tf2::Vector3 & a, const tf2::Vector3 & b, const tf2::Vector3 & c,
  const tf2::Vector3 & d, const Color & color)
{
  Triangle(a, b, c, color);
  Triangle(a, c, d, color);
}

void Batch25d::Disc(const tf2::Vector3 & center, double radius, const Color & color)
{
  const double step = static_cast<double>(swri_math_util::_2pi) / DISC_SEGMENTS;
  tf2::Vector3 previous = center + tf2::Vector3(radius, 0.0, 0.0);
  for (int i = 1; i <= DISC_SEGMENTS; ++i) {
    const tf2::Vector3 next =
      center + tf2::Vector3(radius * std::cos(i * step), radius * std::sin(i * step), 0.0);
    Triangle(center, previous, next, color);
    previous = next;
  }
}

bool Batch25d::TexturedTriangle(const MapPoint & a, const MapPoint & b, const MapPoint & c)
{
  if (levels_.empty()) {
    return false;
  }
  auto area = [](const std::vector<MapPoint> & polygon) {
      double sum = 0.0;
      for (size_t i = 0; i < polygon.size(); ++i) {
        const MapPoint & p = polygon[i];
        const MapPoint & q = polygon[(i + 1) % polygon.size()];
        sum += p.mx * q.my - q.mx * p.my;
      }
      return std::abs(sum) / 2.0;
    };
  const std::vector<MapPoint> triangle = {a, b, c};
  const double whole = area(triangle);
  if (whole <= 0.0) {
    return true;  // Nothing to draw either way.
  }

  // One step of Sutherland-Hodgman clipping against an axis-aligned line in
  // mercator coordinates, carrying the drawn position along.
  auto clip = [](
    const std::vector<MapPoint> & polygon, bool along_x, double limit, bool keep_below) {
      std::vector<MapPoint> out;
      auto inside = [&](const MapPoint & p) {
          const double value = along_x ? p.mx : p.my;
          return keep_below ? value <= limit : value >= limit;
        };
      for (size_t i = 0; i < polygon.size(); ++i) {
        const MapPoint & current = polygon[i];
        const MapPoint & previous = polygon[(i + polygon.size() - 1) % polygon.size()];
        const bool current_in = inside(current);
        const bool previous_in = inside(previous);
        if (current_in != previous_in) {
          const double from = along_x ? previous.mx : previous.my;
          const double to = along_x ? current.mx : current.my;
          const double t = (limit - from) / (to - from);
          out.push_back(
            {previous.position + (current.position - previous.position) * t,
              previous.mx + (current.mx - previous.mx) * t,
              previous.my + (current.my - previous.my) * t});
        }
        if (current_in) {
          out.push_back(current);
        }
      }
      return out;
    };

  const double min_x = std::min({a.mx, b.mx, c.mx});
  const double max_x = std::max({a.mx, b.mx, c.mx});
  const double min_y = std::min({a.my, b.my, c.my});
  const double max_y = std::max({a.my, b.my, c.my});

  for (int32_t level : levels_) {
    const double n = std::pow(2.0, level);
    const int64_t x0 = static_cast<int64_t>(std::floor(min_x * n));
    const int64_t x1 = static_cast<int64_t>(std::floor(max_x * n));
    const int64_t y0 = static_cast<int64_t>(std::floor(min_y * n));
    const int64_t y1 = static_cast<int64_t>(std::floor(max_y * n));
    if ((x1 - x0 + 1) * (y1 - y0 + 1) > MAX_TILES_PER_TRIANGLE) {
      continue;
    }

    struct Piece
    {
      uint32_t texture;
      int64_t x;
      int64_t y;
      std::vector<MapPoint> polygon;
    };
    std::vector<Piece> pieces;
    double covered = 0.0;
    for (int64_t y = y0; y <= y1; ++y) {
      for (int64_t x = x0; x <= x1; ++x) {
        std::vector<MapPoint> polygon = triangle;
        polygon = clip(polygon, true, x / n, false);
        polygon = clip(polygon, true, (x + 1) / n, true);
        polygon = clip(polygon, false, y / n, false);
        polygon = clip(polygon, false, (y + 1) / n, true);
        if (polygon.size() < 3) {
          continue;
        }
        const auto found = imagery_.find({level, x, y});
        if (found == imagery_.end()) {
          continue;
        }
        covered += area(polygon);
        pieces.push_back({found->second, x, y, std::move(polygon)});
      }
    }
    if (covered < whole * (1.0 - 1e-6)) {
      continue;
    }

    // Draped with white, so the imagery shows in its own colours.  Texture
    // rows run north to south in the image but bottom to top in GL.
    const Color white{1.0f, 1.0f, 1.0f};
    for (const Piece & piece : pieces) {
      auto texture_vertex = [&](const MapPoint & p) {
          const float u = static_cast<float>(p.mx * n - static_cast<double>(piece.x));
          const float v = static_cast<float>(1.0 - (p.my * n - static_cast<double>(piece.y)));
          Vertex(p.position, white, piece.texture, u, v);
        };
      for (size_t i = 1; i + 1 < piece.polygon.size(); ++i) {
        texture_vertex(piece.polygon[0]);
        texture_vertex(piece.polygon[i]);
        texture_vertex(piece.polygon[i + 1]);
      }
    }
    return true;
  }
  return false;
}

void Batch25d::AddBuilding(
  const ProjectedBuilding & building,
  const tf2::Vector3 & up, const tf2::Vector3 & right, double lean)
{
  const tf2::Vector3 top = up * (lean * building.height);
  const tf2::Vector3 bottom = up * (lean * building.min_height);
  const size_t count = building.outline.size();

  // Walls facing down the screen, toward the viewer.  Those facing up are
  // behind the building and covered by its roof.
  for (size_t i = 0; i < count && lean > 0.0; ++i) {
    if (i >= building.walls.size() || !building.walls[i]) {
      continue;
    }
    const tf2::Vector3 & a = building.outline[i];
    const tf2::Vector3 & b = building.outline[(i + 1) % count];
    tf2::Vector3 edge = b - a;
    edge.setZ(0.0);
    if (edge.length() < 1e-6) {
      continue;
    }
    edge.normalize();
    // Outward normal: to the right of the edge on a counterclockwise outline.
    const tf2::Vector3 normal = building.counterclockwise ?
      tf2::Vector3(edge.y(), -edge.x(), 0.0) : tf2::Vector3(-edge.y(), edge.x(), 0.0);
    if (normal.dot(up) > 0.0) {
      continue;
    }
    // Walls facing the viewer are lighter than walls seen side on.
    const float tone = static_cast<float>(0.62 - 0.16 * std::abs(normal.dot(right)));
    Quad(a + bottom, b + bottom, b + top, a + top, {tone, tone * 0.98f, tone * 0.95f});
  }

  // The roof shows the imagery that was under it, where there is some.
  const bool drape = !levels_.empty() && building.mercator.size() == 2 * count;
  const Color roof{0.84f, 0.83f, 0.80f};
  for (size_t i = 0; i + 2 < building.triangles.size(); i += 3) {
    const size_t corners[3] = {
      static_cast<size_t>(building.triangles[i]),
      static_cast<size_t>(building.triangles[i + 1]),
      static_cast<size_t>(building.triangles[i + 2])};
    if (drape) {
      auto point = [&](size_t index) {
          return MapPoint{building.outline[index] + top, building.mercator[2 * index],
            building.mercator[2 * index + 1]};
        };
      if (TexturedTriangle(point(corners[0]), point(corners[1]), point(corners[2]))) {
        continue;
      }
    }
    Triangle(
      building.outline[corners[0]] + top, building.outline[corners[1]] + top,
      building.outline[corners[2]] + top, roof);
  }
}

void Batch25d::AddCrown(
  const ProjectedCrown & crown,
  const tf2::Vector3 & up, const tf2::Vector3 & right, double lean)
{
  const float s = crown.shade;
  const tf2::Vector3 base = crown.center + up * (lean * crown.height * CROWN_BASE);
  const tf2::Vector3 top = crown.center + up * (lean * crown.height);
  const tf2::Vector3 side = right * crown.radius;

  if (lean > 0.0) {
    // The trunk, from the ground to the bottom of the crown.
    const tf2::Vector3 half_trunk = right * std::max(0.15, 0.08 * crown.radius);
    Quad(
      crown.center - half_trunk, crown.center + half_trunk, base + half_trunk,
      base - half_trunk, {0.36f * s, 0.27f * s, 0.18f * s});

    // The underside and the sides of the crown, between its bottom and top.
    const Color dark{0.16f * s, 0.29f * s, 0.12f * s};
    Disc(base, crown.radius, dark);
    Quad(base - side, base + side, top + side, top - side, dark);
  }

  // The top shows the canopy in the imagery, where there is some.
  if (!levels_.empty() && crown.mercator_per_meter > 0.0) {
    const tf2::Vector3 north(-crown.east.y(), crown.east.x(), 0.0);
    const double k = crown.mercator_per_meter;
    const double step = static_cast<double>(swri_math_util::_2pi) / DISC_SEGMENTS;
    auto rim = [&](int i) {
        const double dx = crown.radius * std::cos(i * step);
        const double dy = crown.radius * std::sin(i * step);
        // Mercator y grows southward.
        return MapPoint{top + crown.east * dx + north * dy,
          crown.mercator_x + dx * k, crown.mercator_y - dy * k};
      };
    const MapPoint middle{top, crown.mercator_x, crown.mercator_y};
    bool draped = true;
    const size_t before = VertexCount();
    for (int i = 0; i < DISC_SEGMENTS && draped; ++i) {
      draped = TexturedTriangle(middle, rim(i), rim(i + 1));
    }
    if (draped) {
      return;
    }
    // Part of the top had no imagery: draw the whole top plain instead.
    vertices_.resize(2 * before);
    colors_.resize(4 * before);
    texcoords_.resize(2 * before);
    while (!runs_.empty() && runs_.back().first >= before) {
      runs_.pop_back();
    }
    if (!runs_.empty()) {
      runs_.back().count = before - runs_.back().first;
    }
  }

  // A plain top, with a lighter middle so it reads as round.
  Disc(top, crown.radius, {0.25f * s, 0.43f * s, 0.18f * s});
  Disc(top, 0.55 * crown.radius, {0.33f * s, 0.53f * s, 0.24f * s});
}

double FrontEdge(const ProjectedBuilding & building, const tf2::Vector3 & up)
{
  double front = 1e300;
  for (const tf2::Vector3 & point : building.outline) {
    front = std::min(front, point.dot(up));
  }
  return front;
}

double FrontEdge(const ProjectedCrown & crown, const tf2::Vector3 & up)
{
  return crown.center.dot(up) - crown.radius;
}

void BuildBatch(
  const std::vector<const ProjectedBuilding *> & buildings,
  const std::vector<const ProjectedCrown *> & crowns,
  const tf2::Vector3 & up, const tf2::Vector3 & right, double lean,
  Batch25d & batch)
{
  // Ordered by front edge: an object whose nearest point is higher up the
  // screen is behind one whose nearest point is lower, so it goes first.
  struct Item
  {
    double front;
    const ProjectedBuilding * building;
    const ProjectedCrown * crown;
  };
  std::vector<Item> items;
  items.reserve(buildings.size() + crowns.size());
  for (const ProjectedBuilding * building : buildings) {
    items.push_back({FrontEdge(*building, up), building, nullptr});
  }
  for (const ProjectedCrown * crown : crowns) {
    items.push_back({FrontEdge(*crown, up), nullptr, crown});
  }
  std::stable_sort(
    items.begin(), items.end(),
    [](const Item & a, const Item & b) {return a.front > b.front;});

  batch.Clear();
  for (const Item & item : items) {
    if (item.building) {
      batch.AddBuilding(*item.building, up, right, lean);
    } else {
      batch.AddCrown(*item.crown, up, right, lean);
    }
  }
}

void Renderer25d::Draw(
  const std::vector<const ProjectedBuilding *> & buildings,
  const std::vector<const ProjectedCrown *> & crowns,
  const std::vector<ImageryTile> & imagery,
  double lean, uint64_t revision)
{
  if (!gl_initialized_) {
    initializeOpenGLFunctions();
    gl_initialized_ = true;
  }

  // "Up" and "right" on screen, in the frame being drawn, from the
  // modelview matrix (column major), so the lean follows the screen however
  // the map is rotated.
  GLdouble modelview[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
  tf2::Vector3 up(modelview[1], modelview[5], 0.0);
  tf2::Vector3 right(modelview[0], modelview[4], 0.0);
  if (up.length() < 1e-9 || right.length() < 1e-9) {
    return;
  }
  up.normalize();
  right.normalize();

  const bool rebuild = list_ == 0 || revision != built_revision_ || lean != built_lean_ ||
    up.distance(built_up_) > 1e-6;
  if (rebuild) {
    batch_.SetImagery(imagery);
    BuildBatch(buildings, crowns, up, right, lean, batch_);
    if (list_ == 0) {
      list_ = glGenLists(1);
    }
    // The client array state takes effect now; the draws, and the texture
    // binds between them, are recorded into the list.
    glNewList(list_, GL_COMPILE);
    if (batch_.VertexCount() > 0) {
      glEnableClientState(GL_VERTEX_ARRAY);
      glEnableClientState(GL_COLOR_ARRAY);
      glEnableClientState(GL_TEXTURE_COORD_ARRAY);
      glVertexPointer(2, GL_DOUBLE, 0, batch_.Vertices().data());
      glColorPointer(4, GL_UNSIGNED_BYTE, 0, batch_.Colors().data());
      glTexCoordPointer(2, GL_FLOAT, 0, batch_.TexCoords().data());
      for (const Batch25d::Run & run : batch_.Runs()) {
        if (run.texture != 0) {
          glEnable(GL_TEXTURE_2D);
          glBindTexture(GL_TEXTURE_2D, run.texture);
        } else {
          glDisable(GL_TEXTURE_2D);
        }
        glDrawArrays(
          GL_TRIANGLES, static_cast<GLint>(run.first), static_cast<GLsizei>(run.count));
      }
      glDisable(GL_TEXTURE_2D);
      glDisableClientState(GL_TEXTURE_COORD_ARRAY);
      glDisableClientState(GL_COLOR_ARRAY);
      glDisableClientState(GL_VERTEX_ARRAY);
    }
    glEndList();
    // The list holds its own copy of the triangles.
    batch_.Clear();
    built_revision_ = revision;
    built_lean_ = lean;
    built_up_ = up;
  }
  glCallList(list_);
}
}  // namespace tile_map
