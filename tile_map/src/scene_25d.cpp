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

#include "swri_math_util/constants.h"

namespace tile_map
{
namespace
{
/// Segments in the polygon drawn for a round crown.
constexpr int DISC_SEGMENTS = 16;
/// The crown starts this far up the tree, as a fraction of its height.
constexpr double CROWN_BASE = 0.35;

uint8_t ToByte(float value)
{
  return static_cast<uint8_t>(std::lround(std::clamp(value, 0.0f, 1.0f) * 255.0f));
}
}  // namespace

void Batch25d::Clear()
{
  vertices_.clear();
  colors_.clear();
}

void Batch25d::Vertex(const tf2::Vector3 & point, const Color & color)
{
  vertices_.push_back(point.x());
  vertices_.push_back(point.y());
  colors_.push_back(ToByte(color.r));
  colors_.push_back(ToByte(color.g));
  colors_.push_back(ToByte(color.b));
  colors_.push_back(255);
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

  const Color roof{0.84f, 0.83f, 0.80f};
  for (size_t i = 0; i + 2 < building.triangles.size(); i += 3) {
    Triangle(
      building.outline[static_cast<size_t>(building.triangles[i])] + top,
      building.outline[static_cast<size_t>(building.triangles[i + 1])] + top,
      building.outline[static_cast<size_t>(building.triangles[i + 2])] + top,
      roof);
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

  // The top, with a lighter middle so it reads as round.
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
    BuildBatch(buildings, crowns, up, right, lean, batch_);
    if (list_ == 0) {
      list_ = glGenLists(1);
    }
    glNewList(list_, GL_COMPILE);
    if (batch_.VertexCount() > 0) {
      glEnableClientState(GL_VERTEX_ARRAY);
      glEnableClientState(GL_COLOR_ARRAY);
      glVertexPointer(2, GL_DOUBLE, 0, batch_.Vertices().data());
      glColorPointer(4, GL_UNSIGNED_BYTE, 0, batch_.Colors().data());
      glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(batch_.VertexCount()));
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
