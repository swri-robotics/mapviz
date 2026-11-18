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

#ifndef TILE_MAP__SCENE_25D_HPP_
#define TILE_MAP__SCENE_25D_HPP_

#include <QOpenGLFunctions_1_1>

#include <cstdint>
#include <vector>

#include <tf2/LinearMath/Vector3.hpp>

namespace tile_map
{
/// A building in the frame being drawn.
struct ProjectedBuilding
{
  /// See Building::id; 0 when unknown.
  int64_t id = 0;
  std::vector<tf2::Vector3> outline;
  /// See Building::walls.
  std::vector<bool> walls;
  /// Three indices into `outline` per roof triangle.
  std::vector<int> triangles;
  double height = 0.0;
  double min_height = 0.0;
  /// True when the outline runs counterclockwise in the drawn frame.
  bool counterclockwise = true;
};

/// A tree crown in the frame being drawn.
struct ProjectedCrown
{
  tf2::Vector3 center;
  double radius = 0.0;
  double height = 0.0;
  /// Multiplies the crown's colours, so neighbouring trees differ a little.
  float shade = 1.0f;
};

/**
 * Coloured triangles for the 2.5D objects, in the order they are drawn.
 *
 * The view stays top-down: a point h metres above the ground is drawn
 * h * lean metres up the screen.  There is no light source; walls and the
 * sides of crowns get fixed darker tones, so shapes read without shadows.
 */
class Batch25d
{
public:
  void Clear();

  /// Walls facing the viewer, then the roof.
  void AddBuilding(
    const ProjectedBuilding & building,
    const tf2::Vector3 & up, const tf2::Vector3 & right, double lean);

  /// A trunk, the underside and sides of the crown, then its top.
  void AddCrown(
    const ProjectedCrown & crown,
    const tf2::Vector3 & up, const tf2::Vector3 & right, double lean);

  /// Two coordinates per vertex.
  const std::vector<double> & Vertices() const {return vertices_;}
  /// Four bytes (RGBA) per vertex.
  const std::vector<uint8_t> & Colors() const {return colors_;}
  size_t VertexCount() const {return vertices_.size() / 2;}

private:
  struct Color
  {
    float r;
    float g;
    float b;
  };

  void Vertex(const tf2::Vector3 & point, const Color & color);
  void Triangle(
    const tf2::Vector3 & a, const tf2::Vector3 & b, const tf2::Vector3 & c, const Color & color);
  void Quad(
    const tf2::Vector3 & a, const tf2::Vector3 & b, const tf2::Vector3 & c,
    const tf2::Vector3 & d, const Color & color);
  void Disc(const tf2::Vector3 & center, double radius, const Color & color);

  std::vector<double> vertices_;
  std::vector<uint8_t> colors_;
};

/// Where an object is in the back-to-front order: the smaller, the nearer
/// the bottom of the screen and the later it is drawn.
double FrontEdge(const ProjectedBuilding & building, const tf2::Vector3 & up);
double FrontEdge(const ProjectedCrown & crown, const tf2::Vector3 & up);

/**
 * Sorts buildings and crowns together from the top of the screen down, since
 * the canvas has no depth buffer, and adds them to `batch` in that order.
 */
void BuildBatch(
  const std::vector<const ProjectedBuilding *> & buildings,
  const std::vector<const ProjectedCrown *> & crowns,
  const tf2::Vector3 & up, const tf2::Vector3 & right, double lean,
  Batch25d & batch);

/**
 * Draws buildings and crowns.  The triangles are rebuilt only when the
 * objects, the lean or the direction of "up" on screen change; otherwise a
 * compiled display list is replayed.
 */
class Renderer25d : protected QOpenGLFunctions_1_1
{
public:
  /// Needs the canvas's GL context.  `revision` must change whenever the
  /// objects or their positions do.
  void Draw(
    const std::vector<const ProjectedBuilding *> & buildings,
    const std::vector<const ProjectedCrown *> & crowns,
    double lean, uint64_t revision);

private:
  bool gl_initialized_ = false;
  GLuint list_ = 0;
  uint64_t built_revision_ = 0;
  double built_lean_ = -1.0;
  tf2::Vector3 built_up_;
  Batch25d batch_;
};
}  // namespace tile_map

#endif  // TILE_MAP__SCENE_25D_HPP_
