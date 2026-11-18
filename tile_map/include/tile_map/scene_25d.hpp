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
#include <map>
#include <tuple>
#include <vector>

#include <tf2/LinearMath/Vector3.hpp>
#include <tile_map/tile_source.hpp>

namespace tile_map
{
/// A building in the frame being drawn.
struct ProjectedBuilding
{
  /// See Building::id; 0 when unknown.
  int64_t id = 0;
  std::vector<tf2::Vector3> outline;
  /// Where each outline point is on the web mercator map (see
  /// LatLonToMercator()), x and y in turn, for draping imagery over the roof.
  /// Empty if the roof is never textured.
  std::vector<double> mercator;
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
  /// Where the crown is on the web mercator map, how many mercator units a
  /// metre is there, and which way east is in the drawn frame, for draping
  /// imagery over its top.  A zero scale means the top is never textured.
  double mercator_x = 0.0;
  double mercator_y = 0.0;
  double mercator_per_meter = 0.0;
  tf2::Vector3 east{1.0, 0.0, 0.0};
};

/**
 * Coloured and textured triangles for the 2.5D objects, in drawing order.
 *
 * The view stays top-down: a point h metres above the ground is drawn
 * h * lean metres up the screen.  There is no light source; walls and the
 * sides of crowns get fixed darker tones, so shapes read without shadows.
 * Given imagery tiles, roofs and treetops show the imagery that was under
 * them, lifted with them; where a roof isn't fully covered by loaded tiles it
 * is drawn plain instead.
 */
class Batch25d
{
public:
  /// A stretch of triangles drawn with one texture (0 for none).
  struct Run
  {
    uint32_t texture;
    size_t first;
    size_t count;
  };

  /// Empties the batch; the imagery is kept.
  void Clear();

  /// The imagery tiles to texture roofs and treetops with.  Tiles at finer
  /// levels are preferred; none draws everything plain.
  void SetImagery(const std::vector<ImageryTile> & tiles);

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
  /// Two texture coordinates per vertex (0 where untextured).
  const std::vector<float> & TexCoords() const {return texcoords_;}
  const std::vector<Run> & Runs() const {return runs_;}
  size_t VertexCount() const {return vertices_.size() / 2;}

private:
  struct Color
  {
    float r;
    float g;
    float b;
  };

  /// A drawn position and where it is on the web mercator map.
  struct MapPoint
  {
    tf2::Vector3 position;
    double mx;
    double my;
  };

  void Vertex(
    const tf2::Vector3 & point, const Color & color,
    uint32_t texture = 0, float u = 0.0f, float v = 0.0f);
  void Triangle(
    const tf2::Vector3 & a, const tf2::Vector3 & b, const tf2::Vector3 & c, const Color & color);
  void Quad(
    const tf2::Vector3 & a, const tf2::Vector3 & b, const tf2::Vector3 & c,
    const tf2::Vector3 & d, const Color & color);
  void Disc(const tf2::Vector3 & center, double radius, const Color & color);
  /// Adds a triangle draped with imagery, cut along tile edges.  False, and
  /// nothing added, if loaded tiles don't cover all of it.
  bool TexturedTriangle(const MapPoint & a, const MapPoint & b, const MapPoint & c);

  std::vector<double> vertices_;
  std::vector<uint8_t> colors_;
  std::vector<float> texcoords_;
  std::vector<Run> runs_;

  std::map<std::tuple<int32_t, int64_t, int64_t>, uint32_t> imagery_;
  /// The levels in imagery_, finest first.
  std::vector<int32_t> levels_;
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
 * objects, the imagery, the lean or the direction of "up" on screen change;
 * otherwise a compiled display list is replayed.
 */
class Renderer25d : protected QOpenGLFunctions_1_1
{
public:
  /// Needs the canvas's GL context.  `revision` must change whenever the
  /// objects, their positions or the imagery tiles (including which textures
  /// they are drawn with) do, since the display list refers to the textures.
  void Draw(
    const std::vector<const ProjectedBuilding *> & buildings,
    const std::vector<const ProjectedCrown *> & crowns,
    const std::vector<ImageryTile> & imagery,
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
