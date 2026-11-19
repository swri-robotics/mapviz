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

#include <QImage>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLFunctions_1_1>
#include <QOpenGLTexture>

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <tuple>
#include <utility>
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
  /// The middle of the outline's bounding box, and the distance from it to
  /// the farthest corner, for leaving out buildings that are off screen.
  tf2::Vector3 center;
  double extent = 0.0;
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
 * Imagery tiles packed into one texture, so the whole 2.5D scene can be drawn
 * with a single texture and a single draw call.
 *
 * Tiles are scaled to fit (finer levels first) and a corner is kept plain
 * white for the parts that aren't draped with imagery.  Texture coordinates
 * follow GL: v grows upward, with the image's top row at v = 1.
 */
class ImageryAtlas
{
public:
  /// Width and height of the atlas, in pixels.
  static constexpr int SIZE = 2048;
  /// The largest a tile is stored at.
  static constexpr int MAX_SLOT = 256;

  /// Packs the tiles' images.  Tiles with null images are left out, and
  /// coarser levels are dropped first if there are too many to fit.
  void Build(const std::vector<std::pair<ImageryTile, QImage>> & tiles);

  bool Empty() const {return slots_.empty();}
  const QImage & Image() const {return image_;}
  /// The levels packed, finest first.
  const std::vector<int32_t> & Levels() const {return levels_;}

  /// Where a tile is: u0, v0, u1, v1 (left, bottom, right, top).  False if
  /// it isn't in the atlas.
  bool Find(int32_t level, int64_t x, int64_t y, std::array<float, 4> & rect) const;

  /// A texel that is plain white, for drawing in vertex colours alone.
  float WhiteU() const {return white_u_;}
  float WhiteV() const {return white_v_;}

private:
  QImage image_;
  std::map<std::tuple<int32_t, int64_t, int64_t>, std::array<float, 4>> slots_;
  std::vector<int32_t> levels_;
  float white_u_ = 0.0f;
  float white_v_ = 0.0f;
};

/**
 * Triangles for the 2.5D objects, ready for one draw call.
 *
 * The view stays top-down: a point h metres above the ground is drawn
 * h * lean metres up the screen.  The batch keeps each vertex on the ground
 * in x and y and its height in z; LeanMatrix() does the rest when drawing,
 * and the depth test sorts out what is in front, so the triangles don't
 * depend on which way the screen is turned or on the lean.
 *
 * There is no light source.  Walls get tones by which way they face that are
 * the same for opposite faces, and crowns have darker sides than tops, so
 * shapes read without shadows.  Given an atlas, roofs and treetops show the
 * imagery that was under them; where a roof isn't fully covered by the atlas
 * it is drawn plain instead.
 *
 * Detail follows the scale (metres per pixel): crowns a few pixels across get
 * fewer sides, no trunk or sides, and a plain top.
 */
class Batch25d
{
public:
  void Clear();

  /// The imagery to drape over roofs and treetops; null or empty draws them
  /// plain.  The atlas must outlive the batch's use of it.
  void SetAtlas(const ImageryAtlas * atlas) {atlas_ = atlas;}

  /// Metres per pixel on screen, for choosing how much detail to draw.
  void SetScale(double scale) {scale_ = scale;}

  /// Walls, then the roof.
  void AddBuilding(const ProjectedBuilding & building);

  /// A trunk and the sides of the crown, then its top.
  void AddCrown(const ProjectedCrown & crown);

  /// Three coordinates per vertex: x and y on the ground, z the height.
  const std::vector<double> & Vertices() const {return vertices_;}
  /// Four bytes (RGBA) per vertex.
  const std::vector<uint8_t> & Colors() const {return colors_;}
  /// Two atlas texture coordinates per vertex.
  const std::vector<float> & TexCoords() const {return texcoords_;}
  size_t VertexCount() const {return vertices_.size() / 3;}

  /// True if a vertex shows imagery rather than the atlas's white texel.
  bool Draped(size_t vertex) const;

private:
  struct Color
  {
    float r;
    float g;
    float b;
  };

  /// A drawn position (z the height) and where it is on the mercator map.
  struct MapPoint
  {
    tf2::Vector3 position;
    double mx;
    double my;
  };

  /// A clipped piece of a triangle: at most seven corners after four cuts.
  struct Polygon
  {
    MapPoint points[8];
    int count = 0;
  };

  void Vertex(const tf2::Vector3 & point, const Color & color, float u, float v);
  void Plain(const tf2::Vector3 & point, const Color & color);
  void Triangle(
    const tf2::Vector3 & a, const tf2::Vector3 & b, const tf2::Vector3 & c, const Color & color);
  void Quad(
    const tf2::Vector3 & a, const tf2::Vector3 & b, const tf2::Vector3 & c,
    const tf2::Vector3 & d, const Color & color);
  /// A flat disc at the centre's height.
  void Disc(const tf2::Vector3 & center, double radius, int segments, const Color & color);
  /// Adds a triangle draped with imagery, cut along tile edges.  False, and
  /// nothing added, if the atlas doesn't cover all of it.
  bool TexturedTriangle(const MapPoint & a, const MapPoint & b, const MapPoint & c);

  std::vector<double> vertices_;
  std::vector<uint8_t> colors_;
  std::vector<float> texcoords_;
  const ImageryAtlas * atlas_ = nullptr;
  double scale_ = 0.1;
};

/// Depth given to each metre of height, so higher points are drawn in front.
/// It keeps heights up to 500 m inside mapviz's -0.5..0.5 depth range.
constexpr double DEPTH_PER_METER = 0.0008;

/**
 * The matrix (column major, for glMultMatrixd) that draws a batch: it moves
 * each vertex lean * z up the screen, along `up` in the frame being drawn,
 * and sets its depth from z so that higher points are in front.  In this
 * view, of two points on the same line of sight the higher one is always the
 * nearer, which is what makes depth alone enough to draw them in order.
 */
void LeanMatrix(const tf2::Vector3 & up, double lean, double matrix[16]);

/// One vertex as uploaded: position, texture coordinates, colour.
struct GpuVertex25d
{
  float x;
  float y;
  float z;
  float u;
  float v;
  uint8_t rgba[4];
};

/**
 * Everything Renderer25d uploads, built without OpenGL so it can be made on a
 * worker thread.
 *
 * Vertices are stored as floats relative to the middle of the batch, so
 * they keep centimetre precision in frames far from their origin.  Their
 * texture coordinates point into the atlas image as it is stored, with its
 * top row first, so the image can be uploaded without flipping it.
 */
struct Mesh25d
{
  std::vector<GpuVertex25d> vertices;
  /// Added back to the stored vertices when drawing.
  double origin_x = 0.0;
  double origin_y = 0.0;
  /// The imagery the vertices are draped with; null or empty when nothing is.
  std::shared_ptr<const ImageryAtlas> atlas;
};

/// Builds the triangles for the buildings and crowns (see Batch25d).  Doesn't
/// touch OpenGL or anything but its arguments, so it is safe to call from any
/// thread.
Mesh25d BuildMesh25d(
  const std::vector<ProjectedBuilding> & buildings,
  const std::vector<ProjectedCrown> & crowns,
  const std::shared_ptr<const ImageryAtlas> & atlas, double scale);

/**
 * Draws a Mesh25d in one call from a vertex buffer, with one texture (the
 * atlas) and the depth test on.  Turning the map or changing the lean only
 * changes the matrix; the buffer is refilled only by Upload().
 */
class Renderer25d : protected QOpenGLFunctions_1_1
{
public:
  /// Uploads the mesh, and its atlas if that changed.  Needs the GL context.
  void Upload(Mesh25d mesh);

  /// Draws what was last uploaded.  Needs the GL context.
  void Draw(double lean);

  bool Built() const {return built_;}

private:
  void InitializeGl();

  bool gl_initialized_ = false;
  bool built_ = false;
  QOpenGLBuffer buffer_{QOpenGLBuffer::VertexBuffer};
  GLsizei vertex_count_ = 0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  std::unique_ptr<QOpenGLTexture> texture_;
  /// The atlas in `texture_`, so an unchanged atlas isn't uploaded again.
  std::shared_ptr<const ImageryAtlas> texture_atlas_;
};
}  // namespace tile_map

#endif  // TILE_MAP__SCENE_25D_HPP_
