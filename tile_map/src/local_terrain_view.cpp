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

#include <tile_map/local_terrain_view.hpp>

#include <QImage>

#include <cmath>
#include <iterator>
#include <memory>
#include <utility>

namespace tile_map
{
void LocalTerrainView::SetSet(const std::shared_ptr<const ElevationSet> & set)
{
  if (set != set_) {
    set_ = set;
    block_ = ContourLayer::Block();
    // Textures can only be freed with the GL context current.
    refresh_ = true;
  }
}

void LocalTerrainView::SetShading(const ShadingParameters & shading)
{
  if (shading != shading_) {
    shading_ = shading;
    refresh_ = true;
  }
}

void LocalTerrainView::SetTransform(const swri_transform_util::Transform & wgs84_to_target)
{
#ifdef TILE_MAP_HAVE_TRANSFORM_EQUALITY
  if (wgs84_to_target == transform_) {
    return;
  }
#endif
  transform_ = wgs84_to_target;
  for (auto & entry : tiles_) {
    Project(entry.first, entry.second);
  }
}

void LocalTerrainView::SetView(
  double latitude, double longitude, double scale, int32_t width, int32_t height)
{
  block_ = ContourLayer::LocalBlockFor(set_, latitude, longitude, scale, width, height);
}

void LocalTerrainView::Project(const TileKey & key, Tile & tile) const
{
  const auto [layer, row, column] = key;
  const double size = std::pow(2.0, layer) * set_->TileSize();
  tile.corners.clear();
  tile.corners.reserve((SUBDIVISIONS + 1) * (SUBDIVISIONS + 1));
  for (int j = 0; j <= SUBDIVISIONS; ++j) {
    for (int i = 0; i <= SUBDIVISIONS; ++i) {
      double latitude;
      double longitude;
      set_->PixelToLatLon(
        (static_cast<double>(column) + static_cast<double>(i) / SUBDIVISIONS) * size,
        (static_cast<double>(row) + static_cast<double>(j) / SUBDIVISIONS) * size,
        latitude, longitude);
      tile.corners.push_back(transform_ * tf2::Vector3(longitude, latitude, 0.0));
    }
  }
}

void LocalTerrainView::Draw()
{
  if (!gl_initialized_) {
    initializeOpenGLFunctions();
    gl_initialized_ = true;
  }

  if (refresh_) {
    tiles_.clear();
    refresh_ = false;
  }
  // Drop what is out of view.
  for (auto it = tiles_.begin(); it != tiles_.end(); ) {
    const auto [layer, row, column] = it->first;
    const bool keep = block_.set && layer == block_.level &&
      column >= block_.x && column < block_.x + block_.columns &&
      row >= block_.y && row < block_.y + block_.rows;
    it = keep ? std::next(it) : tiles_.erase(it);
  }
  if (!block_.set || block_.columns == 0) {
    return;
  }

  // Shade tiles that came into view, a few per frame.
  const double cell_size = std::sqrt(set_->PixelWidthMeters() * set_->PixelHeightMeters()) *
    std::pow(2.0, block_.level);
  int shaded = 0;
  for (int64_t row = block_.y; row < block_.y + block_.rows && shaded < TILES_PER_FRAME; ++row) {
    for (int64_t column = block_.x;
      column < block_.x + block_.columns && shaded < TILES_PER_FRAME; ++column)
    {
      const TileKey key{block_.level, row, column};
      if (tiles_.count(key)) {
        continue;
      }
      Tile & tile = tiles_[key];
      ++shaded;
      const auto heights = set_->ReadTile(block_.level, row, column);
      if (!heights) {
        continue;
      }
      const QImage shade = SlopeShade(
        *heights, set_->TileSize(), set_->TileSize(), cell_size, shading_);
      if (shade.isNull()) {
        continue;
      }
      // Row 0 of the image is the top of the tile, at texture coordinate 0.
      tile.texture = std::make_unique<QOpenGLTexture>(
        shade.convertToFormat(QImage::Format_RGBA8888), QOpenGLTexture::DontGenerateMipMaps);
      tile.texture->setMinificationFilter(QOpenGLTexture::Linear);
      tile.texture->setMagnificationFilter(QOpenGLTexture::Linear);
      tile.texture->setWrapMode(QOpenGLTexture::ClampToEdge);
      Project(key, tile);
    }
  }

  glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_COLOR_BUFFER_BIT);
  glEnable(GL_TEXTURE_2D);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColor4d(1.0, 1.0, 1.0, 1.0);
  const int n = SUBDIVISIONS + 1;
  for (const auto & entry : tiles_) {
    const Tile & tile = entry.second;
    if (!tile.texture || tile.corners.size() != static_cast<size_t>(n * n)) {
      continue;
    }
    tile.texture->bind();
    glBegin(GL_QUADS);
    for (int j = 0; j < SUBDIVISIONS; ++j) {
      for (int i = 0; i < SUBDIVISIONS; ++i) {
        const int corners[4][2] = {{i, j}, {i + 1, j}, {i + 1, j + 1}, {i, j + 1}};
        for (const auto & corner : corners) {
          const tf2::Vector3 & p = tile.corners[corner[1] * n + corner[0]];
          glTexCoord2d(
            static_cast<double>(corner[0]) / SUBDIVISIONS,
            static_cast<double>(corner[1]) / SUBDIVISIONS);
          glVertex2d(p.x(), p.y());
        }
      }
    }
    glEnd();
    tile.texture->release();
  }
  glPopAttrib();
}
}  // namespace tile_map
