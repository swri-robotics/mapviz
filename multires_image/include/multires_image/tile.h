// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef MULTIRES_IMAGE_TILE_H_
#define MULTIRES_IMAGE_TILE_H_

// C++ standard libraries
#include <string>

// QT libraries
#include <QImage>
#include <QMutex>

#include <tf/transform_datatypes.h>

#include <swri_transform_util/transform.h>

#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif

namespace multires_image
{
  class Tile
  {
  public:
    Tile(
      const std::string& path, int column, int row, int level,
      const tf::Point& topLeft,
      const tf::Point& topRight,
      const tf::Point& bottomLeft,
      const tf::Point& bottomRight);
    ~Tile(void);

    bool Exists();
    bool Failed() const { return m_failed; }
    bool TextureLoaded() const { return m_textureLoaded; }
    const QImage& Image() const { return m_image; }
    int64_t TileID() const { return m_tileId; }
    int Layer() const { return m_level; }
    int MemorySize() const { return m_memorySize; }
    int Row() const { return m_row; }
    int Column() const { return m_column; }

    bool LoadImageToMemory(bool gl = true);
    void UnloadImage();

    bool LoadTexture();
    void UnloadTexture();

    void Draw();

    void Transform(const swri_transform_util::Transform& transform);
    void Transform(const swri_transform_util::Transform& transform, const swri_transform_util::Transform& offset_tf);

  private:
    const std::string   m_path;
    const int           m_column;
    const int           m_row;
    const int           m_level;

    tf::Point           m_top_left;
    tf::Point           m_top_right;
    tf::Point           m_bottom_right;
    tf::Point           m_bottom_left;

    tf::Point           m_transformed_top_left;
    tf::Point           m_transformed_top_right;
    tf::Point           m_transformed_bottom_right;
    tf::Point           m_transformed_bottom_left;

    bool                m_failed;
    bool                m_textureLoaded;
    int                 m_dimension;
    int                 m_textureId;
    int64_t             m_tileId;
    int                 m_memorySize;
    QImage              m_image;
    QMutex              m_mutex;
  };
}

#endif  // MULTIRES_IMAGE_TILE_H_
