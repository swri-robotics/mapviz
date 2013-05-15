// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-58058A
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Steve Dellenback <sdellenback@swri.org> (210) 522-3914
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
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

#include <transform_util/transform.h>

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

    void Transform(const transform_util::Transform& transform);

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
