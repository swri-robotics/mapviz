/*
* ImageManip.h
*
* Copyright (C) 2010 Southwest Research Institute <malban@swri.org>
*
* Created on: Mar 01, 2010
*     Author: Marc Alban
*/

#ifndef MULTIRES_IMAGE_TILE_H_
#define MULTIRES_IMAGE_TILE_H_

// C++ standard libraries
#include <string>

// QT libraries
#include <QImage>
#include <QMutex>

#include <transform_util/georeference.h>
#include <transform_util/utm_util.h>

#include <multires_image/point.h>

#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif

namespace multires_image
{
  class Tile
  {
  public:
    Tile(const transform_util::GeoReference& geo, const transform_util::UtmUtil& utm,
      const std::string& path, int column, int row, int level, const PointT<double>& topLeft,
      const PointT<double>& topRight, const PointT<double>& bottomLeft, const PointT<double>& bottomRight);
    ~Tile(void);

    void Initialize();
    bool Exists();
    bool HasUtm() const { return m_hasUtm; }
    bool Failed() const { return m_failed; }
    bool TextureLoaded() const { return m_textureLoaded; }
    const QImage& Image() const { return m_image; }
    long TileID() const { return m_tileId; }
    int Layer() const { return m_level; }
    int MemorySize() const { return m_memorySize; }
    int Row() const { return m_row; }
    int Column() const { return m_column; }

    bool LoadImageToMemory(bool gl = true);
    void UnloadImage();

    bool LoadTexture();
    void UnloadTexture();

    void Draw();
    void DrawUtm();
    void DrawRelative();

    void GetUtmPosition(
      double& top_left_x, double& top_left_y,
      double& top_right_x, double& top_right_y,
      double& bottom_right_x, double& bottom_right_y,
      double& bottom_left_x, double& bottom_left_y);

    void SetRelativePosition(
      double top_left_x, double top_left_y,
      double top_right_x, double top_right_y,
      double bottom_right_x, double bottom_right_y,
      double bottom_left_x, double bottom_left_y);

    void AdjustGeoReference(double latitude, double longitude);

  private:

    void ConvertToUtm();

    const transform_util::GeoReference& m_geo;
    const transform_util::UtmUtil& m_utm;
    const std::string   m_path;
    const int           m_column;
    const int           m_row;
    const int           m_level;

    PointT<double>      m_topLeft;
    PointT<double>      m_topRight;
    PointT<double>      m_bottomRight;
    PointT<double>      m_bottomLeft;

    PointT<double>      m_topLeftUtm;
    PointT<double>      m_topRightUtm;
    PointT<double>      m_bottomRightUtm;
    PointT<double>      m_bottomLeftUtm;


    PointT<double>      m_topLeftRelative;
    PointT<double>      m_topRightRelative;
    PointT<double>      m_bottomRightRelative;
    PointT<double>      m_bottomLeftRelative;


    bool                m_hasUtm;
    bool                m_failed;
    bool                m_textureLoaded;
    int                 m_dimension;
    int                 m_textureId;
    long                m_tileId;
    int                 m_memorySize;
    QImage              m_image;
    QMutex              m_mutex;
  };
}

#endif  // MULTIRES_IMAGE_TILE_H_
