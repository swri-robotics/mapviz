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

#ifndef MULTIRES_IMAGE_TILE_SET_LAYER_H_
#define MULTIRES_IMAGE_TILE_SET_LAYER_H_

// C++ standard libraries
#include <string>
#include <vector>

#include <transform_util/georeference.h>
#include <transform_util/utm_util.h>

#include <multires_image/tile.h>
#include <multires_image/point.h>
#include <multires_image/bounding_box.h>

namespace multires_image
{

  class TileSetLayer
  {
  public:
    TileSetLayer(
      const transform_util::GeoReference& geo,
      const transform_util::UtmUtil& utm,
      const std::string& path,
      int tileSize, int layer);

    ~TileSetLayer(void);

    bool Load();
    bool Load(const std::string extension);

    Tile* GetTile(int column, int row) { return m_tiles[column][row]; }

    void GetTileIndex(const PointT<double>& position, int& row, int& column) const;
    void GetTileIndex(double x, double y, int& row, int& column) const;
    void GetTileRange(const BoundingBox<double>& area,
      int& startRow, int& startColumn,
      int& endRow, int& endColumn) const;

    int RowCount() { return m_rows; }
    int ColumnCount() { return m_columns; }

    void AdjustGeoReference(double latitude, double longitude);

  private:
    const transform_util::GeoReference& m_geo;
    const transform_util::UtmUtil& m_utm;
    const std::string      m_path;
    const int              m_tileSize;
    const int              m_layer;
    const double           m_scale;

    bool                   m_expectTiles;

    int                    m_columns;
    int                    m_rows;

    std::vector<std::vector<Tile*> > m_tiles;
  };
}

#endif  // MULTIRES_IMAGE_TILE_SET_LAYER_H_
