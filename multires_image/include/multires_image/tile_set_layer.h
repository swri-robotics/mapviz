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

#ifndef MULTIRES_IMAGE_TILE_SET_LAYER_H_
#define MULTIRES_IMAGE_TILE_SET_LAYER_H_

// C++ standard libraries
#include <string>
#include <vector>

#include <tf/transform_datatypes.h>

#include <swri_transform_util/georeference.h>

#include <multires_image/tile.h>

namespace multires_image
{
  class TileSetLayer
  {
  public:
    TileSetLayer(
      const swri_transform_util::GeoReference& geo,
      const std::string& path,
      int tileSize, int layer);

    ~TileSetLayer(void);

    bool Load();
    bool Load(const std::string extension);

    Tile* GetTile(int column, int row) { return m_tiles[column][row]; }

    void GetTileIndex(const tf::Point& position, int& row, int& column) const;
    void GetTileIndex(double x, double y, int& row, int& column) const;
    void GetTileRange(
      const tf::Point& top_left,
      const tf::Point& bottom_right,
      int& startRow, int& startColumn,
      int& endRow, int& endColumn) const;

    int RowCount() { return m_rows; }
    int ColumnCount() { return m_columns; }

  private:
    const swri_transform_util::GeoReference& m_geo;
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
