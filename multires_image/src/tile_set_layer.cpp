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

#include <multires_image/tile_set_layer.h>

// C++ standard libraries
#include <cmath>
#include <cstdio>

// QT libraries
#include <QString>

namespace multires_image
{
  TileSetLayer::TileSetLayer(const swri_transform_util::GeoReference& geo,
                  const std::string& path,
                  int tileSize, int layer) :
  m_geo(geo),
  m_path(path),
  m_tileSize(tileSize),
  m_layer(layer),
  m_scale(std::pow(2.0, m_layer)),
  m_expectTiles(true)
  {
    // Calculate the width and height in pixels of this layer
    float width = std::ceil(m_geo.Width() / std::pow(2.0f, layer));
    float height = std::ceil(m_geo.Height() / std::pow(2.0f, layer));

    // Calculate the number for tile rows and columns for this layer
    m_columns = std::ceil(width / tileSize);
    m_rows = std::ceil(height / tileSize);

    m_tiles.reserve(m_columns);
    for (int c = 0; c < m_columns; c++)
    {
      m_tiles.push_back(std::vector<Tile*>());
      m_tiles[c].reserve(m_rows);
    }
  }

  TileSetLayer::~TileSetLayer(void)
  {
  }

  bool TileSetLayer::Load()
  {
    return Load("jpg");
  }

  bool TileSetLayer::Load(const std::string extension)
  {
    bool needsTiles = false;

    for (int32_t c = 0; c < m_columns; c++)
    {
      for (int32_t r = 0; r < m_rows; r++)
      {
        std::string rowString = QString::number(r).toStdString();
        while (rowString.length() < 5) rowString = '0' + rowString;

        std::string columnString = QString::number(c).toStdString();
        while (columnString.length() < 5) columnString = '0' + columnString;

        // Get 4 corners of this tile
        int left   = c       * m_tileSize * m_scale;
        int top    = r       * m_tileSize * m_scale;
        int bottom = (r + 1) * m_tileSize * m_scale;
        int right  = (c + 1) * m_tileSize * m_scale;

        if (right > (int64_t)m_geo.Width())
        {
          right = m_geo.Width();
        }
        if (bottom > (int64_t)m_geo.Height())
        {
          bottom = m_geo.Height();
        }

        double x, y;
        m_geo.GetCoordinate(left, top, x, y);
        tf::Point top_left(x, y, 0);

        m_geo.GetCoordinate(right, top, x, y);
        tf::Point top_right(x, y, 0);

        m_geo.GetCoordinate(left, bottom, x, y);
        tf::Point bottom_left(x, y, 0);

        m_geo.GetCoordinate(right, bottom, x, y);
        tf::Point bottom_right(x, y, 0);

        m_tiles[c].push_back(new Tile(
          m_path + "/tile" + rowString + "x" +  columnString + "." + extension,
          c, r, m_layer, top_left, top_right, bottom_left, bottom_right));

        needsTiles |= !m_tiles[c][r]->Exists();
      }
    }

    if (needsTiles)
    {
      if (m_expectTiles)
      {
        printf("Error: Missing expected tiles\n");
        return false;
      }
    }

    return true;
  }

  void TileSetLayer::GetTileIndex(double x, double y, int& row, int& column) const
  {
    tf::Point position(x, y, 0);
    GetTileIndex(position, row, column);
  }

  void TileSetLayer::GetTileIndex(const tf::Point& position, int& row, int& column) const
  {
    int x, y;
    m_geo.GetPixel(position.x(), position.y(), x, y);

    column = static_cast<int>(x / (m_scale * m_tileSize));
    row = static_cast<int>(y / (m_scale * m_tileSize));
  }

  void TileSetLayer::GetTileRange(
      const tf::Point& top_left,
      const tf::Point& bottom_right,
      int& startRow, int& startColumn,
      int& endRow, int& endColumn) const
  {
    GetTileIndex(top_left.x(), top_left.y(), startRow, startColumn);
    if (startColumn < 0)
    {
      startColumn = 0;
    }
    if ((uint32_t)startColumn >= m_tiles.size())
    {
      startColumn = m_tiles.size() - 1;
    }
    if (startRow < 0)
    {
      startRow = 0;
    }
    if ((uint32_t)startRow >= m_tiles[0].size())
    {
      startRow = m_tiles[0].size() - 1;
    }

    GetTileIndex(bottom_right.x(), bottom_right.y(), endRow, endColumn);
    if (endColumn < 0)
    {
      endColumn = 0;
    }
    if ((uint32_t)endColumn >= m_tiles.size())
    {
      endColumn = m_tiles.size() - 1;
    }
    if (endRow < 0)
    {
      endRow = 0;
    }
    if ((uint32_t)endRow >= m_tiles[0].size())
    {
      endRow = m_tiles[0].size() - 1;
    }
  }
}

