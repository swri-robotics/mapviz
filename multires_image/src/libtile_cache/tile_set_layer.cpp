/*
* ImageManip.h
*
* Copyright (C) 2010 Southwest Research Institute <malban@swri.org>
*
* Created on: Mar 01, 2010
*     Author: Marc Alban
*/

// C++ standard libraries
#include <cmath>
#include <cstdio>

// QT libraries
#include <QString>

#include "tile_set_layer.h"

TileSetLayer::TileSetLayer(const georeference::GeoReference& geo, 
                const geospatial_index::WGS84UTM& utm,
                const std::string& path, 
                int tileSize, int layer) :
m_geo(geo),
m_utm(utm),
m_path(path),
m_tileSize(tileSize),
m_layer(layer),
m_scale(std::pow(2, m_layer)),
m_expectTiles(true)
{
  // Calculate the width and height in pixels of this layer
  float width = ceil(m_geo.Width() / pow(2.0f, layer));
  float height = ceil(m_geo.Height() / pow(2.0f, layer));

  // Calculate the number for tile rows and columns for this layer
	m_columns = (int)ceil((float)width / tileSize);
	m_rows = (int)ceil((float)height / tileSize);

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
  return this->Load("jpg");
}

bool TileSetLayer::Load(const std::string extension)
{
  printf("Loading layer %d: expecting %dx%d tiles\n.", m_layer, m_rows, m_columns);

	bool needsTiles = false;

	for (int c = 0; c < m_columns; c++)
	{
		for (int r = 0; r < m_rows; r++)
		{
			std::string rowString = QString::number(r).toStdString();
			while(rowString.length() < 5) rowString = '0' + rowString;

			std::string columnString = QString::number(c).toStdString();
			while(columnString.length() < 5) columnString = '0' + columnString;

      // Get 4 corners of this tile
      int left = c * m_tileSize * m_scale;
      int top = r * m_tileSize * m_scale;
      int bottom = (r + 1) * m_tileSize * m_scale;
      int right = (c + 1) * m_tileSize * m_scale;
      
      if (right > m_geo.Width()) right = m_geo.Width();
      if (bottom > m_geo.Height()) bottom = m_geo.Height();

      PointT<double> top_left, top_right, bottom_left, bottom_right;
      m_geo.GetCoordinate(left, top, top_left.X, top_left.Y);
      m_geo.GetCoordinate(right, top, top_right.X, top_right.Y);
      m_geo.GetCoordinate(left, bottom, bottom_left.X, bottom_left.Y);
      m_geo.GetCoordinate(right, bottom, bottom_right.X, bottom_right.Y);

			m_tiles[c].push_back(new Tile(
			  m_geo,
			  m_utm,
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
		/*
		if (!ImageManip.TileImage(image, path, tileSize))
		{
			Console.WriteLine("Failed to generate tiles from " + image);	
		}
		*/
	}

	return true;
}

void TileSetLayer::GetTileIndex(double x, double y, int& row, int& column) const
{
  PointT<double> position(x, y);
  GetTileIndex(position, row, column);
}

void TileSetLayer::GetTileIndex(const PointT<double>& position, int& row, int& column) const
{
  int x, y;
  m_geo.GetPixel(position.X, position.Y, x, y);

  column = static_cast<int>(x / (m_scale * m_tileSize));
  row = static_cast<int>(y / (m_scale * m_tileSize));
}

void TileSetLayer::GetTileRange(const BoundingBox<double>& area,
								int& startRow, int& startColumn, 
								int& endRow, int& endColumn) const
{
  GetTileIndex(area.topLeft.X, area.topLeft.Y, startRow, startColumn);
	if (startColumn < 0)
	{
		startColumn = 0;
	}
	if (startColumn >= m_tiles.size())
	{
		startColumn = m_tiles.size() - 1;
	}
  if (startRow < 0)
	{
		startRow = 0;
	}
	if (startRow >= m_tiles[0].size())
	{
		startRow = m_tiles[0].size() - 1;
	}

  GetTileIndex(area.bottomRight.X, area.bottomRight.Y, endRow, endColumn);
	if (endColumn < 0)
	{
		endColumn = 0;
	}
	if (endColumn >= m_tiles.size())
	{
		endColumn = m_tiles.size() - 1;
	}
	if (endRow < 0)
	{
		endRow = 0;
	}
	if (endRow >= m_tiles[0].size())
	{
		endRow = m_tiles[0].size() - 1;
	}
}

void TileSetLayer::AdjustGeoReference(double latitude, double longitude)
{
	for (int c = 0; c < m_columns; c++)
	{
		for (int r = 0; r < m_rows; r++)
		{
			m_tiles[c][r]->AdjustGeoReference(latitude, longitude);
		}
	}
}

