/*
* ImageManip.h
*
* Copyright (C) 2010 Southwest Research Institute <malban@swri.org>
*
* Created on: Mar 01, 2010
*     Author: Marc Alban
*/

#ifndef TILESETLAYER_H
#define TILESETLAYER_H

// C++ standard libraries
#include <string>
#include <vector>

#include <georeference/georeference.h>
#include <geospatial_index/wgs84_utm.h>

#include <multires_image/tile.h>
#include <multires_image/point.h>
#include <multires_image/bounding_box.h>

namespace multires_image
{

class TileSetLayer
{
public:
	TileSetLayer(const georeference::GeoReference& geo, const geospatial_index::WGS84UTM& utm,
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
  const georeference::GeoReference& m_geo;
  const geospatial_index::WGS84UTM& m_utm;
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

#endif // TILESETLAYER_H
