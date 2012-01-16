/*
* ImageManip.h
*
* Copyright (C) 2010 Southwest Research Institute <malban@swri.org>
*
* Created on: Mar 01, 2010
*     Author: Marc Alban
*/

#ifndef TILESET_H
#define TILESET_H

// C++ standard libraries
#include <string>
#include <vector>

#include <georeference/georeference.h>
#include <geospatial_index/wgs84_utm.h>

#include "tile_set_layer.h"
#include "point.h"

class TileSet
{
public:

	TileSet(const std::string& geofile);
	TileSet(const std::string& geofile, const std::string extension);
	TileSet(const georeference::GeoReference& georeference);
	TileSet(const georeference::GeoReference& georeference,
	        const std::string extension);

	~TileSet(void);

	bool Load();

	int LayerCount() { return m_layerCount; }
	int TileSize() { return m_tileSize; }

	georeference::GeoReference& GeoReference() { return m_geo; }

	TileSetLayer* GetLayer(int layer) { return m_layers[layer]; }

	void AdjustGeoReference(double latitude, double longitude);

  void UtmToGeoreference(double easting, double northing, double& x, double& y);
  void GeoreferenceToUtm(double x, double y, double& easting, double& northing);

private:
  georeference::GeoReference m_geo;
  geospatial_index::WGS84UTM m_utm;
	int                        m_tileSize;
	int                        m_width;
	int                        m_height;

	std::string                m_cacheDir;
	std::string                m_extension;

	int                        m_layerCount;

	std::vector<TileSetLayer*> m_layers;
};

#endif // TILESET_H
