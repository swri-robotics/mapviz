/*
* ImageManip.h
*
* Copyright (C) 2010 Southwest Research Institute <malban@swri.org>
*
* Created on: Mar 01, 2010
*     Author: Marc Alban
*/

#include <multires_image/tile_set.h>

// C++ standard libraries
#include <algorithm>
#include <cmath>
#include <cstdlib>

// QT libraries
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QString>

#include <multires_image/point.h>

namespace multires_image
{

TileSet::TileSet(const std::string& geofile) :
  m_geo(geofile),
  m_extension("jpg")
{
}

TileSet::TileSet(const std::string& geofile, const std::string extension) :
  m_geo(geofile),
  m_extension(extension)
{
}

TileSet::TileSet(const georeference::GeoReference& georeference) :
  m_geo(georeference),
  m_extension("jpg")
{
}

TileSet::TileSet(const georeference::GeoReference& georeference,
                 const std::string extension) :
  m_geo(georeference),
  m_extension(extension)
{
}

TileSet::~TileSet(void)
{
	// Free each of the layers.
	for (unsigned int i = 0; i < m_layers.size(); i++)
	{
		delete m_layers[i];
	}
}

bool TileSet::Load()
{
  if (!m_geo.Load())
  {
    return false;
  }

  m_cacheDir = m_geo.Path();
  m_width = m_geo.Width();
  m_height = m_geo.Height();
  m_tileSize = m_geo.TileSize();

  int maxDimension = std::max(m_width, m_height);
  m_layerCount = (int)ceil(log((float)maxDimension / m_tileSize) / log(2.0f)) + 1;
  m_layers.reserve(m_layerCount);
  
	// Check if the cache directory for this image exists.
	QDir directory(m_cacheDir.c_str());
	if (!directory.exists())
	{
	  return false;
	}

	// Load each layer.
	for (int i = 0; i < m_layerCount; i++)
	{
		QString layerNum = QString::number(i);

		// Check if this layer exists in the cache.
		QDir layerDir(directory.absolutePath() + "/layer" + layerNum);
		if (!layerDir.exists(layerDir.absolutePath()))
		{
			return false;
		}

    // Load the base layer.
    m_layers.push_back(new TileSetLayer(m_geo, m_utm, layerDir.absolutePath().toStdString(), m_tileSize, i));

		if (!m_layers[i]->Load(m_extension))
			return false;
	}

	return true;
}

void TileSet::UtmToGeoreference(double easting, double northing, double& x, double& y)
{
  x = 0;
  y = 0;
  if (m_geo.Datum() == "wgs84")
  {
    if (m_geo.Projection() == "utm")
    {
      x = easting;
      y = northing;
    }
    else if (m_geo.Projection() == "geographic")
    {
      // TODO UTM zone should not be hard coded.
      m_utm.GetLatLong(14, 'R', easting, northing, y, x);
    }
  }
}

void TileSet::GeoreferenceToUtm(double x, double y, double& easting, double& northing)
{
  easting = 0;
  northing = 0;
  if (m_geo.Datum() == "wgs84")
  {
    if (m_geo.Projection() == "utm")
    {
      easting = x;
      northing = y;
    }
    else if (m_geo.Projection() == "geographic")
    {
      int zone;
      char band;
      m_utm.GetUtm(y, x, zone, band, easting, northing);
    }
  }
}

void TileSet::AdjustGeoReference(double latitude, double longitude)
{	
	for (uint32_t i = 0; i < m_layers.size(); i++)
	{
		m_layers[i]->AdjustGeoReference(latitude, longitude);
	}
}

}
