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

#include <multires_image/tile_set.h>

// C++ standard libraries
#include <cmath>
#include <cstdlib>
#include <algorithm>

// QT libraries
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QString>

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

  TileSet::TileSet(const transform_util::GeoReference& georeference) :
    m_geo(georeference),
    m_extension("jpg")
  {
  }

  TileSet::TileSet(const transform_util::GeoReference& georeference,
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
      m_layers.push_back(new TileSetLayer(m_geo, layerDir.absolutePath().toStdString(), m_tileSize, i));

      if (!m_layers[i]->Load(m_extension))
        return false;
    }

    return true;
  }
}
