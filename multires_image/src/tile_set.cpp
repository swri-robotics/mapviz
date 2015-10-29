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

  TileSet::TileSet(const swri_transform_util::GeoReference& georeference) :
    m_geo(georeference),
    m_extension("jpg")
  {
  }

  TileSet::TileSet(const swri_transform_util::GeoReference& georeference,
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
    m_extension = m_geo.Extension();

    float max_dim = std::max(m_width, m_height);
    m_layerCount = std::ceil(std::log(max_dim / m_tileSize) / std::log(2.0f)) + 1;
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
