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

#ifndef MULTIRES_IMAGE_TILE_SET_H_
#define MULTIRES_IMAGE_TILE_SET_H_

// C++ standard libraries
#include <string>
#include <vector>

#include <swri_transform_util/georeference.h>

#include <multires_image/tile_set_layer.h>

namespace multires_image
{
  class TileSet
  {
  public:
    explicit TileSet(const std::string& geofile);
    TileSet(const std::string& geofile, const std::string extension);
    explicit TileSet(const swri_transform_util::GeoReference& georeference);
    TileSet(const swri_transform_util::GeoReference& georeference,
            const std::string extension);

    ~TileSet(void);

    bool Load();

    int LayerCount() { return m_layerCount; }
    int TileSize() { return m_tileSize; }

    swri_transform_util::GeoReference& GeoReference() { return m_geo; }

    TileSetLayer* GetLayer(int layer) { return m_layers[layer]; }

  private:
    swri_transform_util::GeoReference  m_geo;
    int                           m_tileSize;
    int                           m_width;
    int                           m_height;

    std::string                   m_cacheDir;
    std::string                   m_extension;

    int                           m_layerCount;

    std::vector<TileSetLayer*>    m_layers;
  };
}

#endif  // MULTIRES_IMAGE_TILE_SET_H_
