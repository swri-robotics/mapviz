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
#ifndef MULTIRES_IMAGE_TILESET_H_
#define MULTIRES_IMAGE_TILESET_H_

// C++ standard libraries
#include <string>
#include <vector>

#include <transform_util/georeference.h>
#include <transform_util/utm_util.h>

#include <multires_image/tile_set_layer.h>
#include <multires_image/point.h>

namespace multires_image
{
  class TileSet
  {
  public:

    TileSet(const std::string& geofile);
    TileSet(const std::string& geofile, const std::string extension);
    TileSet(const transform_util::GeoReference& georeference);
    TileSet(const transform_util::GeoReference& georeference,
            const std::string extension);

    ~TileSet(void);

    bool Load();

    int LayerCount() { return m_layerCount; }
    int TileSize() { return m_tileSize; }

    transform_util::GeoReference& GeoReference() { return m_geo; }

    TileSetLayer* GetLayer(int layer) { return m_layers[layer]; }

    void AdjustGeoReference(double latitude, double longitude);

    void UtmToGeoreference(double easting, double northing, double& x, double& y);
    void GeoreferenceToUtm(double x, double y, double& easting, double& northing);

  private:
    transform_util::GeoReference  m_geo;
    transform_util::UtmUtil       m_utm;
    int                           m_tileSize;
    int                           m_width;
    int                           m_height;

    std::string                   m_cacheDir;
    std::string                   m_extension;

    int                           m_layerCount;

    std::vector<TileSetLayer*>    m_layers;
  };
}

#endif  // MULTIRES_IMAGE_TILESET_H_
