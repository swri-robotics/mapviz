// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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
// ARE DISCLAIMED. IN NO EVENT SHALL Southwest Research Institute® BE LIABLE 
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// *****************************************************************************

#include <tile_map/tile_source.h>
#include <sstream>
#include <boost/lexical_cast.hpp>

namespace tile_map
{
  /**
   * Default constructor.
   *
   * Not actually used in the code, this is only necessary so that this class can
   * be used by STL data structures.
   */
  TileSource::TileSource() :
      coord_order_(ZYX),
      is_custom_(false),
      max_zoom_(-1)
  {
  }

  /**
   * Creates a new tile source from a set of known parameters.
   *
   * @param[in] name A user-friendly display name
   * @param[in] base_url The base HTTP URL of the data source; e. g.:
   *   "http://tile.stamen.com/terrain/"
   * @param[in] coord_order The order in which coordinates should be appended
   *   to the URL
   * @param is_custom If this is a custom (i. e. not one of the default)
   *   tile source; custom sources are saved and loaded from our settings
   * @param[in] max_zoom The maximum zoom level
   * @param[in] suffix A suffix that should be appended to the URL; i. e. ".jpg"
   */
  TileSource::TileSource(const QString& name,
                         const QString& base_url,
                         COORD_ORDER coord_order,
                         bool is_custom,
                         int32_t max_zoom,
                         const QString& suffix) :
      base_url_(base_url),
      coord_order_(coord_order),
      is_custom_(is_custom),
      max_zoom_(max_zoom),
      name_(name),
      suffix_(suffix)
  {
  }

  /**
   * Copy constructor
   */
  TileSource::TileSource(const TileSource& tile_source) :
      base_url_(tile_source.base_url_),
      coord_order_(tile_source.coord_order_),
      is_custom_(tile_source.is_custom_),
      max_zoom_(tile_source.max_zoom_),
      name_(tile_source.name_),
      suffix_(tile_source.suffix_)
  {
  }

  const QString& TileSource::GetBaseUrl() const
  {
    return base_url_;
  }

  void TileSource::SetBaseUrl(const QString& base_url)
  {
    base_url_ = base_url;
  }

  TileSource::COORD_ORDER TileSource::GetCoordOrder() const
  {
    return coord_order_;
  }

  void TileSource::SetCoordOrder(TileSource::COORD_ORDER coord_order)
  {
    coord_order_ = coord_order;
  }

  bool TileSource::IsCustom() const
  {
    return is_custom_;
  }

  void TileSource::SetCustom(bool is_custom)
  {
    is_custom_ = is_custom;
  }

  int32_t TileSource::GetMaxZoom() const
  {
    return max_zoom_;
  }

  void TileSource::SetMaxZoom(int32_t max_zoom)
  {
    max_zoom_ = max_zoom;
  }

  const QString& TileSource::GetName() const
  {
    return name_;
  }

  void TileSource::SetName(const QString& name)
  {
    name_ = name;
  }

  const QString& TileSource::GetSuffix() const
  {
    return suffix_;
  }

  void TileSource::SetSuffix(const QString& suffix)
  {
    suffix_ = suffix;
  }

  /**
   * Given a zoom level and x and y coordinates appropriate for the tile source's
   * projection, this will generate a URL that points to an image tile for that
   * location.
   *
   * @param[in] level The zoom level
   * @param[in] x The X coordinate of the tile
   * @param[in] y The Y coordinate of the tile
   * @return A URL that references that tile
   */
  std::string TileSource::GenerateTileUrl(int32_t level, int64_t x, int64_t y) const
  {
    std::stringstream url;
    url << base_url_.toStdString();

    switch (coord_order_)
    {
      case TileSource::XYZ:
      case TileSource::XZY:
        url << boost::lexical_cast<std::string>(x);
        break;
      case TileSource::YXZ:
      case TileSource::YZX:
        url << boost::lexical_cast<std::string>(y);
        break;
      case TileSource::ZXY:
      case TileSource::ZYX:
        url << boost::lexical_cast<std::string>(level);
        break;
    }
    url << "/";
    switch (coord_order_)
    {
      case TileSource::ZXY:
      case TileSource::YXZ:
        url << boost::lexical_cast<std::string>(x);
        break;
      case TileSource::ZYX:
      case TileSource::XYZ:
        url << boost::lexical_cast<std::string>(y);
        break;
      case TileSource::XZY:
      case TileSource::YZX:
        url << boost::lexical_cast<std::string>(level);
        break;
    }
    url << "/";
    switch (coord_order_)
    {
      case TileSource::YZX:
      case TileSource::ZYX:
        url << boost::lexical_cast<std::string>(x);
        break;
      case TileSource::ZXY:
      case TileSource::XZY:
        url << boost::lexical_cast<std::string>(y);
        break;
      case TileSource::YXZ:
      case TileSource::XYZ:
        url << boost::lexical_cast<std::string>(level);
        break;
    }
    url << suffix_.toStdString();

    return url.str();
  }
}
