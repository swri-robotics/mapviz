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

#ifndef TILE_MAP_WMTS_SOURCE_H
#define TILE_MAP_WMTS_SOURCE_H

#include "tile_source.h"

#include <boost/functional/hash.hpp>

namespace tile_map
{
  class WmtsSource : public TileSource
  {
  Q_OBJECT
  public:
    /**
     * Creates a new tile source from a set of known parameters.
     *
     * @param[in] name A user-friendly display name
     * @param[in] base_url The base HTTP URL of the data source; e. g.:
     *   "http://tile.stamen.com/terrain/"
     * @param[in] is_custom If this is a custom (i. e. not one of the default)
     *   tile source; custom sources are saved and loaded from our settings
     * @param[in] max_zoom The maximum zoom level
     */
    explicit WmtsSource(const QString& name,
               const QString& base_url,
               bool is_custom,
               int32_t max_zoom);

    virtual size_t GenerateTileHash(int32_t level, int64_t x, int64_t y);

    /**
     * Given a zoom level and x and y coordinates appropriate for the tile source's
     * projection, this will generate a URL that points to an image tile for that
     * location.
     *
     * This expects the URL to have three strings in it, "{level}", "{x}", and "{y}",
     * which will be replaced with the passed values.  See tile_map_plugin.cpp for
     * example URLs.
     *
     * @param[in] level The zoom level
     * @param[in] x The X coordinate of the tile
     * @param[in] y The Y coordinate of the tile
     * @return A URL that references that tile
     */
    virtual QString GenerateTileUrl(int32_t level, int64_t x, int64_t y);

    virtual QString GetType() const;

    static const QString WMTS_TYPE;

  private:
    boost::hash<std::string> hash_;
  };
}

#endif //TILE_MAP_WMTS_SOURCE_H
