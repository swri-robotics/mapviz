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

#ifndef TILE_MAP_BING_SOURCE_H
#define TILE_MAP_BING_SOURCE_H

#include "tile_source.h"

#include <boost/functional/hash.hpp>
#include <boost/random.hpp>

#include <vector>

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QString>

namespace tile_map
{
  class BingSource : public TileSource
  {
    Q_OBJECT
  public:
    /**
     * Initializes a Bing map source with a given name.
     *
     * Note that currently, only a single, hard-coded Bing map source is supported.
     * There's only one Bing Maps, after all.
     * In the future, though, it would probably make sense to extend its
     * functionality to allow pulling different tile sets from Bing.
     * @param name The name the source will appear as in the combo box.
     */
    explicit BingSource(const QString& name);

    /**
     * Generates a unique hash that identifies the tile as the given coordinates.
     *
     * Note that Bing Maps tiles could potentially be pulled from one of many
     * different servers, depending on the subdomain list given to us after we
     * authenticate with our API Key.  That means the exact URL to any given tile
     * should not be used as part of the hash, because there are many valid URLs
     * for a tile.
     * @param level The zoom level
     * @param x The X coordinate
     * @param y The Y coordinate
     * @return A hash that uniquely identifies this tile
     */
    virtual size_t GenerateTileHash(int32_t level, int64_t x, int64_t y);

    /**
     * Generates a URL that will retrieve a tile for the given coordinates.
     *
     * Since Bing can give us a list of subdomains to pull tiles from, the
     * exact subdomain for a tile is chosen at random every time this function
     * is called.  That means you are not guaranteed to get the same URL for a
     * tile every time you call this function.
     * @param level The zoom level
     * @param x The X coordinate
     * @param y The Y coordinate
     * @return A URL that points to this tile
     */
    virtual QString GenerateTileUrl(int32_t level, int64_t x, int64_t y);

    virtual QString GetType() const;

    QString GetApiKey() const;

    /**
     * Bing requires an API key in order to access its tiles.  The key provided
     * will determine the URL we use to retrieve map tiles, so setting the API
     * key will also cause this object to make a network request to the Bing Map
     * server to get the appropriate URL.
     *
     * More information about getting an API key:
     * https://msdn.microsoft.com/en-us/library/ff428642.aspx
     * @param api_key A valid Bing Maps key
     */
    void SetApiKey(const QString& api_key);

    static const QString BING_TYPE;

  protected Q_SLOTS:
    void ReplyFinished(QNetworkReply* reply);

  protected:
    /**
     * Bing Maps identifies tiles using a quadkey that is generated from the zoom
     * level and x and y coordinates.  Details on how the quadkey is generated can
     * be found here:
     * https://msdn.microsoft.com/en-us/library/bb259689.aspx
     *
     * @param level The zoom level
     * @param x The X coordinate
     * @param y The Y coordinate
     * @return The quadkey that represents the tile at the requested location
     */
    QString GenerateQuadKey(int32_t level, int64_t x, int64_t y) const;

    QString api_key_;
    boost::hash<std::string> hash_;
    QNetworkAccessManager network_manager_;
    boost::random::mt19937 rng_;
    std::vector<QString> subdomains_;
    QString tile_url_;

    static const std::string BING_IMAGE_URL_KEY;
    static const std::string BING_IMAGE_URL_SUBDOMAIN_KEY;
    static const std::string BING_RESOURCE_SET_KEY;
    static const std::string BING_RESOURCE_KEY;
    static const std::string BING_STATUS_CODE_KEY;
  };
}

#endif //TILE_MAP_BING_SOURCE_H
