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

#ifndef TILE_MAP_TILE_SOURCE_H
#define TILE_MAP_TILE_SOURCE_H

#include <QObject>
#include <QString>

namespace tile_map
{
  /**
   * Represents a network source for map tiles; contains information about how to
   * connect to the source and how to retrieve tiles from it.
   *
   * Implementations of this should be specific to a type of map server such as
   * WMS, WMTS, or TMS, and implement the appropriate methods for retrieiving tiles
   * from those servers.
   */
  class TileSource : public QObject
  {
  Q_OBJECT
  public:
    virtual ~TileSource() {};

    virtual const QString& GetBaseUrl() const;

    virtual void SetBaseUrl(const QString& base_url);

    virtual bool IsCustom() const;

    virtual bool IsReady() const { return is_ready_; };

    virtual void SetCustom(bool is_custom);

    virtual int32_t GetMaxZoom() const;

    virtual void SetMaxZoom(int32_t max_zoom);

    virtual int32_t GetMinZoom() const;

    virtual void SetMinZoom(int32_t min_zoom);

    virtual const QString& GetName() const;

    virtual void SetName(const QString& name);

    /**
     * Generates a hash that uniquely identifies the tile from this source at the
     * specified level and coordinates.
     * @param level The zoom level
     * @param x The X coordinate
     * @param y The Y coordinate
     * @return A hash identifying the tile
     */
    virtual size_t GenerateTileHash(int32_t level, int64_t x, int64_t y) = 0;

    /**
     * Generates an HTTP or HTTPS URL that refers to a map tile from this source
     * at the given level and x and y coordinates.
     * @param level The zoom level
     * @param x The x coordinate
     * @param y The y coordinate
     * @return A URL referring to the map tile
     */
    virtual QString GenerateTileUrl(int32_t level, int64_t x, int64_t y) = 0;

    /**
     * Returns a string identifying the type of map source ("wmts", "bing", etc.)
     * @return
     */
    virtual QString GetType() const = 0;

  Q_SIGNALS:
    void ErrorMessage(const std::string& error_msg) const;
    void InfoMessage(const std::string& info_msg) const;

  protected:
    TileSource() :
      is_custom_(false),
      is_ready_(true),
      max_zoom_(20),
      min_zoom_(0)
    {};

    QString base_url_;
    bool is_custom_;
    bool is_ready_;
    int32_t max_zoom_;
    int32_t min_zoom_;
    QString name_;
  };
}

#endif //TILE_MAP_TILE_SOURCE_H
