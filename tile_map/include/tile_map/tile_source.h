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

#include <QString>

namespace tile_map
{
  /**
   * Represents a network source for map tiles; contains information about how to
   * connect to the source and how to retrieve tiles from it.
   *
   * Currently this implementation is fairly WMTS-centric, but there's no reason
   * this couldn't be made a bit more generate for supporting other service formats
   * in the future.
   */
  class TileSource
  {
  public:
    enum COORD_ORDER
    {
      // The order of the listing here should match the order listed
      // in the combo box in tile_map_config.ui
      ZXY,
      ZYX,
      XYZ,
      XZY,
      YXZ,
      YZX
    };

    TileSource();

    TileSource(const QString& name,
               const QString& base_url,
               COORD_ORDER coord_order,
               bool is_custom,
               int32_t max_zoom,
               const QString& suffix
               );

    TileSource(const TileSource& tile_source);

    const QString& GetBaseUrl() const;

    void SetBaseUrl(const QString& base_url);

    COORD_ORDER GetCoordOrder() const;

    void SetCoordOrder(COORD_ORDER coord_order);

    bool IsCustom() const;

    void SetCustom(bool is_custom);

    int32_t GetMaxZoom() const;

    void SetMaxZoom(int32_t max_zoom);

    const QString& GetName() const;

    void SetName(const QString& name);

    const QString& GetSuffix() const;

    void SetSuffix(const QString& suffix);

    std::string GenerateTileUrl(int32_t level, int64_t x, int64_t y) const;

  private:
    QString base_url_;
    COORD_ORDER coord_order_;
    bool is_custom_;
    int32_t max_zoom_;
    QString name_;
    QString suffix_;
  };
}

#endif //TILE_MAP_TILE_SOURCE_H
