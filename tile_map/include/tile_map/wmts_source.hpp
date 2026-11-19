// *****************************************************************************
//
// Copyright (c) 2015-2025, Southwest Research Institute® (SwRI®)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef TILE_MAP__WMTS_SOURCE_HPP_
#define TILE_MAP__WMTS_SOURCE_HPP_

#include <QByteArray>
#include <QNetworkAccessManager>
#include <QObject>
#include <QString>

#include <cstddef>
#include <cstdint>
#include <string>

#include "tile_source.hpp"

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
  explicit WmtsSource(
    const QString & name,
    const QString & base_url,
    bool is_custom,
    int32_t max_zoom);

  virtual size_t GenerateTileHash(int32_t level, int64_t x, int64_t y);

  /**
   * Describes what is wrong with a base URL, if anything.
   *
   * Most tile servers document their URLs with "{z}" for the zoom level, so
   * that spelling is accepted as an alias for "{level}" rather than treated
   * as an error.  A URL with no coordinate placeholders at all resolves to
   * the same image for every tile, which is never what anyone wants, so it is
   * reported instead of being silently accepted.
   *
   * @param[in] base_url The URL to inspect
   * @return An empty string if the URL is usable, otherwise a message
   *   suitable for showing to the user
   */
  static QString ValidateBaseUrl(const QString & base_url);

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

  /**
   * For a built-in source served by ArcGIS (".../arcgis/rest/services/<name>/
   * MapServer/..."), fetches the service description once and lowers or
   * raises the maximum zoom to the deepest level the service says it has
   * cached.  Until the reply arrives, or if it never does, or if the level it
   * gives is outside [min zoom, MAX_SUPPORTED_ZOOM], the max zoom passed to
   * the constructor stays in effect.  Custom sources keep the max zoom the
   * user entered.
   */
  void RequestServiceLimits() override;

  /**
   * The URL of the ArcGIS service description for a tile URL, e.g.
   * ".../USGSImageryOnly/MapServer?f=json", or an empty string if the URL
   * is not an ArcGIS MapServer URL.
   */
  static QString ArcGisServiceInfoUrl(const QString & base_url);

  /**
   * The deepest tile level an ArcGIS MapServer description says is
   * available, or -1 if it doesn't say.  That is the level named by "maxLOD"
   * when the service sets one, otherwise the deepest level in
   * "tileInfo.lods" whose scale is no finer than "maxScale".
   *
   * A description with neither (a "maxScale" of 0 means "no limit") gives -1:
   * "tileInfo.lods" lists the levels of the tiling scheme, not the levels
   * that are actually cached, and many services list 24 levels while caching
   * far fewer.
   */
  static int32_t ParseArcGisMaxLevel(const QByteArray & service_info);

  static const QString WMTS_TYPE;

protected:
  /// Applies an ArcGIS service description fetched by RequestServiceLimits().
  void HandleServiceInfo(const QByteArray & service_info);

private:
  std::hash<std::string> hash_;
  /// Created on first use, so sources that never need it don't open one.
  QNetworkAccessManager * network_manager_ = nullptr;
  bool limits_requested_ = false;
};
}  // namespace tile_map

#endif  // TILE_MAP__WMTS_SOURCE_HPP_
