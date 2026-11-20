// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
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

#ifndef TILE_MAP__ELEVATION_SET_HPP_
#define TILE_MAP__ELEVATION_SET_HPP_

#include <QString>

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <tuple>
#include <vector>

namespace tile_map
{
/// Where tile_map's terrain shading and contour lines get heights.
enum class ElevationSource
{
  /// AWS Terrain Tiles, downloaded as needed.
  Online,
  /// An elevation set on disk where one covers the middle of the view,
  /// otherwise the online tiles.
  LocalThenOnline,
  /// Only elevation sets on disk.
  LocalOnly,
};

/**
 * Elevation stored on disk, for using tile_map's terrain shading and contour
 * lines without a network connection.
 *
 * A set is a .geo file and a folder of tiles laid out like multires_image's
 * imagery: `layer<i>/tile<row>x<column>.png`, where layer i has pixels 2^i
 * times the size of layer 0's.  Its tiles hold heights in the Terrarium
 * encoding, with alpha 0 where there is no data, and the .geo says so with
 * `encoding: terrarium`.  The tiepoints map pixel corners to longitude and
 * latitude (`projection: wgs84`) or to UTM easting and northing
 * (`projection: utm`, with `utm_zone` and `utm_band`).
 *
 * tile_map's make_elevation_set script writes these from AWS Terrain Tiles or
 * from elevation GeoTIFFs.
 */
class ElevationSet
{
public:
  /// Reads a .geo file.  Returns null, with the reason in `error`, if it
  /// isn't an elevation set or can't be used.
  static std::shared_ptr<ElevationSet> Load(const QString & geo_path, QString * error = nullptr);

  const QString & GeoPath() const {return geo_path_;}
  /// The credit line from the .geo file, or empty.
  const QString & Attribution() const {return attribution_;}

  /// Size of layer 0, in pixels.
  int64_t Width() const {return width_;}
  int64_t Height() const {return height_;}
  int32_t TileSize() const {return tile_size_;}
  int32_t LayerCount() const {return layer_count_;}

  /// Tile columns and rows in a layer.
  int64_t Columns(int32_t layer) const;
  int64_t Rows(int32_t layer) const;

  /// Latitude and longitude of a point given in layer 0 pixels, measured
  /// from the corner of the first pixel.
  void PixelToLatLon(double x, double y, double & latitude, double & longitude) const;

  /// The layer 0 pixel position of a latitude and longitude (the inverse of
  /// PixelToLatLon()).
  void LatLonToPixel(double latitude, double longitude, double & x, double & y) const;

  /// True if the point is within the set's pixels.
  bool Contains(double latitude, double longitude) const;

  /// Ground size of a layer 0 pixel in meters, across (x) and down (y), at
  /// the middle of the set.
  double PixelWidthMeters() const {return pixel_width_m_;}
  double PixelHeightMeters() const {return pixel_height_m_;}

  /// The heights in one tile, row by row (TileSize() squared of them, NaN
  /// where there is no data), or null if the tile has no file.  Safe to call
  /// from any thread; recent tiles are kept in memory.
  std::shared_ptr<const std::vector<float>> ReadTile(
    int32_t layer, int64_t row, int64_t column) const;

  /// The file a tile is read from.
  QString TilePath(int32_t layer, int64_t row, int64_t column) const;

  /// How many tiles ReadTile() keeps in memory.
  static constexpr size_t CACHED_TILES = 256;

private:
  ElevationSet() = default;

  /// Coordinates (longitude and latitude, or easting and northing) of a
  /// layer 0 pixel position.
  void PixelToCoordinates(double x, double y, double & cx, double & cy) const;

  QString geo_path_;
  QString tile_folder_;
  QString extension_;
  QString attribution_;
  int64_t width_ = 0;
  int64_t height_ = 0;
  int32_t tile_size_ = 256;
  int32_t layer_count_ = 1;
  bool utm_ = false;
  int utm_zone_ = 0;
  char utm_band_ = 'N';
  /// Pixel to coordinates: cx = a[0] + a[1] x + a[2] y, cy = a[3] + a[4] x + a[5] y.
  double affine_[6] = {0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  double pixel_width_m_ = 1.0;
  double pixel_height_m_ = 1.0;

  using TileKey = std::tuple<int32_t, int64_t, int64_t>;
  mutable std::mutex cache_mutex_;
  mutable std::map<TileKey, std::shared_ptr<const std::vector<float>>> cache_;
  /// Tiles in the cache, least recently read first.
  mutable std::vector<TileKey> cache_order_;
};

/**
 * The elevation sets in a folder and the folders under it.
 */
class ElevationLibrary
{
public:
  /// Looks for sets in `directory` (a leading ~ is the home folder).  An
  /// empty directory means no sets.
  void SetDirectory(const QString & directory);
  const QString & Directory() const {return directory_;}

  /// Looks through the folder again.
  void Rescan();

  /// The set with the smallest pixels that contains the point, or null.
  std::shared_ptr<const ElevationSet> SetAt(double latitude, double longitude) const;

  const std::vector<std::shared_ptr<const ElevationSet>> & Sets() const {return sets_;}

  /// .geo files that looked like elevation sets but couldn't be read, with
  /// why.
  const std::vector<QString> & Problems() const {return problems_;}

private:
  QString directory_;
  std::vector<std::shared_ptr<const ElevationSet>> sets_;
  std::vector<QString> problems_;
};
}  // namespace tile_map

#endif  // TILE_MAP__ELEVATION_SET_HPP_
