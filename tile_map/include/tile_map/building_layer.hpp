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

#ifndef TILE_MAP__BUILDING_LAYER_HPP_
#define TILE_MAP__BUILDING_LAYER_HPP_

#include <QByteArray>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QObject>
#include <QString>
#include <QtGlobal>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <tile_map/buildings.hpp>
#include <tile_map/scene_25d.hpp>
#include <tile_map/trees.hpp>
#include "swri_transform_util/transform.h"
#include <rclcpp/rclcpp.hpp>

namespace tile_map
{
enum class BuildingSource
{
  /// OpenStreetMap, queried through the Overpass API.  No key needed.
  Overpass,
  /// Stadia Maps' OpenMapTiles vector tiles.  Needs a Stadia API key.
  Stadia,
};

/**
 * Map data for tile_map's 2.5D rendering: buildings, and the trees that
 * OpenStreetMap maps (woods, forests and single trees).
 *
 * Data is fetched for z14 web mercator tiles around the view and kept on
 * disk, so each area is downloaded once.  Renderer25d draws what Collect()
 * returns.
 */
class BuildingLayer : public QObject
{
  Q_OBJECT

public:
  /// The web mercator level whose tiles buildings are fetched for.
  static constexpr int32_t TILE_LEVEL = 14;
  /// Buildings are not fetched or drawn when zoomed out past this many meters
  /// per pixel; they would be too small to see and too many to download.
  static constexpr double MAX_SCALE = 3.0;
  /// The most tiles fetched for one view.
  static constexpr size_t MAX_TILES = 16;

  using TileKey = std::pair<int64_t, int64_t>;

  explicit BuildingLayer(
    rclcpp::Logger logger = rclcpp::get_logger("tile_map::BuildingLayer"),
    const QString & cache_dir = "/tmp/tile_map_buildings");

  void SetLogger(rclcpp::Logger logger) {logger_ = logger;}

  void SetSource(BuildingSource source);
  BuildingSource Source() const {return source_;}

  void SetStadiaApiKey(const QString & api_key);

  /// The Overpass API endpoint, for using a mirror or a private instance.
  void SetOverpassUrl(const QString & url);
  const QString & OverpassUrl() const {return overpass_url_;}

  /// The area being drawn; fetches buildings for it that aren't loaded yet.
  void SetView(double latitude, double longitude, double scale, int32_t width, int32_t height);

  /// The transform from WGS84 into the frame being drawn.
  void SetTransform(const swri_transform_util::Transform & wgs84_to_target);

  /// Adds the buildings and mapped trees in view.  Overpass returns a
  /// building in every area it touches, so repeats are left out.
  void Collect(
    std::vector<const ProjectedBuilding *> * buildings,
    std::vector<const ProjectedCrown *> * crowns) const;

  /// Changes whenever what Collect() returns does.
  uint64_t Revision() const {return revision_;}

  /// Who to credit for the data: "© OpenStreetMap contributors" or Stadia's
  /// equivalent.
  QString Attribution() const;

  /// Progress or a problem to show in the status line; empty when there is
  /// nothing to say.
  QString Status(bool & is_error) const;

  /// The z14 tiles covering a view, or none when zoomed out too far.
  static std::vector<TileKey> TilesInView(
    double latitude, double longitude, double scale, int32_t width, int32_t height);

  /// Where a tile's buildings are kept on disk.
  static QString CachePath(const QString & cache_dir, BuildingSource source, const TileKey & tile);

Q_SIGNALS:
  /// A download failed, for the status line.
  void RequestFailed(QString url, QString error_string);
  /// New buildings are ready to draw.
  void Updated();

private Q_SLOTS:
  void ReplyFinished(QNetworkReply * reply);

private:
  struct TileData
  {
    enum class State {Queued, Loading, Loaded, Failed};
    State state = State::Queued;
    std::vector<Building> buildings;
    std::vector<std::vector<int>> triangles;
    std::vector<ProjectedBuilding> projected;
    /// Mapped trees: single trees, and trees spread through woods.
    std::vector<Crown> crowns;
    std::vector<ProjectedCrown> projected_crowns;
    /// When a failed tile may be requested again.
    qint64 retry_after_ms = 0;
    uint64_t last_used = 0;
  };

  void StartRequests();
  QNetworkRequest MakeRequest(const TileKey & tile) const;
  /// Reads a tile's data, from the network or the disk cache.  False if it
  /// can't be read, with `error` set.
  bool LoadTile(const TileKey & tile, const QByteArray & data, QString & error);
  void Project(TileData & tile);
  /// Drops every tile and asks for the view again, after the source changes.
  void Forget();

  rclcpp::Logger logger_;
  QString cache_dir_;
  BuildingSource source_ = BuildingSource::Overpass;
  QString stadia_api_key_;
  QString overpass_url_;

  QNetworkAccessManager network_;
  std::map<TileKey, TileData> tiles_;
  std::vector<TileKey> view_tiles_;
  std::deque<TileKey> queue_;
  int in_flight_ = 0;
  uint64_t frame_ = 0;
  /// Incremented whenever the source or key changes, so replies to requests
  /// made before the change are ignored.
  int generation_ = 0;

  swri_transform_util::Transform transform_;
  uint64_t revision_ = 1;

  QString last_error_;
  bool key_rejected_ = false;
  bool too_far_out_ = false;
};
}  // namespace tile_map

#endif  // TILE_MAP__BUILDING_LAYER_HPP_
