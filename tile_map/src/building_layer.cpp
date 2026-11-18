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

#include <tile_map/building_layer.hpp>

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QNetworkRequest>
#include <QUrl>
#include <QUrlQuery>

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <utility>

#include <rclcpp/logging.hpp>
#include <tile_map/terrain.hpp>
#include "swri_math_util/constants.h"

namespace tile_map
{
namespace
{
const char DEFAULT_OVERPASS_URL[] = "https://overpass-api.de/api/interpreter";
const char STADIA_URL[] =
  "https://tiles.stadiamaps.com/data/openmaptiles/{level}/{x}/{y}.pbf?api_key={api_key}";

/// Cached areas older than this are downloaded again.
constexpr qint64 CACHE_MAX_AGE_DAYS = 30;
/// Overpass is a shared, free service: one request at a time.
constexpr int OVERPASS_CONCURRENT_REQUESTS = 1;
constexpr int STADIA_CONCURRENT_REQUESTS = 4;
/// How long to wait before asking again after the server said it was busy,
/// and after any other failure.
constexpr qint64 BUSY_RETRY_MS = 60 * 1000;
constexpr qint64 FAILED_RETRY_MS = 5 * 60 * 1000;
/// Overpass can take a minute on a busy day.
constexpr int REQUEST_TIMEOUT_MS = 120 * 1000;
/// Tiles kept in memory beyond the ones in view.
constexpr size_t MAX_KEPT_TILES = 64;

/// Attributes on each reply, so ReplyFinished() knows what it was for.
constexpr QNetworkRequest::Attribute TILE_X_ATTRIBUTE = QNetworkRequest::User;
constexpr auto TILE_Y_ATTRIBUTE =
  static_cast<QNetworkRequest::Attribute>(QNetworkRequest::User + 1);
constexpr auto GENERATION_ATTRIBUTE =
  static_cast<QNetworkRequest::Attribute>(QNetworkRequest::User + 2);

int64_t LongitudeToTileX(double longitude, int32_t level)
{
  const double n = std::pow(2.0, level);
  return static_cast<int64_t>(std::floor((longitude + 180.0) / 360.0 * n));
}

int64_t LatitudeToTileY(double latitude, int32_t level)
{
  const double n = std::pow(2.0, level);
  const double lat = latitude * swri_math_util::_deg_2_rad;
  return static_cast<int64_t>(
    std::floor((1.0 - std::asinh(std::tan(lat)) / swri_math_util::_pi) / 2.0 * n));
}
}  // namespace

BuildingLayer::BuildingLayer(rclcpp::Logger logger, const QString & cache_dir)
: logger_(logger),
  cache_dir_(cache_dir),
  overpass_url_(DEFAULT_OVERPASS_URL),
  network_(this)
{
  QObject::connect(
    &network_, &QNetworkAccessManager::finished, this, &BuildingLayer::ReplyFinished);
}

void BuildingLayer::SetSource(BuildingSource source)
{
  if (source != source_) {
    source_ = source;
    Forget();
  }
}

void BuildingLayer::SetStadiaApiKey(const QString & api_key)
{
  const QString key = api_key.trimmed();
  if (key != stadia_api_key_) {
    stadia_api_key_ = key;
    key_rejected_ = false;
    if (source_ == BuildingSource::Stadia) {
      Forget();
    }
  }
}

void BuildingLayer::SetOverpassUrl(const QString & url)
{
  const QString trimmed = url.trimmed().isEmpty() ? QString(DEFAULT_OVERPASS_URL) : url.trimmed();
  if (trimmed != overpass_url_) {
    overpass_url_ = trimmed;
    if (source_ == BuildingSource::Overpass) {
      Forget();
    }
  }
}

void BuildingLayer::Forget()
{
  // Requests already sent are left to finish, but their replies are ignored.
  ++generation_;
  in_flight_ = 0;
  queue_.clear();
  tiles_.clear();
  last_error_.clear();
  // Ask for the view again.
  for (const TileKey & tile : view_tiles_) {
    tiles_[tile].state = TileData::State::Queued;
    queue_.push_back(tile);
  }
  StartRequests();
}

std::vector<BuildingLayer::TileKey> BuildingLayer::TilesInView(
  double latitude, double longitude, double scale, int32_t width, int32_t height)
{
  std::vector<TileKey> tiles;
  if (scale <= 0.0 || scale > MAX_SCALE) {
    return tiles;
  }

  // Half the canvas diagonal around the centre, so a rotated view is covered.
  const double radius_m = 0.5 * scale * std::hypot(width, height);
  const double meters_per_degree = 111320.0;
  const double cos_lat = std::max(
    0.01, static_cast<double>(std::cos(latitude * swri_math_util::_deg_2_rad)));
  const double dlat = radius_m / meters_per_degree;
  const double dlon = radius_m / (meters_per_degree * cos_lat);

  const int64_t max_index = (int64_t{1} << TILE_LEVEL) - 1;
  auto clamp = [max_index](int64_t index) {return std::clamp(index, int64_t{0}, max_index);};
  const int64_t x0 = clamp(LongitudeToTileX(longitude - dlon, TILE_LEVEL));
  const int64_t x1 = clamp(LongitudeToTileX(longitude + dlon, TILE_LEVEL));
  const int64_t y0 = clamp(LatitudeToTileY(latitude + dlat, TILE_LEVEL));
  const int64_t y1 = clamp(LatitudeToTileY(latitude - dlat, TILE_LEVEL));

  if (static_cast<size_t>((x1 - x0 + 1) * (y1 - y0 + 1)) > MAX_TILES) {
    return tiles;
  }
  for (int64_t y = y0; y <= y1; ++y) {
    for (int64_t x = x0; x <= x1; ++x) {
      tiles.emplace_back(x, y);
    }
  }
  return tiles;
}

QString BuildingLayer::CachePath(
  const QString & cache_dir, BuildingSource source, const TileKey & tile)
{
  const bool overpass = source == BuildingSource::Overpass;
  return QString("%1/%2/%3/%4/%5.%6")
         .arg(cache_dir)
         .arg(overpass ? "overpass" : "stadia")
         .arg(TILE_LEVEL)
         .arg(tile.first)
         .arg(tile.second)
         .arg(overpass ? "json" : "pbf");
}

void BuildingLayer::SetView(
  double latitude, double longitude, double scale, int32_t width, int32_t height)
{
  ++frame_;
  view_tiles_ = TilesInView(latitude, longitude, scale, width, height);
  too_far_out_ = view_tiles_.empty();

  const qint64 now = QDateTime::currentMSecsSinceEpoch();
  for (const TileKey & key : view_tiles_) {
    auto found = tiles_.find(key);
    if (found == tiles_.end()) {
      TileData & tile = tiles_[key];
      tile.last_used = frame_;

      // Use the disk copy if there is a recent one.
      const QString path = CachePath(cache_dir_, source_, key);
      const QFileInfo info(path);
      QFile file(path);
      QString error;
      if (info.exists() &&
        info.lastModified().daysTo(QDateTime::currentDateTime()) < CACHE_MAX_AGE_DAYS &&
        file.open(QIODevice::ReadOnly) && LoadTile(key, file.readAll(), error))
      {
        continue;
      }
      tile.state = TileData::State::Queued;
      queue_.push_back(key);
      continue;
    }

    TileData & tile = found->second;
    tile.last_used = frame_;
    if (tile.state == TileData::State::Failed && now >= tile.retry_after_ms && !key_rejected_) {
      tile.state = TileData::State::Queued;
      queue_.push_back(key);
    }
  }

  // Forget tiles that have been out of view the longest.
  if (tiles_.size() > MAX_KEPT_TILES + view_tiles_.size()) {
    std::vector<std::pair<uint64_t, TileKey>> by_age;
    for (const auto & entry : tiles_) {
      if (entry.second.state == TileData::State::Loaded ||
        entry.second.state == TileData::State::Failed)
      {
        by_age.emplace_back(entry.second.last_used, entry.first);
      }
    }
    std::sort(by_age.begin(), by_age.end());
    for (size_t i = 0; i < by_age.size() && tiles_.size() > MAX_KEPT_TILES; ++i) {
      if (by_age[i].first < frame_) {
        tiles_.erase(by_age[i].second);
      }
    }
  }

  StartRequests();
}

QNetworkRequest BuildingLayer::MakeRequest(const TileKey & tile) const
{
  QUrl url;
  if (source_ == BuildingSource::Overpass) {
    const double north = TileRowLatitude(TILE_LEVEL, static_cast<double>(tile.second));
    const double south = TileRowLatitude(TILE_LEVEL, static_cast<double>(tile.second + 1));
    const double n = std::pow(2.0, TILE_LEVEL);
    const double west = static_cast<double>(tile.first) / n * 360.0 - 180.0;
    const double east = static_cast<double>(tile.first + 1) / n * 360.0 - 180.0;
    url = QUrl(overpass_url_);
    QUrlQuery query;
    query.addQueryItem("data", OverpassBuildingQuery(south, west, north, east));
    url.setQuery(query);
  } else {
    QString text(STADIA_URL);
    text.replace("{level}", QString::number(TILE_LEVEL));
    text.replace("{x}", QString::number(tile.first));
    text.replace("{y}", QString::number(tile.second));
    text.replace("{api_key}", stadia_api_key_);
    url = QUrl(text);
  }

  QNetworkRequest request(url);
  // The Overpass usage policy asks clients to identify themselves.
  request.setHeader(QNetworkRequest::UserAgentHeader, "mapviz tile_map (2.5D buildings)");
  request.setTransferTimeout(REQUEST_TIMEOUT_MS);
  request.setAttribute(TILE_X_ATTRIBUTE, QVariant::fromValue<qlonglong>(tile.first));
  request.setAttribute(TILE_Y_ATTRIBUTE, QVariant::fromValue<qlonglong>(tile.second));
  request.setAttribute(GENERATION_ATTRIBUTE, generation_);
  return request;
}

void BuildingLayer::StartRequests()
{
  if (source_ == BuildingSource::Stadia && (stadia_api_key_.isEmpty() || key_rejected_)) {
    return;
  }
  const int limit = source_ == BuildingSource::Overpass ?
    OVERPASS_CONCURRENT_REQUESTS : STADIA_CONCURRENT_REQUESTS;

  while (in_flight_ < limit && !queue_.empty()) {
    const TileKey key = queue_.front();
    queue_.pop_front();
    auto found = tiles_.find(key);
    // Only fetch what is still wanted: in view and not already on its way.
    if (found == tiles_.end() || found->second.state != TileData::State::Queued ||
      std::find(view_tiles_.begin(), view_tiles_.end(), key) == view_tiles_.end())
    {
      if (found != tiles_.end() && found->second.state == TileData::State::Queued) {
        tiles_.erase(found);
      }
      continue;
    }
    found->second.state = TileData::State::Loading;
    ++in_flight_;
    network_.get(MakeRequest(key));
  }
}

void BuildingLayer::ReplyFinished(QNetworkReply * reply)
{
  reply->deleteLater();
  if (reply->request().attribute(GENERATION_ATTRIBUTE).toInt() != generation_) {
    return;
  }
  in_flight_ = std::max(0, in_flight_ - 1);

  const TileKey key(
    reply->request().attribute(TILE_X_ATTRIBUTE).toLongLong(),
    reply->request().attribute(TILE_Y_ATTRIBUTE).toLongLong());
  auto found = tiles_.find(key);
  if (found == tiles_.end()) {
    StartRequests();
    return;
  }
  TileData & tile = found->second;
  const qint64 now = QDateTime::currentMSecsSinceEpoch();
  const int status = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
  // The key is part of a Stadia URL, so report failures without it.
  const QString url = reply->url().toString(QUrl::RemoveQuery);

  // Overpass answers 429 when it is rate limiting, and its front end answers
  // 502, 503 or 504 when the servers behind it are overloaded.
  const bool busy = status == 429 || status == 502 || status == 503 || status == 504;
  QString error;
  if (reply->error() != QNetworkReply::NoError) {
    // Qt's own error text repeats the whole URL, which for Overpass is the
    // query; keep the status line short.
    if ((status == 401 || status == 403) && source_ == BuildingSource::Stadia) {
      key_rejected_ = true;
      error = "Stadia rejected the API key for building data";
    } else if (busy) {
      error = "The building data server is busy; trying again in a minute";
    } else if (status > 0) {
      error = QString("Couldn't download building data (HTTP %1)").arg(status);
    } else {
      error = "Couldn't reach the building data server";
    }
  } else {
    const QByteArray data = reply->readAll();
    if (LoadTile(key, data, error)) {
      const QString path = CachePath(cache_dir_, source_, key);
      QDir().mkpath(QFileInfo(path).absolutePath());
      QFile file(path);
      if (file.open(QIODevice::WriteOnly)) {
        file.write(data);
      }
      last_error_.clear();
      Q_EMIT Updated();
      StartRequests();
      return;
    }
  }

  tile.state = TileData::State::Failed;
  tile.retry_after_ms = now + (busy ? BUSY_RETRY_MS : FAILED_RETRY_MS);
  last_error_ = error;
  RCLCPP_WARN(
    logger_, "tile_map: building data for %s failed: %s (%s)",
    url.toStdString().c_str(), error.toStdString().c_str(),
    reply->errorString().toStdString().c_str());
  Q_EMIT RequestFailed(url, error);
  StartRequests();
}

bool BuildingLayer::LoadTile(const TileKey & key, const QByteArray & data, QString & error)
{
  std::vector<Building> buildings = source_ == BuildingSource::Overpass ?
    ParseOverpassBuildings(data, &error) :
    DecodeVectorTileBuildings(data, TILE_LEVEL, key.first, key.second, &error);
  if (!error.isEmpty()) {
    return false;
  }

  TileData & tile = tiles_[key];
  tile.state = TileData::State::Loaded;
  tile.buildings = std::move(buildings);

  // Triangulate once, in a local frame where degrees of longitude and
  // latitude are about the same length; the triangles stay valid in any
  // frame the buildings are later drawn in.
  tile.triangles.clear();
  for (const Building & building : tile.buildings) {
    std::vector<double> points;
    const double cos_lat = std::cos(
      building.outline.front().latitude * swri_math_util::_deg_2_rad);
    for (const LatLon & point : building.outline) {
      points.push_back(point.longitude * cos_lat);
      points.push_back(point.latitude);
    }
    tile.triangles.push_back(Triangulate(points));
  }
  Project(tile);
  return true;
}

void BuildingLayer::SetTransform(const swri_transform_util::Transform & wgs84_to_target)
{
#ifdef TILE_MAP_HAVE_TRANSFORM_EQUALITY
  if (wgs84_to_target == transform_) {
    return;
  }
#endif
  transform_ = wgs84_to_target;
  for (auto & entry : tiles_) {
    if (entry.second.state == TileData::State::Loaded) {
      Project(entry.second);
    }
  }
}

void BuildingLayer::Project(TileData & tile)
{
  tile.projected.clear();
  tile.projected.reserve(tile.buildings.size());
  for (size_t b = 0; b < tile.buildings.size(); ++b) {
    const Building & building = tile.buildings[b];
    Projected projected;
    projected.id = building.id;
    projected.walls = building.walls;
    projected.triangles = tile.triangles[b];
    projected.height = building.height;
    projected.min_height = building.min_height;

    double area = 0.0;
    tf2::Vector3 sum(0.0, 0.0, 0.0);
    for (const LatLon & point : building.outline) {
      projected.outline.push_back(
        transform_ * tf2::Vector3(point.longitude, point.latitude, 0.0));
      sum += projected.outline.back();
    }
    for (size_t i = 0; i < projected.outline.size(); ++i) {
      const tf2::Vector3 & p = projected.outline[i];
      const tf2::Vector3 & q = projected.outline[(i + 1) % projected.outline.size()];
      area += p.x() * q.y() - q.x() * p.y();
    }
    projected.counterclockwise = area >= 0.0;
    projected.centroid = sum / static_cast<double>(projected.outline.size());
    tile.projected.push_back(std::move(projected));
  }
}

void BuildingLayer::Draw(double lean)
{
  if (!gl_initialized_) {
    initializeOpenGLFunctions();
    gl_initialized_ = true;
  }
  if (view_tiles_.empty()) {
    return;
  }

  // "Up" and "right" on screen, in the frame being drawn, from the
  // modelview matrix (column major), so the lean follows the screen however
  // the map is rotated.
  GLdouble modelview[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
  tf2::Vector3 up(modelview[1], modelview[5], 0.0);
  tf2::Vector3 right(modelview[0], modelview[4], 0.0);
  if (up.length() < 1e-9 || right.length() < 1e-9) {
    return;
  }
  up.normalize();
  right.normalize();

  // Overpass returns a building in every area it touches, so skip repeats.
  std::vector<const Projected *> visible;
  std::unordered_set<int64_t> seen;
  for (const TileKey & key : view_tiles_) {
    auto found = tiles_.find(key);
    if (found == tiles_.end() || found->second.state != TileData::State::Loaded) {
      continue;
    }
    for (const Projected & building : found->second.projected) {
      if (building.id == 0 || seen.insert(building.id).second) {
        visible.push_back(&building);
      }
    }
  }

  // Back to front: what is higher on the screen is farther away.
  std::sort(
    visible.begin(), visible.end(), [&up](const Projected * a, const Projected * b) {
      return a->centroid.dot(up) > b->centroid.dot(up);
    });

  for (const Projected * building : visible) {
    const tf2::Vector3 top = up * (lean * building->height);
    const tf2::Vector3 bottom = up * (lean * building->min_height);
    const size_t count = building->outline.size();

    // Walls facing down the screen, toward the viewer.  Those facing up are
    // behind the building and covered by its roof.
    glBegin(GL_QUADS);
    for (size_t i = 0; i < count; ++i) {
      if (!building->walls[i]) {
        continue;
      }
      const tf2::Vector3 & a = building->outline[i];
      const tf2::Vector3 & b = building->outline[(i + 1) % count];
      tf2::Vector3 edge = b - a;
      edge.setZ(0.0);
      if (edge.length() < 1e-6) {
        continue;
      }
      edge.normalize();
      // Outward normal: to the right of the edge on a counterclockwise outline.
      const tf2::Vector3 normal = building->counterclockwise ?
        tf2::Vector3(edge.y(), -edge.x(), 0.0) : tf2::Vector3(-edge.y(), edge.x(), 0.0);
      if (normal.dot(up) > 0.0) {
        continue;
      }
      // Walls facing the viewer are lighter than walls seen side on.
      const float tone = static_cast<float>(0.62 - 0.16 * std::abs(normal.dot(right)));
      glColor4f(tone, tone * 0.98f, tone * 0.95f, 1.0f);
      const tf2::Vector3 a0 = a + bottom;
      const tf2::Vector3 b0 = b + bottom;
      const tf2::Vector3 b1 = b + top;
      const tf2::Vector3 a1 = a + top;
      glVertex2d(a0.x(), a0.y());
      glVertex2d(b0.x(), b0.y());
      glVertex2d(b1.x(), b1.y());
      glVertex2d(a1.x(), a1.y());
    }
    glEnd();

    // The roof.
    glColor4f(0.84f, 0.83f, 0.80f, 1.0f);
    glBegin(GL_TRIANGLES);
    for (int index : building->triangles) {
      const tf2::Vector3 p = building->outline[static_cast<size_t>(index)] + top;
      glVertex2d(p.x(), p.y());
    }
    glEnd();

    // A thin edge around the roof keeps neighbouring buildings apart.
    glColor4f(0.45f, 0.44f, 0.42f, 1.0f);
    glBegin(GL_LINE_LOOP);
    for (const tf2::Vector3 & point : building->outline) {
      const tf2::Vector3 p = point + top;
      glVertex2d(p.x(), p.y());
    }
    glEnd();
  }
}

QString BuildingLayer::Attribution() const
{
  return source_ == BuildingSource::Overpass ?
         QString("Buildings: © OpenStreetMap contributors") :
         QString("Buildings: © Stadia Maps © OpenMapTiles © OpenStreetMap");
}

QString BuildingLayer::Status(bool & is_error) const
{
  is_error = false;
  if (source_ == BuildingSource::Stadia && stadia_api_key_.isEmpty()) {
    is_error = true;
    return "Stadia building data needs a Stadia API key (set it on a Stamen source)";
  }
  if (key_rejected_) {
    is_error = true;
    return "Stadia rejected the API key for building data";
  }
  if (too_far_out_) {
    return "Zoom in closer to see buildings";
  }
  size_t loading = 0;
  for (const TileKey & key : view_tiles_) {
    auto found = tiles_.find(key);
    if (found != tiles_.end() &&
      (found->second.state == TileData::State::Queued ||
      found->second.state == TileData::State::Loading))
    {
      ++loading;
    }
  }
  if (loading > 0) {
    return QString("Loading buildings (%1 of %2 areas left)").arg(loading).arg(view_tiles_.size());
  }
  if (!last_error_.isEmpty()) {
    is_error = true;
    return last_error_;
  }
  return {};
}
}  // namespace tile_map
