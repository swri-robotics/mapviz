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

#include <tile_map/elevation_set.hpp>

#include <QDir>
#include <QDirIterator>
#include <QFileInfo>
#include <QImage>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <string>
#include <utility>

#include "swri_math_util/constants.h"
#include "swri_transform_util/earth_constants.h"
#include "swri_transform_util/utm_util.h"
#include "yaml-cpp/yaml.h"
#include <tile_map/terrain.hpp>

namespace tile_map
{
namespace
{
const swri_transform_util::UtmUtil & Utm()
{
  static const swri_transform_util::UtmUtil utm;
  return utm;
}

void Fail(QString * error, const QString & why)
{
  if (error) {
    *error = why;
  }
}
}  // namespace

std::shared_ptr<ElevationSet> ElevationSet::Load(const QString & geo_path, QString * error)
{
  Fail(error, QString());
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(geo_path.toStdString());
  } catch (const std::exception & e) {
    Fail(error, QString("can't read it: ") + e.what());
    return nullptr;
  }
  if (!doc.IsMap() || !doc["encoding"]) {
    // Imagery for multires_image; not an error.
    return nullptr;
  }

  std::shared_ptr<ElevationSet> set(new ElevationSet());
  set->geo_path_ = geo_path;
  try {
    const std::string encoding = doc["encoding"].as<std::string>();
    if (encoding != "terrarium") {
      Fail(error, QString("unknown encoding \"%1\"").arg(QString::fromStdString(encoding)));
      return nullptr;
    }
    if (!doc["image_path"] || !doc["image_width"] || !doc["image_height"] || !doc["tiepoints"]) {
      Fail(error, "needs image_path, image_width, image_height and tiepoints");
      return nullptr;
    }
    const QString folder = QString::fromStdString(doc["image_path"].as<std::string>());
    set->tile_folder_ = QDir::isAbsolutePath(folder) ? folder :
      QFileInfo(geo_path).dir().filePath(folder);
    set->width_ = doc["image_width"].as<int64_t>();
    set->height_ = doc["image_height"].as<int64_t>();
    if (doc["tile_size"]) {
      set->tile_size_ = doc["tile_size"].as<int32_t>();
    }
    set->extension_ = doc["extension"] ?
      QString::fromStdString(doc["extension"].as<std::string>()) : QString("png");
    if (doc["attribution"]) {
      set->attribution_ = QString::fromStdString(doc["attribution"].as<std::string>());
    }
    if (set->width_ <= 0 || set->height_ <= 0 || set->tile_size_ <= 0) {
      Fail(error, "has no pixels");
      return nullptr;
    }

    const std::string projection =
      doc["projection"] ? doc["projection"].as<std::string>() : std::string("wgs84");
    if (projection == "utm") {
      if (!doc["utm_zone"] || !doc["utm_band"]) {
        Fail(error, "a utm set needs utm_zone and utm_band");
        return nullptr;
      }
      set->utm_ = true;
      set->utm_zone_ = doc["utm_zone"].as<int>();
      const std::string band = doc["utm_band"].as<std::string>();
      set->utm_band_ = band.empty() ? 'N' : band[0];
      if (set->utm_zone_ < 1 || set->utm_zone_ > 60) {
        Fail(error, "utm_zone must be 1 to 60");
        return nullptr;
      }
    } else if (projection != "wgs84" && projection != "geographic") {
      Fail(
        error, QString("unknown projection \"%1\"; use wgs84 or utm")
        .arg(QString::fromStdString(projection)));
      return nullptr;
    }

    // Least-squares affine fit from pixels to coordinates, about the mean of
    // the tiepoints so that large UTM coordinates keep their precision.
    std::vector<std::array<double, 4>> points;
    for (const YAML::Node & tiepoint : doc["tiepoints"]) {
      const YAML::Node point = tiepoint["point"];
      if (!point || point.size() != 4) {
        Fail(error, "each tiepoint needs a point: [x, y, X, Y]");
        return nullptr;
      }
      points.push_back(
        {point[0].as<double>(), point[1].as<double>(), point[2].as<double>(),
          point[3].as<double>()});
    }
    if (points.size() < 3) {
      Fail(error, "needs at least 3 tiepoints");
      return nullptr;
    }
    double mean[4] = {0.0, 0.0, 0.0, 0.0};
    for (const auto & p : points) {
      for (int i = 0; i < 4; ++i) {
        mean[i] += p[i] / static_cast<double>(points.size());
      }
    }
    double sxx = 0.0;
    double sxy = 0.0;
    double syy = 0.0;
    double sx[2] = {0.0, 0.0};
    double sy[2] = {0.0, 0.0};
    for (const auto & p : points) {
      const double x = p[0] - mean[0];
      const double y = p[1] - mean[1];
      sxx += x * x;
      sxy += x * y;
      syy += y * y;
      for (int c = 0; c < 2; ++c) {
        sx[c] += x * (p[2 + c] - mean[2 + c]);
        sy[c] += y * (p[2 + c] - mean[2 + c]);
      }
    }
    const double det = sxx * syy - sxy * sxy;
    if (!(std::abs(det) > 1e-12 * std::max(1.0, sxx * syy))) {
      Fail(error, "its tiepoints are all on one line");
      return nullptr;
    }
    for (int c = 0; c < 2; ++c) {
      const double a1 = (sx[c] * syy - sy[c] * sxy) / det;
      const double a2 = (sy[c] * sxx - sx[c] * sxy) / det;
      set->affine_[3 * c] = mean[2 + c] - a1 * mean[0] - a2 * mean[1];
      set->affine_[3 * c + 1] = a1;
      set->affine_[3 * c + 2] = a2;
    }
  } catch (const YAML::Exception & e) {
    Fail(error, QString("bad value: ") + e.what());
    return nullptr;
  }

  const int64_t largest = std::max(set->width_, set->height_);
  set->layer_count_ = largest <= set->tile_size_ ? 1 :
    static_cast<int32_t>(std::ceil(std::log2(static_cast<double>(largest) / set->tile_size_))) + 1;

  // Pixel size at the middle of the set.
  const double cx = 0.5 * static_cast<double>(set->width_);
  const double cy = 0.5 * static_cast<double>(set->height_);
  double lat0;
  double lon0;
  double lat_x;
  double lon_x;
  double lat_y;
  double lon_y;
  set->PixelToLatLon(cx, cy, lat0, lon0);
  set->PixelToLatLon(cx + 1.0, cy, lat_x, lon_x);
  set->PixelToLatLon(cx, cy + 1.0, lat_y, lon_y);
  const double meters_per_degree = swri_transform_util::_earth_equator_circumference / 360.0;
  const double cos_lat = std::cos(lat0 * swri_math_util::_deg_2_rad);
  set->pixel_width_m_ =
    meters_per_degree * std::hypot(lat_x - lat0, (lon_x - lon0) * cos_lat);
  set->pixel_height_m_ =
    meters_per_degree * std::hypot(lat_y - lat0, (lon_y - lon0) * cos_lat);
  if (!(set->pixel_width_m_ > 0.0) || !(set->pixel_height_m_ > 0.0)) {
    Fail(error, "its pixels have no size");
    return nullptr;
  }
  return set;
}

int64_t ElevationSet::Columns(int32_t layer) const
{
  const int64_t pixels = (width_ + (int64_t{1} << layer) - 1) >> layer;
  return (pixels + tile_size_ - 1) / tile_size_;
}

int64_t ElevationSet::Rows(int32_t layer) const
{
  const int64_t pixels = (height_ + (int64_t{1} << layer) - 1) >> layer;
  return (pixels + tile_size_ - 1) / tile_size_;
}

void ElevationSet::PixelToCoordinates(double x, double y, double & cx, double & cy) const
{
  cx = affine_[0] + affine_[1] * x + affine_[2] * y;
  cy = affine_[3] + affine_[4] * x + affine_[5] * y;
}

void ElevationSet::PixelToLatLon(double x, double y, double & latitude, double & longitude) const
{
  double cx;
  double cy;
  PixelToCoordinates(x, y, cx, cy);
  if (utm_) {
    Utm().ToLatLon(utm_zone_, utm_band_, cx, cy, latitude, longitude);
  } else {
    longitude = cx;
    latitude = cy;
  }
}

void ElevationSet::LatLonToPixel(double latitude, double longitude, double & x, double & y) const
{
  // Newton's method from the middle of the set: exact after one step for
  // wgs84 sets, and a few for UTM ones, whatever zone the point is in.
  x = 0.5 * static_cast<double>(width_);
  y = 0.5 * static_cast<double>(height_);
  const double step = 1.0;
  for (int i = 0; i < 6; ++i) {
    double lat;
    double lon;
    double lat_x;
    double lon_x;
    double lat_y;
    double lon_y;
    PixelToLatLon(x, y, lat, lon);
    PixelToLatLon(x + step, y, lat_x, lon_x);
    PixelToLatLon(x, y + step, lat_y, lon_y);
    // Jacobian of (lat, lon) by (x, y).
    const double a = (lat_x - lat) / step;
    const double b = (lat_y - lat) / step;
    const double c = (lon_x - lon) / step;
    const double d = (lon_y - lon) / step;
    const double det = a * d - b * c;
    if (det == 0.0 || !std::isfinite(det)) {
      return;
    }
    const double dlat = latitude - lat;
    const double dlon = longitude - lon;
    const double dx = (d * dlat - b * dlon) / det;
    const double dy = (a * dlon - c * dlat) / det;
    x += dx;
    y += dy;
    if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
      return;
    }
  }
}

bool ElevationSet::Contains(double latitude, double longitude) const
{
  double x;
  double y;
  LatLonToPixel(latitude, longitude, x, y);
  return x >= 0.0 && y >= 0.0 && x <= static_cast<double>(width_) &&
         y <= static_cast<double>(height_);
}

QString ElevationSet::TilePath(int32_t layer, int64_t row, int64_t column) const
{
  return QString("%1/layer%2/tile%3x%4.%5")
         .arg(tile_folder_).arg(layer)
         .arg(row, 5, 10, QChar('0'))
         .arg(column, 5, 10, QChar('0'))
         .arg(extension_);
}

std::shared_ptr<const std::vector<float>> ElevationSet::ReadTile(
  int32_t layer, int64_t row, int64_t column) const
{
  const TileKey key{layer, row, column};
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    const auto found = cache_.find(key);
    if (found != cache_.end()) {
      cache_order_.erase(std::find(cache_order_.begin(), cache_order_.end(), key));
      cache_order_.push_back(key);
      return found->second;
    }
  }

  std::shared_ptr<const std::vector<float>> heights;
  const QImage image(TilePath(layer, row, column));
  if (!image.isNull() && image.width() == tile_size_ && image.height() == tile_size_) {
    heights = std::make_shared<const std::vector<float>>(terrarium::DecodeImage(image));
  }

  std::lock_guard<std::mutex> lock(cache_mutex_);
  if (cache_.emplace(key, heights).second) {
    cache_order_.push_back(key);
    if (cache_order_.size() > CACHED_TILES) {
      cache_.erase(cache_order_.front());
      cache_order_.erase(cache_order_.begin());
    }
  }
  return heights;
}

void ElevationLibrary::SetDirectory(const QString & directory)
{
  QString expanded = directory.trimmed();
  if (expanded == "~" || expanded.startsWith("~/")) {
    expanded = QDir::homePath() + expanded.mid(1);
  }
  if (expanded != directory_) {
    directory_ = expanded;
    Rescan();
  }
}

void ElevationLibrary::Rescan()
{
  sets_.clear();
  problems_.clear();
  if (directory_.isEmpty() || !QDir(directory_).exists()) {
    return;
  }
  QDirIterator files(
    directory_, {"*.geo"}, QDir::Files,
    QDirIterator::Subdirectories | QDirIterator::FollowSymlinks);
  while (files.hasNext()) {
    const QString path = files.next();
    QString error;
    std::shared_ptr<ElevationSet> set = ElevationSet::Load(path, &error);
    if (set) {
      sets_.push_back(set);
    } else if (!error.isEmpty()) {
      problems_.push_back(path + ": " + error);
    }
  }
}

std::shared_ptr<const ElevationSet> ElevationLibrary::SetAt(
  double latitude, double longitude) const
{
  std::shared_ptr<const ElevationSet> best;
  for (const auto & set : sets_) {
    if (set->Contains(latitude, longitude) &&
      (!best || set->PixelWidthMeters() < best->PixelWidthMeters()))
    {
      best = set;
    }
  }
  return best;
}
}  // namespace tile_map
